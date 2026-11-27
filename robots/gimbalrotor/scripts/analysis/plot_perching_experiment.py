#!/usr/bin/env python3
"""Figure of a perching experiment (a rosbag of perching_task.py with the whole-body controller) for the
paper: the load moving from the rotors onto the hook and back, and the robot held while it hangs,
against the free flight before the task.

  rosrun gimbalrotor plot_perching_experiment.py <bag> [--out DIR] [--name NAME]

The look follows the figures of ~/paper/ral2026 (LaTeX text, "(n) Title", two columns), the size the
text width of ~/paper/ar2026 (130 mm). Time 0 is when the hook contact is switched on ("[whole body]
contact hook_bar on" on /rosout, else the first nonzero force of the contact). Each panel has two
segments on a broken time axis: the free flight before the task (--hover, default the --hover-length s
ending 1 s before perching_task starts, the arms as they were then) and the task around the contact
(--range, default from 10 s before the switch on to 10 s after the switch off). Panels:
  (1) thrust     the commanded thrust of each rotor (four_axes/command, moving average), the front
                 rotors (on the side of the hook, --front) in blue, the rear ones in orange, and the
                 normal force of the hook contact in the whole-body QP (whole_body/contacts/<contact>/
                 force, z of the contact frame) in black
  (2) servo      |present current| of the joints of the supporting arm (servo/states, raw x 2.69 mA:
                 XM540-W150 / XH430-W350): the load the arm takes
  (3) position   the CoG - its target (debug/pose/pid: -err_p) in x, y, z
  (4) attitude   the attitude - its target in roll, pitch, yaw
The errors are centred on 0. The load ramp of the contact (on, off: --ramp) is shaded.

Statistics (printed and written to <name>.yaml), over the windows
  hover   the free flight before the task (--hover)
  flight  just before the contact, the robot at the bar (--flight, from the switch on)
  hang    after the ramp until the switch off (--hang, from the switch on)
the mean thrust of each rotor, front, rear, their sum and its reduction against hover and flight, the
mean |current| per joint, the mean, the std and the rms of each error.

Output: <out>/<name>.svg, .pdf, .png and .yaml (default: the directory of the bag, the name of the bag +
_perching). For the paper: --out ~/paper/ar2026/figs (svg for the Makefile of figs/)
"""

import argparse
import math
import os
import re

import numpy as np
import rosbag
import yaml

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402

# front rotors blue, rear ones orange (left solid, right dashed); joints tab10; x / roll red, y / pitch
# green, z / yaw blue as in ral2026
FRONT_COLOR, REAR_COLOR, CONTACT_COLOR = "#1f77b4", "#ff7f0e", "k"
JOINT_COLORS = ["#9467bd", "#8c564b", "#e377c2"]
RPY_COLORS = ["#D62728", "#2CA02C", "#1F77B4"]
AXES = ["x", "y", "z", "roll", "pitch", "yaw"]
CURRENT_UNIT = 2.69e-3  # [A] per raw unit of Present Current (XM540-W150, XH430-W350)
SHADE = "0.9"


def parse():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("bag")
    parser.add_argument("--ns", default="/gimbalrotor")
    parser.add_argument("--contact", default="hook_bar")
    parser.add_argument("--out", help="directory (default: that of the bag)")
    parser.add_argument("--name", help="file name without the extension")
    parser.add_argument("--range", type=float, nargs=2, metavar=("BEGIN", "END"),
                        help="[s] of the task segment from the switch on (default: -10 to 10 s after the switch off)")
    parser.add_argument("--hover", type=float, nargs=2, metavar=("BEGIN", "END"),
                        help="[s] from the start of the bag: the free flight segment and window (default: "
                        "--hover-length s ending 1 s before perching_task starts)")
    parser.add_argument("--hover-length", type=float, default=10.0)
    parser.add_argument("--no-hover", action="store_true", help="without the free flight segment")
    parser.add_argument("--hover-width", type=float, default=0.2,
                        help="the width of the free flight segment, a fraction of the panel (its own time scale)")
    parser.add_argument("--flight", type=float, nargs=2, default=[-8.0, -0.5], metavar=("BEGIN", "END"),
                        help="[s] the window before the contact, from the switch on")
    parser.add_argument("--hang", type=float, nargs=2, metavar=("BEGIN", "END"),
                        help="[s] the hanging window from the switch on (default: ramp + 10 s to the switch off - 0.5 s)")
    parser.add_argument("--ramp", type=float, default=5.0, help="[s] ramp_time of the contact")
    parser.add_argument("--front", type=int, nargs="+", default=[2, 3],
                        help="rotors (1-based) on the side of the hook (+x: 2, 3); the others are the rear")
    parser.add_argument("--servo", type=int, nargs="+", default=[4, 5, 6],
                        help="indices in servo/states of the supporting arm (front_arm_joint1..3)")
    parser.add_argument("--servo-names", nargs="+", default=["Joint 1", "Joint 2", "Joint 3"])
    parser.add_argument("--smooth", type=float, default=0.5, help="[s] moving average of thrust and current")
    parser.add_argument("--width", type=float, default=130.0, help="[mm] of the figure")
    parser.add_argument("--height", type=float, default=95.0, help="[mm] of the figure")
    parser.add_argument("--font-size", type=float, default=9.0)
    parser.add_argument("--no-yaw", action="store_true", help="leave yaw out of the attitude panel")
    return parser.parse_args()


def read(args):
    ns = args.ns.rstrip("/")
    topics = {
        "pid": ns + "/debug/pose/pid",
        "thrust": ns + "/four_axes/command",
        "force": "%s/whole_body/contacts/%s/force" % (ns, args.contact),
        "servo": ns + "/servo/states",
        "rosout": "/rosout",
    }
    data = {k: [] for k in topics if k != "rosout"}
    switches, task_start = [], None
    pattern = re.compile(r"\[whole body\] contact %s (on|off)" % re.escape(args.contact))
    with rosbag.Bag(os.path.expanduser(args.bag)) as bag:
        bag_start = bag.get_start_time()
        for topic, msg, stamp in bag.read_messages(topics=list(topics.values())):
            t = stamp.to_sec()
            if topic == topics["pid"]:
                data["pid"].append([t] + [-getattr(msg, axis).err_p for axis in AXES])  # measured - target
            elif topic == topics["thrust"]:
                data["thrust"].append([t] + list(msg.base_thrust))
            elif topic == topics["force"]:
                f = msg.wrench.force
                data["force"].append([t, f.x, f.y, f.z])
            elif topic == topics["servo"]:
                load = {s.index: s.load for s in msg.servos}
                if all(i in load for i in args.servo):
                    data["servo"].append([t] + [abs(load[i]) * CURRENT_UNIT for i in args.servo])
            elif topic == topics["rosout"]:
                if msg.name.endswith("aerial_robot_base_node"):
                    match = pattern.search(msg.msg)
                    if match:
                        switches.append((t, match.group(1) == "on"))
                elif msg.name.endswith("perching_task") and task_start is None and "State machine starting" in msg.msg:
                    task_start = t
    out = {k: np.array(v) for k, v in data.items()}
    for k, v in out.items():
        if len(v) == 0:
            raise RuntimeError("no messages of %s in the bag" % topics[k])
    return out, switches, task_start, bag_start


def switch_times(data, switches):
    """The first switch on and the first switch off after it [bag time]."""
    on = next((t for t, state in switches if state), None)
    off = next((t for t, state in switches if not state and on is not None and t > on), None)
    if on is None:  # from the force of the contact
        force = data["force"]
        loaded = np.linalg.norm(force[:, 1:], axis=1) > 1e-6
        if not loaded.any():
            raise RuntimeError("the contact is never on")
        on = force[np.argmax(loaded), 0]
        unloaded = (force[:, 0] > on) & ~loaded
        off = force[np.argmax(unloaded), 0] if unloaded.any() else None
    return on, off


def smooth(t, y, window):
    if window <= 0 or len(t) < 2:
        return y
    n = max(int(round(window / np.median(np.diff(t)))), 1)
    pad = np.pad(y, (n // 2, n - 1 - n // 2), mode="edge")
    return np.convolve(pad, np.ones(n) / n, mode="valid")


def window(arr, begin, end):
    return arr[(arr[:, 0] >= begin) & (arr[:, 0] <= end)]


def statistics(data, windows, args):
    """windows: {name: (begin, end) in bag time}"""
    n_rotor = data["thrust"].shape[1] - 1
    front = [i - 1 for i in args.front]
    rear = [i for i in range(n_rotor) if i not in front]
    stats = {"thrust_N": {}, "servo_current_abs_mean_A": {}, "error": {}}
    for w, (b, e) in windows.items():
        thrust = window(data["thrust"], b, e)[:, 1:]
        mean = thrust.mean(axis=0)
        stats["thrust_N"][w] = {
            "rotors": [float(v) for v in mean],
            "front": float(mean[front].sum()),
            "rear": float(mean[rear].sum()),
            "sum": float(mean.sum()),
        }
    for w in windows:
        if w != "hang" and "hang" in stats["thrust_N"]:
            stats["thrust_N"]["hang"]["reduction_from_%s_percent" % w] = 100.0 * (
                1.0 - stats["thrust_N"]["hang"]["sum"] / stats["thrust_N"][w]["sum"]
            )
    if "hang" in windows:
        stats["contact_normal_N"] = {"hang": float(np.mean(window(data["force"], *windows["hang"])[:, 3]))}
    for k, name in enumerate(args.servo_names):
        stats["servo_current_abs_mean_A"][name] = {
            w: float(np.mean(window(data["servo"], b, e)[:, k + 1])) for w, (b, e) in windows.items()
        }
    for k, axis in enumerate(AXES):
        scale, unit = (1000.0, "mm") if k < 3 else (180.0 / math.pi, "deg")
        stats["error"][axis] = {"unit": unit}
        for w, (b, e) in windows.items():
            v = window(data["pid"], b, e)[:, k + 1] * scale
            stats["error"][axis][w] = {
                "mean": float(np.mean(v)),
                "std": float(np.std(v)),
                "rms": float(np.sqrt(np.mean(v**2))),
            }
    return stats


def style(args):
    # the look of the figures of ~/paper/ral2026 (plot_dynleg_command.py)
    plt.rcParams.update(
        {
            "text.usetex": True,
            "font.family": "serif",
            "font.size": args.font_size,
            "axes.titlesize": args.font_size,
            "axes.labelsize": args.font_size,
            "xtick.labelsize": args.font_size - 1,
            "ytick.labelsize": args.font_size - 1,
            "legend.fontsize": args.font_size - 1,
            "legend.frameon": False,
            "legend.handletextpad": 0.3,
            "legend.labelspacing": 0.2,
            "legend.handlelength": 1.5,
            "legend.columnspacing": 0.8,
            "lines.linewidth": 0.8,
            "axes.linewidth": 0.6,
        }
    )


class Panel:
    """A panel of one or two segments sharing the y axis on a broken time axis."""

    def __init__(self, fig, spec, segments, ratio):
        n = len(segments)
        sub = spec.subgridspec(1, n, width_ratios=ratio[:n], wspace=0.06)
        self.axes = [fig.add_subplot(sub[0, 0])]
        for k in range(1, n):
            self.axes.append(fig.add_subplot(sub[0, k], sharey=self.axes[0]))
        for ax, (b, e) in zip(self.axes, segments):
            ax.set_xlim(b, e)
        if n > 1:
            left, right = self.axes
            left.spines["right"].set_visible(False)
            right.spines["left"].set_visible(False)
            right.tick_params(left=False, labelleft=False)
            marks = dict(marker=[(-1, -1.5), (1, 1.5)], markersize=4, linestyle="none", color="k", mec="k",
                         mew=0.6, clip_on=False)
            left.plot([1, 1], [0, 1], transform=left.transAxes, **marks)
            right.plot([0, 0], [0, 1], transform=right.transAxes, **marks)
        # the centre of the panel in the axes coordinates of the last segment (titles, labels)
        self.centre = 0.5 - 0.5 * (ratio[0] / ratio[1] if n > 1 else 0.0)

    def plot(self, t, y, **kw):
        for k, ax in enumerate(self.axes):
            ax.plot(t, y, **(kw if k == len(self.axes) - 1 else {key: v for key, v in kw.items() if key != "label"}))

    @property
    def main(self):
        return self.axes[-1]

    @property
    def first(self):
        return self.axes[0]


def plot(data, t_on, t_off, hover, args, stats, path):
    style(args)
    mm = 1 / 25.4
    fig = plt.figure(figsize=(args.width * mm, args.height * mm))
    grid = fig.add_gridspec(2, 2, hspace=0.45, wspace=0.32)
    if args.range:
        begin, end = args.range
    else:
        begin, end = -10.0, (t_off - t_on if t_off else data["pid"][-1, 0] - t_on) + 10.0
    segments = ([(hover[0] - t_on, hover[1] - t_on)] if hover else []) + [(begin, end)]
    ratio = [args.hover_width, 1.0 - args.hover_width] if hover else [1.0]
    # only what is shown, relative to the switch on: the axes scaled to it
    shown = {}
    for k, v in data.items():
        rel = v.copy()
        rel[:, 0] -= t_on
        mask = np.zeros(len(rel), bool)
        for b, e in segments:
            mask |= (rel[:, 0] >= b - 1.0) & (rel[:, 0] <= e + 1.0)
        shown[k] = rel[mask]
    panels = [Panel(fig, grid[r, c], segments, ratio) for r in range(2) for c in range(2)]

    def broken(arr):
        """NaN between the segments: no line across the break"""
        t = arr[:, 0]
        if len(segments) > 1:
            gap = np.where(np.diff(t) > 5.0)[0]
            arr = np.insert(arr, gap + 1, np.nan, axis=0)
        return arr

    for panel in panels:
        for ax in panel.axes:
            ax.axvspan(0.0, args.ramp, color=SHADE, lw=0, zorder=0)
            if t_off:
                ax.axvspan(t_off - t_on, t_off - t_on + args.ramp, color=SHADE, lw=0, zorder=0)
        if hover:  # no times on the free flight: its own scale, from another part of the bag
            panel.first.set_xticks([])
            panel.first.text(0.5, -0.04, "Free\nflight", transform=panel.first.transAxes, ha="center", va="top",
                             fontsize=args.font_size - 1, linespacing=0.9)
    for panel in panels[2:]:
        panel.main.set_xlabel("Time [s]")
        panel.main.xaxis.set_label_coords(panel.centre, -0.2)

    # (1) the thrust of each rotor (front blue, rear orange) and the contact force
    panel = panels[0]
    thrust = broken(shown["thrust"])
    t = thrust[:, 0]
    rear = [i for i in range(1, thrust.shape[1]) if i not in args.front]
    styles = {}
    for group, color in ((args.front, FRONT_COLOR), (rear, REAR_COLOR)):
        for k, i in enumerate(sorted(group)):
            styles[i] = dict(color=color, ls="-" if k == 0 else "--")
    for i in range(1, thrust.shape[1]):
        y = thrust[:, i].copy()
        valid = ~np.isnan(y)
        y[valid] = smooth(t[valid], y[valid], args.smooth)
        panel.plot(t, y, label="Rotor %d" % i, **styles[i])
    force = broken(shown["force"])
    panel.plot(force[:, 0], force[:, 3], color=CONTACT_COLOR, lw=0.8, label="Contact")
    panel.main.set_title(r"\textbf{(1) Thrusts and contact force}", x=panel.centre)
    panel.first.set_ylabel("Force [N]")
    top = max(np.nanmax(thrust[:, 1:]), np.nanmax(force[:, 3]))
    panel.first.set_ylim(0.0, 1.45 * top)
    order = sorted(args.front) + sorted(rear)
    handles = [Line2D([], [], **styles[i], label=("Front %d" if i in args.front else "Rear %d") % i)
               for i in order] + [Line2D([], [], color=CONTACT_COLOR, label="Contact")]
    panel.main.legend(handles=handles, loc="upper center", ncol=3, bbox_to_anchor=(panel.centre, 1.0))

    # (2) the servo current of the supporting arm
    panel = panels[1]
    servo = broken(shown["servo"])
    for k, name in enumerate(args.servo_names):
        y = servo[:, k + 1].copy()
        valid = ~np.isnan(y)
        y[valid] = smooth(servo[valid, 0], y[valid], args.smooth)
        panel.plot(servo[:, 0], y, color=JOINT_COLORS[k], label=name)
    panel.main.set_title(r"\textbf{(2) Arm servo current}", x=panel.centre)
    panel.first.set_ylabel("Current [A]")
    panel.first.set_ylim(0.0, 1.3 * np.nanmax(servo[:, 1:]))
    panel.main.legend(loc="upper center", ncol=len(args.servo_names), bbox_to_anchor=(panel.centre, 1.0))

    # (3), (4) the errors, centred on 0
    pid = broken(shown["pid"])
    for panel, columns, scale, title, unit in (
        (panels[2], range(3), 1000.0, "(3) CoG position error", "[mm]"),
        (panels[3], range(3, 6), 180.0 / math.pi, "(4) Attitude error", "[deg]"),
    ):
        peak = 0.0
        for c, k in enumerate(columns):
            if AXES[k] == "yaw" and args.no_yaw:
                continue
            y = pid[:, k + 1] * scale
            panel.plot(pid[:, 0], y, color=RPY_COLORS[c], label=AXES[k])
            peak = max(peak, np.nanmax(np.abs(y)))
        for ax in panel.axes:
            ax.axhline(0.0, color="k", lw=0.4, zorder=1)
        panel.first.set_ylim(-1.15 * peak, 1.15 * peak)
        panel.main.set_title(r"\textbf{%s}" % title, x=panel.centre)
        panel.first.set_ylabel("Error " + unit)
        panel.main.legend(loc="lower center", ncol=3, bbox_to_anchor=(panel.centre, 0.0))

    for panel in panels:
        for ax in panel.axes:
            ax.tick_params(direction="out", length=2.5, width=0.5)
    for ext in ("svg", "pdf", "png"):
        fig.savefig(path + "." + ext, dpi=300, bbox_inches="tight", pad_inches=0.01)
    plt.close(fig)


def main():
    args = parse()
    data, switches, task_start, bag_start = read(args)
    t_on, t_off = switch_times(data, switches)
    end = t_off if t_off else data["pid"][-1, 0]
    hover = None
    if not args.no_hover:
        if args.hover:
            hover = (bag_start + args.hover[0], bag_start + args.hover[1])
        elif task_start is not None:
            hover = (task_start - 1.0 - args.hover_length, task_start - 1.0)
        else:
            print("no perching_task start in /rosout: without the free flight (give --hover)")
    hang = args.hang if args.hang else (args.ramp + 10.0, end - t_on - 0.5)
    args.hang_abs = (t_on + hang[0], t_on + hang[1])
    windows = {}
    if hover:
        windows["hover"] = hover
    windows["flight"] = (t_on + args.flight[0], t_on + args.flight[1])
    windows["hang"] = args.hang_abs
    stats = statistics(data, windows, args)
    stats["bag"] = os.path.abspath(os.path.expanduser(args.bag))
    stats["windows_from_contact_on_s"] = {w: [float(b - t_on), float(e - t_on)] for w, (b, e) in windows.items()}
    stats["windows_from_bag_start_s"] = {w: [float(b - bag_start), float(e - bag_start)] for w, (b, e) in windows.items()}
    stats["contact_on_from_bag_start_s"] = float(t_on - bag_start)
    stats["contact_off_s"] = float(t_off - t_on) if t_off else None

    out = os.path.expanduser(args.out) if args.out else os.path.dirname(stats["bag"])
    name = args.name or os.path.splitext(os.path.basename(args.bag))[0] + "_perching"
    path = os.path.join(out, name)
    plot(data, t_on, t_off, hover, args, stats, path)
    with open(path + ".yaml", "w") as f:
        yaml.safe_dump(stats, f, sort_keys=False)

    print("contact on at %.1f s of the bag, off %s s later" % (t_on - bag_start, "%.1f" % stats["contact_off_s"] if t_off else "-"))
    for w, (b, e) in stats["windows_from_bag_start_s"].items():
        print("window %-6s %6.1f..%6.1f s of the bag" % (w, b, e))
    for w, th in stats["thrust_N"].items():
        extra = "".join(" (-%.1f %% from %s)" % (v, k.split("_")[2]) for k, v in th.items() if k.startswith("reduction"))
        print("thrust %-6s rotors %s front %.2f rear %.2f sum %.2f N%s"
              % (w, " ".join("%.2f" % v for v in th["rotors"]), th["front"], th["rear"], th["sum"], extra))
    for name, c in stats["servo_current_abs_mean_A"].items():
        print("|current| %s: %s" % (name, ", ".join("%s %.3f A" % (w, v) for w, v in c.items())))
    print("error        " + "".join("%-22s" % ("%s mean/std/rms" % w) for w in windows))
    for axis, e in stats["error"].items():
        print("%-6s [%-3s] " % (axis, e["unit"]) + "".join(
            "%6.2f %6.2f %6.2f   " % tuple(e[w][k] for k in ("mean", "std", "rms")) for w in windows))
    print("written %s.{svg,pdf,png,yaml}" % path)


if __name__ == "__main__":
    main()
