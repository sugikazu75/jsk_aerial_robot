#!/usr/bin/env python3
"""Compare runs of hand_accuracy_test.py (e.g. hover and perching): the accuracy of the hand and the
thrust of the rotors per target, over the MEASURE phase of every repetition (the return to the start,
rep -1, left out).

  rosrun gimbalrotor compare_hand_accuracy.py ~/.ros/hand_accuracy/<hover>.csv ~/.ros/hand_accuracy/<perching>.csv

Per run and target (and "all": every target together):
  err      the measured hand - the requested target (u, z): the total error         [mm]
  ik_err   the hand of the IK - the target: what the IK could not reach              [mm]
  ctrl_err the measured hand - the hand of the IK: the error of the control          [mm]
  for each: rms (of the norm), bias (norm of the mean), std (about the mean); lateral: rms of the
  distance from the plane; angle: rms of the error of the direction [deg]
  thrust   the sum of the commanded thrusts (mean, std) and each rotor (mean)        [N]
  contact  the normal force of the hook contact of the whole-body QP (mean)          [N]
  torque_ratio_max  the largest |torque| / limit of the joints in the QP statics (1: at the limit,
           the QP then moves load from the contact to the rotors), torque_joint_max that joint,
           torque_saturated the share of the samples at the limit (> 0.99), qp_failed the share of
           the samples in which the QP failed (the usual allocation, the contact not considered)

Output (--out, default the directory of the first file): compare_hand_accuracy.csv (the table) and
compare_hand_accuracy.png (the plane, the error and the thrust per target). No ROS needed.
"""

import argparse
import csv
import os

import numpy as np
import yaml

COLORS = ["#257de0", "#f47230", "#00b894", "#f2a600", "#ee7fac", "#008a17", "#6a4ce2", "#ec4647"]


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    data = {k: np.array([r[k] for r in rows]) for k in rows[0]} if rows else {}
    for k in data:
        if k != "phase":
            data[k] = data[k].astype(float)
    meta_path = os.path.splitext(path)[0] + ".yaml"
    meta = {}
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = yaml.safe_load(f) or {}
    label = meta.get("condition", os.path.basename(path))
    if meta.get("label"):
        label += " (%s)" % meta["label"]
    return data, meta, label


def stats(err):
    """rms of the norm, norm of the mean, std about the mean of error vectors (n, 2)."""
    if len(err) == 0:
        return np.nan, np.nan, np.nan
    norm = np.linalg.norm(err, axis=1)
    mean = err.mean(axis=0)
    return (
        float(np.sqrt(np.mean(norm**2))),
        float(np.linalg.norm(mean)),
        float(np.sqrt(np.mean(np.sum((err - mean) ** 2, axis=1)))),
    )


def summarize(data, n_rotor):
    sel = (data["phase"] == "measure") & (data["rep"] >= 0)
    indices = sorted(set(data["index"][sel].astype(int)))
    out = []
    for index in indices + ["all"]:
        m = sel if index == "all" else sel & (data["index"] == index)
        goal = np.c_[data["goal_u"][m], data["goal_z"][m]]
        ik = np.c_[data["ik_u"][m], data["ik_z"][m]]
        meas = np.c_[data["meas_u"][m], data["meas_z"][m]]
        r = {"target": index, "samples": int(m.sum())}
        if index != "all" and "goal_x_world" in data:
            r["goal_x"], r["goal_z"] = float(data["goal_x_world"][m][0]), float(data["goal_z_world"][m][0])
        for name, err in (("err", meas - goal), ("ik_err", ik - goal), ("ctrl_err", meas - ik)):
            rms, bias, std = stats(1e3 * err)
            r[name + "_rms"], r[name + "_bias"], r[name + "_std"] = rms, bias, std
        r["lateral_rms"] = float(1e3 * np.sqrt(np.mean(data["meas_lateral"][m] ** 2)))
        angle = np.angle(np.exp(1j * (data["meas_angle"][m] - data["goal_angle"][m])))
        r["angle_rms"] = float(np.degrees(np.sqrt(np.mean(angle**2))))
        r["thrust_sum_mean"] = float(np.mean(data["thrust_sum"][m]))
        r["thrust_sum_std"] = float(np.std(data["thrust_sum"][m]))
        for i in range(n_rotor):
            r["thrust_%d_mean" % i] = float(np.mean(data["thrust_%d" % i][m]))
        tau = [k for k in data if k.startswith("tau_") and not k.startswith("tau_limit_")]
        if tau and m.any():
            ratio = np.column_stack([np.abs(data[k][m]) / data["tau_limit_" + k[4:]][m] for k in tau])
            worst = np.nanmax(ratio, axis=1) if np.any(np.isfinite(ratio)) else np.full(int(m.sum()), np.nan)
            r["torque_ratio_max"] = float(np.nanmax(worst)) if np.any(np.isfinite(worst)) else np.nan
            r["torque_joint_max"] = tau[int(np.nanargmax(np.nanmax(ratio, axis=0)))][4:] if np.any(np.isfinite(ratio)) else ""
            r["torque_saturated"] = float(np.mean(worst > 0.99)) if np.any(np.isfinite(worst)) else np.nan
        if "qp_ok" in data:
            r["qp_failed"] = float(1.0 - np.mean(data["qp_ok"][m])) if m.any() else np.nan
        r["contact_fz_mean"] = float(np.nanmean(data["contact_fz"][m])) if np.any(np.isfinite(data["contact_fz"][m])) else np.nan
        out.append(r)
    return out


def plot(runs, path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    text, grid = "#191919", "#e0e0e0"
    plt.rcParams.update(
        {
            "font.size": 9,
            "axes.edgecolor": grid,
            "axes.labelcolor": text,
            "xtick.color": "#666666",
            "ytick.color": "#666666",
            "axes.spines.top": False,
            "axes.spines.right": False,
        }
    )
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    ax_plane, ax_err, ax_thrust = axes

    # the error in the plane: the measured hand - the target (MEASURE), every target together
    first = runs[0]
    for k, run in enumerate(runs):
        d = run["data"]
        m = (d["phase"] == "measure") & (d["rep"] >= 0)
        du, dz = 1e3 * (d["meas_u"][m] - d["goal_u"][m]), 1e3 * (d["meas_z"][m] - d["goal_z"][m])
        ax_plane.scatter(du, dz, s=8, color=COLORS[k % 8], alpha=0.3, linewidths=0)
        ax_plane.scatter([np.mean(du)], [np.mean(dz)], s=70, marker="o", color=COLORS[k % 8], edgecolors="white", linewidths=2, zorder=5, label=run["label"] + " (mean)")
    ax_plane.axhline(0, color="#666666", linewidth=0.8)
    ax_plane.axvline(0, color="#666666", linewidth=0.8)
    ax_plane.set_aspect("equal", adjustable="datalim")
    ax_plane.set_xlabel("error along the heading u [mm]")
    ax_plane.set_ylabel("error in z [mm]")
    ax_plane.set_title("error of the hand in the plane", color=text, loc="left")
    ax_plane.grid(color=grid, linewidth=0.6)
    ax_plane.legend(frameon=False, fontsize=8)

    # per target: the RMS error, the mean total thrust
    targets = [r["target"] for r in first["summary"] if r["target"] != "all"] + ["all"]
    x = np.arange(len(targets))
    goal_of = {r["target"]: r for r in first["summary"]}
    ticks = ["(%.2f, %.2f)" % (goal_of[t]["goal_x"], goal_of[t]["goal_z"]) if "goal_x" in goal_of.get(t, {}) else str(t) for t in targets]
    width = 0.8 / len(runs)
    for k, run in enumerate(runs):
        by = {r["target"]: r for r in run["summary"]}
        err = [by.get(t, {}).get("err_rms", np.nan) for t in targets]
        ctrl = [by.get(t, {}).get("ctrl_err_rms", np.nan) for t in targets]
        thrust = [by.get(t, {}).get("thrust_sum_mean", np.nan) for t in targets]
        thrust_std = [by.get(t, {}).get("thrust_sum_std", np.nan) for t in targets]
        offset = (k - (len(runs) - 1) / 2) * width
        ax_err.bar(x + offset, err, width * 0.9, color=COLORS[k % 8], label=run["label"])
        ax_err.scatter(x + offset, ctrl, s=120, marker="_", linewidths=2.5, color=text, zorder=5, label="control part (vs IK)" if k == 0 else None)
        ax_thrust.bar(x + offset, thrust, width * 0.9, yerr=thrust_std, color=COLORS[k % 8], ecolor="#666666", capsize=2, label=run["label"])
    for ax, label, title in (
        (ax_err, "RMS error of the hand [mm]", "accuracy per target"),
        (ax_thrust, "sum of the commanded thrust [N]", "thrust per target (mean, std)"),
    ):
        ax.set_xticks(x)
        ax.set_xticklabels(ticks, fontsize=7)
        ax.set_xlabel("target (x, z) in the world [m]")
        ax.set_ylabel(label)
        ax.set_title(title, color=text, loc="left")
        ax.grid(axis="y", color=grid, linewidth=0.6)
        ax.set_axisbelow(True)
        ax.legend(frameon=False, fontsize=8)
    fig.tight_layout()
    fig.savefig(path, dpi=150)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("files", nargs="+", help="CSV files of hand_accuracy_test.py")
    parser.add_argument("--out", help="output directory (default: that of the first file)")
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    runs = []
    for path in args.files:
        data, meta, label = load(path)
        if not data:
            raise SystemExit("%s: no rows" % path)
        n_rotor = int(meta.get("n_rotor", sum(1 for k in data if k.startswith("thrust_") and k != "thrust_sum")))
        runs.append({"path": path, "data": data, "meta": meta, "label": label, "n_rotor": n_rotor, "summary": summarize(data, n_rotor)})

    out = os.path.expanduser(args.out or os.path.dirname(os.path.abspath(args.files[0])))
    os.makedirs(out, exist_ok=True)
    keys = []
    for run in runs:
        for r in run["summary"]:
            keys += [k for k in r if k not in keys]
    with open(os.path.join(out, "compare_hand_accuracy.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["run"] + keys)
        writer.writeheader()
        for run in runs:
            for r in run["summary"]:
                writer.writerow({"run": run["label"], **r})

    print("%-22s %6s %13s %8s %8s %8s %8s %8s %8s %9s %9s %7s %7s %7s" % ("run", "target", "(x, z) [m]", "err_rms", "bias", "std", "ik_err", "ctrl", "lateral", "thrust", "contact", "tau_max", "tau_sat", "qp_fail"))
    for run in runs:
        for r in run["summary"]:
            print(
                "%-22s %6s %13s %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f %9.2f %9.2f %7.2f %7.2f %7.2f"
                % (run["label"][:22], r["target"], "(%.2f, %.2f)" % (r["goal_x"], r["goal_z"]) if "goal_x" in r else "", r["err_rms"], r["err_bias"], r["err_std"], r["ik_err_rms"], r["ctrl_err_rms"], r["lateral_rms"], r["thrust_sum_mean"], r["contact_fz_mean"],
                   r.get("torque_ratio_max", np.nan), r.get("torque_saturated", np.nan), r.get("qp_failed", np.nan))
            )
    print("[mm], [N]; tau_max: |torque|/limit, tau_sat, qp_fail: share of the samples; the table: %s" % os.path.join(out, "compare_hand_accuracy.csv"))
    if not args.no_plot:
        plot(runs, os.path.join(out, "compare_hand_accuracy.png"))
        print("the plot: %s" % os.path.join(out, "compare_hand_accuracy.png"))


if __name__ == "__main__":
    main()
