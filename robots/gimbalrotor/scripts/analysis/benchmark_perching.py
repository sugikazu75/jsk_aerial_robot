#!/usr/bin/env python3
"""Benchmark the wall time of the perching motion of quad_with_dual_arm: the planning
(scripts/planning/plan_perching_motion.py) and its trajectory optimisation
(aerial_to_solver/scripts/optimize_trajectory.py), stage by stage, over several RRT seeds.

  rosrun gimbalrotor benchmark_perching.py --runs 10
  rosrun gimbalrotor benchmark_perching.py --runs 20 --skip-optimize
  rosrun gimbalrotor benchmark_perching.py --optimize-only config/quad_with_dual_arm/trajectories/perching_motion.json

Each run is a separate process (so that every run pays the setup), with the seed --seed + k; the files
go to --out-dir (a temporary directory by default), never over the trajectories of the package. The
scripts print their stages as one line "timing: {...}", which is read here:

  planning      setup     xacro, model (pinocchio), collision_setup (the collision scene)
                ik        ik_hang, ik_reach_touch, ik_unhook (the contact postures, scipy least squares)
                rrt       rrt_search (OMPL RRTConnect, the state checks included), rrt_simplify
                others    misc, phase_collision (collision check of every frame), statics, output
  optimisation  setup     load, model, problem_build (the shooting problem and the initial guess),
                          solver_setup (SolverHpipmSQP and the cost of the initial guess)
                solve     solve (the SQP iterations)
                others    output
"""

import argparse
import json
import os
import subprocess
import sys
import tempfile

import numpy as np
import rospkg

GROUPS = {
    "plan": {
        "setup": ("xacro", "model", "collision_setup"),
        "ik": ("ik_hang", "ik_reach_touch", "ik_unhook"),
        "rrt": ("rrt_search", "rrt_simplify"),
    },
    "optimize": {
        "setup": ("load", "model", "problem_build", "solver_setup"),
    },
}


def run(command, log_path):
    """Run a script, keep its output in log_path and return (timing dict or None, return code)."""
    process = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    with open(log_path, "w") as f:
        f.write(process.stdout)
    timing = None
    for line in process.stdout.splitlines():
        if line.startswith("timing: "):
            timing = json.loads(line[len("timing: "):])
    return timing, process.returncode


def with_groups(kind, timing):
    """The stages and the sums of their groups ("[setup]", ...)."""
    times = dict(timing["times"])
    for group, stages in GROUPS[kind].items():
        times["[%s]" % group] = sum(times.get(s, 0.0) for s in stages)
    return times


def summarize(title, results):
    """Statistics of each stage over the runs that finished."""
    done = [r for r in results if r["timing"] is not None]
    print("\n=== %s: %d of %d runs finished" % (title, len(done), len(results)))
    if not done:
        return {}
    names = list(dict.fromkeys(name for r in done for name in r["times"]))
    stats = {}
    print("%-18s %9s %9s %9s %9s %9s" % ("stage [s]", "mean", "std", "median", "min", "max"))
    for name in names:
        values = np.array([r["times"].get(name, 0.0) for r in done])
        stats[name] = dict(mean=values.mean(), std=values.std(), median=np.median(values), min=values.min(),
                           max=values.max())
        print("%-18s %9.3f %9.3f %9.3f %9.3f %9.3f" % ((name,) + tuple(stats[name].values())))
    counts = list(dict.fromkeys(name for r in done for name in r["timing"]["counts"]))
    for name in counts:
        values = np.array([float(r["timing"]["counts"][name]) for r in done])
        print("%-18s %9.1f %9.1f %9.1f %9.1f %9.1f" % (name, values.mean(), values.std(), np.median(values),
                                                     values.min(), values.max()))
    return stats


def main():
    package = rospkg.RosPack().get_path("gimbalrotor")
    solver_package = rospkg.RosPack().get_path("aerial_to_solver")
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--seed", type=int, default=1, help="the seed of the first run (then +1 per run)")
    parser.add_argument("--config", help="the planning parameters (default: those of the planning script)")
    parser.add_argument("--plan-args", default="", help='more options of the planning, e.g. "--time 30"')
    parser.add_argument("--optimize-args", default="", help="more options of the optimisation")
    parser.add_argument("--skip-optimize", action="store_true", help="the planning only")
    parser.add_argument("--optimize-only", metavar="TRAJECTORY",
                        help="the optimisation only, --runs times on this trajectory file")
    parser.add_argument("--out-dir", help="the trajectories, the logs and results.json (default: a temporary one)")
    args = parser.parse_args()

    out_dir = os.path.abspath(args.out_dir or tempfile.mkdtemp(prefix="benchmark_perching_"))
    os.makedirs(out_dir, exist_ok=True)
    planner = os.path.join(package, "scripts", "planning", "plan_perching_motion.py")
    optimizer = os.path.join(solver_package, "scripts", "optimize_trajectory.py")
    print("output: %s" % out_dir)

    plans, optimizations = [], []
    for k in range(args.runs):
        seed = args.seed + k
        if args.optimize_only:
            trajectory = os.path.abspath(args.optimize_only)
        else:
            trajectory = os.path.join(out_dir, "plan_%d.json" % seed)
            command = [sys.executable, planner, "--seed", str(seed), "--output", trajectory, "--no-plot"]
            command += ["--config", args.config] if args.config else []
            timing, code = run(command + args.plan_args.split(), os.path.join(out_dir, "plan_%d.log" % seed))
            entry = dict(seed=seed, returncode=code, timing=timing)
            if timing is not None:
                entry["times"] = with_groups("plan", timing)
            plans.append(entry)
            print("plan     seed %3d: %s" % (seed, "%.2f s (rrt %.2f s, ik %.2f s, setup %.2f s)" % tuple(
                entry["times"][n] for n in ("total", "[rrt]", "[ik]", "[setup]")) if timing else
                "failed (code %d), see plan_%d.log" % (code, seed)))
            if timing is None:
                continue
        if args.skip_optimize:
            continue
        command = [sys.executable, optimizer, trajectory, "--output",
                   os.path.join(out_dir, "optimized_%d.json" % seed), "--no-plot"]
        timing, code = run(command + args.optimize_args.split(), os.path.join(out_dir, "optimize_%d.log" % seed))
        entry = dict(seed=seed, returncode=code, timing=timing)
        if timing is not None:
            entry["times"] = with_groups("optimize", timing)
        optimizations.append(entry)
        print("optimize seed %3d: %s" % (seed, "%.2f s (solve %.2f s, %d iterations, %s; setup %.2f s)" % (
            entry["times"]["total"], entry["times"]["solve"], timing["counts"]["iterations"],
            "converged" if timing["counts"]["converged"] else "stopped", entry["times"]["[setup]"]) if timing else
            "failed (code %d), see optimize_%d.log" % (code, seed)))

    stats = {}
    if plans:
        stats["plan"] = summarize("planning", plans)
    if optimizations:
        stats["optimize"] = summarize("optimisation", optimizations)
    path = os.path.join(out_dir, "results.json")
    with open(path, "w") as f:
        json.dump(dict(plan=plans, optimize=optimizations, stats=stats), f, indent=1)
    print("\nwrote %s" % path)


if __name__ == "__main__":
    main()
