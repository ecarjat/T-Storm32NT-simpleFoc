#!/usr/bin/env python3
"""
Check monotonicity of angle samples over time, accounting for 0..2π wrap and direction.

Given a CSV with columns: time, target, velocity, angle
it unwraps the angle stream and verifies monotonicity (increasing or decreasing,
per user selection), within a tolerance.
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List, Tuple

import math


def load_csv(path: Path) -> List[Tuple[float, float]]:
    samples: List[Tuple[float, float]] = []
    with path.open() as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row["time"])
                ang = float(row["angle"])
                samples.append((t, ang))
            except Exception:
                continue
    return samples


def unwrap_angles(samples: List[Tuple[float, float]], decreasing: bool, wrap: float = 2 * math.pi, tol: float = 1e-6) -> List[Tuple[float, float]]:
    """Return a list of (t, unwrapped_angle)."""
    if not samples:
        return []
    unwrapped: List[Tuple[float, float]] = []
    prev_t, prev_ang = samples[0]
    offset = 0.0
    unwrapped.append((prev_t, prev_ang))
    for t, ang in samples[1:]:
        delta = ang - prev_ang
        if decreasing:
            if delta > wrap / 2:  # likely wrapped upward
                offset -= wrap
            elif delta < -wrap / 2:  # wrapped downward (unlikely in decreasing dir)
                offset += wrap
        else:
            if delta < -wrap / 2:  # likely wrapped downward
                offset += wrap
            elif delta > wrap / 2:  # wrapped upward (unlikely in increasing dir)
                offset -= wrap
        unwrapped_ang = ang + offset
        unwrapped.append((t, unwrapped_ang))
        prev_ang = ang
    return unwrapped


def check_monotonic(samples: List[Tuple[float, float]], decreasing: bool, tol: float = 1e-6) -> List[Tuple[int, float, float, float]]:
    """
    Returns a list of violations:
      (index, t_prev, angle_prev, angle_curr)
    where angle decreased by more than tol.
    """
    violations: List[Tuple[int, float, float, float]] = []
    if len(samples) < 2:
        return violations
    prev_t, prev_ang = samples[0]
    for idx, (t, ang) in enumerate(samples[1:], start=1):
        if decreasing:
            if ang > prev_ang + tol:
                violations.append((idx, prev_t, prev_ang, ang))
        else:
            if ang + tol < prev_ang:
                violations.append((idx, prev_t, prev_ang, ang))
        prev_t, prev_ang = t, ang
    return violations


def check_jumps(
    samples: List[Tuple[float, float]],
    max_jump_rad: float,
    max_inst_vel: float | None = None,
    min_dt: float = 1e-6,
) -> List[Tuple[int, float, float, float, float, float, float]]:
    """
    Returns a list of jump violations:
      (index, dt, delta, inst_vel, t_curr, avg_before, avg_after)
    where:
      - abs(delta) > max_jump_rad, or
      - abs(inst_vel) > max_inst_vel (if provided)
    """
    violations: List[Tuple[int, float, float, float, float, float, float]] = []
    if len(samples) < 2:
        return violations
    prev_t, prev_ang = samples[0]
    for idx, (t, ang) in enumerate(samples[1:], start=1):
        dt = max(min_dt, t - prev_t)
        delta = ang - prev_ang
        inst_vel = delta / dt
        jump = abs(delta) > max_jump_rad
        vel_violation = max_inst_vel is not None and abs(inst_vel) > max_inst_vel
        if jump or vel_violation:
            # compute average delta for 5 samples before/after
            before = samples[max(0, idx - 5) : idx]
            after = samples[idx : idx + 5]
            def avg_delta(span):
                if len(span) < 2:
                    return 0.0
                s = 0.0
                c = 0
                pt, pa = span[0]
                for tt, aa in span[1:]:
                    s += (aa - pa)
                    c += 1
                    pt, pa = tt, aa
                return s / c if c else 0.0
            avg_before = avg_delta(before)
            avg_after = avg_delta(after)
            violations.append((idx, dt, delta, inst_vel, t, avg_before, avg_after))
        prev_t, prev_ang = t, ang
    return violations


def main():
    parser = argparse.ArgumentParser(description="Verify monotonicity of angle over time in a telemetry CSV.")
    parser.add_argument("csv", type=Path, help="CSV file with columns: time,target,velocity,angle")
    parser.add_argument("--tol", type=float, default=1e-6, help="Allowed deviation tolerance (default 1e-6)")
    parser.add_argument("--direction", choices=["increasing", "decreasing"], default="increasing", help="Expected monotonic direction")
    parser.add_argument("--max-jump-rad", type=float, default=0.05, help="Max allowed step between samples (rad). Default 0.05 (~2.8 deg)")
    parser.add_argument("--max-inst-vel", type=float, default=None, help="Max allowed instantaneous velocity (rad/s). Optional.")
    args = parser.parse_args()

    raw_samples = load_csv(args.csv)
    if not raw_samples:
        print("No samples loaded.")
        return
    decreasing = args.direction == "decreasing"
    samples = unwrap_angles(raw_samples, decreasing=decreasing, tol=args.tol)
    if not samples:
        print("No samples loaded.")
        return
    violations = check_monotonic(samples, decreasing=decreasing, tol=args.tol)
    jumps = check_jumps(samples, max_jump_rad=args.max_jump_rad, max_inst_vel=args.max_inst_vel)

    if not violations:
        print(f"OK: angle monotonic ({args.direction}) over {len(samples)} samples.")
    else:
        print(f"Found {len(violations)} monotonicity violations (tol={args.tol}, {args.direction}):")
        for idx, t_prev, ang_prev, ang in violations[:20]:
            print(f"  idx={idx} t_prev={t_prev:.6f} angle_prev={ang_prev:.6f} angle={ang:.6f}")
        if len(violations) > 20:
            print(f"... {len(violations) - 20} more")

    if not jumps:
        print(f"OK: no jump violations (max_jump_rad={args.max_jump_rad}, max_inst_vel={args.max_inst_vel}).")
    else:
        print(f"Found {len(jumps)} jump violations (max_jump_rad={args.max_jump_rad}, max_inst_vel={args.max_inst_vel}):")
        for idx, dt, delta, inst_vel, t, avg_before, avg_after in jumps[:20]:
            print(
                f"  idx={idx} t={t:.6f} dt={dt:.6e} delta={delta:.6f} inst_vel={inst_vel:.3f} "
                f"avg_before={avg_before:.6f} avg_after={avg_after:.6f}"
            )
        if len(jumps) > 20:
            print(f"... {len(jumps) - 20} more")


if __name__ == "__main__":
    main()
