#!/usr/bin/env python3
"""
Velocity PID auto-tuner CLI (host-side).

Implements the workflow described in AGENT_PID.md:
- Runs velocity step sequences via PacketCommander (PySFOCClient).
- Computes metrics (RMSE/MAE/overshoot/settling/jitter).
- Two-stage PD->PID grid search.

Usage (hardware):
    python pid_tuner.py --port /dev/ttyACM0 --baud 460800 --output out/

Usage (dry-run simulation):
    python pid_tuner.py --dry-run
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from velocity_pid_tuner import (  # type: ignore[import-not-found]
    SearchConfig,
    SerialInterface,
    SimulatedPlantConfig,
    BayesConfig,
    run_search,
    run_bayes_search,
)
from velocity_pid_tuner.plot_live import LivePlotter
from velocity_pid_tuner.report import print_summary, save_csv, save_json
from velocity_pid_tuner.report import save_traces


def _parse_float_list(raw: str) -> Sequence[float]:
    return [float(tok) for tok in raw.replace(" ", "").split(",") if tok]


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Velocity PID auto-tuner (PacketCommander).")
    p.add_argument("--port", help="Serial port (e.g. /dev/ttyACM0). Required unless --dry-run.", default=None)
    p.add_argument("--baud", type=int, default=460800, help="Baud rate (default: %(default)s).")
    p.add_argument("--dry-run", action="store_true", help="Run against simulated plant instead of hardware.")
    p.add_argument("--output", type=Path, default=Path("pid_tuner_out"), help="Output directory for results (default: %(default)s).")
    p.add_argument("--include-high", action="store_true", help="Include high-speed set (60 rad/s).")
    p.add_argument("--algo", type=str, choices=["grid", "bayes"], default="bayes", help="Select search algorithm (grid or bayes). Default: %(default)s.")
    p.add_argument("--telemetry-rate", type=float, default=200.0, help="Requested telemetry rate (Hz). Default: %(default)s.")
    p.add_argument("--poll-interval", type=float, default=0.01, help="Host poll interval seconds. Default: %(default)s.")
    p.add_argument("--top-k-stage1", type=int, default=5, help="Top PD candidates to expand with I gains. Default: %(default)s.")

    p.add_argument("--vel-low", type=str, help="Override low-speed list (comma separated).")
    p.add_argument("--vel-mid", type=str, help="Override mid-speed list (comma separated).")
    p.add_argument("--vel-high", type=str, help="Override high-speed list (comma separated).")

    p.add_argument("--p-grid", type=str, help="Comma-separated P grid.")
    p.add_argument("--i-grid", type=str, help="Comma-separated I grid.")
    p.add_argument("--d-grid", type=str, help="Comma-separated D grid.")
    p.add_argument("--p-range", type=str, help="P range for bayes (min,max).")
    p.add_argument("--i-range", type=str, help="I range for bayes (min,max).")
    p.add_argument("--d-range", type=str, help="D range for bayes (min,max).")
    p.add_argument("--bo-init", type=int, default=6, help="Initial random evaluations for bayes. Default: %(default)s.")
    p.add_argument("--bo-iters", type=int, default=12, help="Bayes optimization iterations. Default: %(default)s.")
    p.add_argument("--bo-candidates", type=int, default=200, help="Random candidate samples per BO iteration. Default: %(default)s.")
    p.add_argument("--bo-lengthscale", type=float, default=0.5, help="Kernel lengthscale for BO. Default: %(default)s.")
    p.add_argument("--bo-noise", type=float, default=1e-3, help="Observation noise for BO GP. Default: %(default)s.")
    p.add_argument("--save-traces", action="store_true", help="Save telemetry traces (CSV) for the top candidate(s).")
    p.add_argument("--traces-top", type=int, default=1, help="How many top candidates to save traces for when --save-traces is set. Default: %(default)s.")
    p.add_argument("--live-plot", action="store_true", help="Show live plot of target/measured velocity during tests (matplotlib).")
    p.add_argument("--verbose", action="store_true", help="Print progress for each search stage/candidate.")
    # Dry-run simulation tuning
    p.add_argument("--sim-oscillate", action="store_true", help="In dry-run mode, inject oscillations to test backoff.")
    p.add_argument("--sim-osc-gain", type=float, default=1.0, help="Oscillation gain (relative to target) in dry-run mode. Default: %(default)s.")
    p.add_argument("--sim-osc-freq", type=float, default=5.0, help="Oscillation frequency (Hz) in dry-run mode. Default: %(default)s.")
    p.add_argument("--stall-target-min", type=float, default=1.0, help="Min |target| rad/s to start stall detection. Default: %(default)s.")
    p.add_argument("--stall-measured-max", type=float, default=0.2, help="Max |measured| rad/s considered stalled. Default: %(default)s.")
    p.add_argument("--stall-duration", type=float, default=0.5, help="Seconds below threshold before aborting candidate. Default: %(default)s.")
    p.add_argument("--osc-pp-min", type=float, default=10.0, help="Min peak-to-peak velocity (rad/s) to flag oscillation. Default: %(default)s.")
    p.add_argument("--osc-duration", type=float, default=0.3, help="Seconds of oscillation before aborting candidate. Default: %(default)s.")
    p.add_argument("--osc-window", type=float, default=0.5, help="Window length (s) for oscillation detection. Default: %(default)s.")
    return p


def main():
    args = build_arg_parser().parse_args()

    cfg = SearchConfig()
    cfg.include_high = args.include_high
    cfg.telemetry_rate_hz = args.telemetry_rate
    cfg.poll_interval = args.poll_interval
    cfg.top_k_stage1 = args.top_k_stage1

    if args.vel_low:
        cfg.vel_low = tuple(_parse_float_list(args.vel_low))
    if args.vel_mid:
        cfg.vel_mid = tuple(_parse_float_list(args.vel_mid))
    if args.vel_high:
        cfg.vel_high = tuple(_parse_float_list(args.vel_high))
    if args.p_grid:
        cfg.p_grid = tuple(_parse_float_list(args.p_grid))
    if args.i_grid:
        cfg.i_grid = tuple(_parse_float_list(args.i_grid))
    if args.d_grid:
        cfg.d_grid = tuple(_parse_float_list(args.d_grid))

    iface = SerialInterface(
        port=args.port,
        baud=args.baud,
        telemetry_rate_hz=cfg.telemetry_rate_hz,
        dry_run=args.dry_run,
        sim_config=SimulatedPlantConfig(
            oscillate=args.sim_oscillate,
            osc_gain=args.sim_osc_gain,
            osc_freq=args.sim_osc_freq,
        ),
    )
    plotter = LivePlotter() if args.live_plot else None
    from velocity_pid_tuner.test_runner import SafetyConfig
    safety_cfg = SafetyConfig(
        stall_target_min=args.stall_target_min,
        stall_measured_max=args.stall_measured_max,
        stall_duration=args.stall_duration,
        osc_pp_min=args.osc_pp_min,
        osc_duration=args.osc_duration,
        osc_window=args.osc_window,
    )

    try:
        if args.algo == "bayes":
            bo_cfg = BayesConfig(
                p_range=tuple(_parse_float_list(args.p_range)) if args.p_range else (0.05, 1.5),
                i_range=tuple(_parse_float_list(args.i_range)) if args.i_range else (0.0, 10.0),
                d_range=tuple(_parse_float_list(args.d_range)) if args.d_range else (0.0, 0.05),
                init_points=args.bo_init,
                iterations=args.bo_iters,
                candidates_per_iter=args.bo_candidates,
                lengthscale=args.bo_lengthscale,
                noise=args.bo_noise,
                telemetry_rate_hz=args.telemetry_rate,
                poll_interval=args.poll_interval,
                include_high=args.include_high,
                vel_low=cfg.vel_low,
                vel_mid=cfg.vel_mid,
                vel_high=cfg.vel_high,
                timings=cfg.timings,
                weights=cfg.weights,
            )
            search_result = run_bayes_search(iface, bo_cfg, verbose=args.verbose, plotter=plotter, safety=safety_cfg)
        else:
            search_result = run_search(iface, cfg, verbose=args.verbose, plotter=plotter, safety=safety_cfg)
        print_summary(search_result, top_n=10)
        save_json(search_result, args.output)
        save_csv(search_result, args.output)
        if args.save_traces:
            save_traces(search_result, args.output, top_n=args.traces_top)
        print(f"\nSaved results to {args.output}")
    finally:
        iface.close()


if __name__ == "__main__":
    main()
