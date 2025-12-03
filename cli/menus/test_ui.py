from __future__ import annotations

import curses
import time
from typing import TYPE_CHECKING

from pysfoc import CONTROL_MODE_IDS  # type: ignore[import-not-found]
from pysfoc.constants import CONTROL_MODE_SEQUENCE, REG_CONTROL_MODE, REG_STATUS, REGISTER_IDS  # type: ignore[import-not-found]
from pysfoc.packet_commander import PacketCommanderClient  # type: ignore[import-not-found]
from pysfoc.api import MotorState  # type: ignore[import-not-found]

from pysfoc import OPENLOOP_MODES  # type: ignore[import-not-found]
from menus.utils import draw_ui, do_full_rotation, do_run, do_step, adjust_target, stop_run


def run_test_ui(client: PacketCommanderClient, state: MotorState) -> None:
    def refresh_status_regs():
        for reg, attr in (
            (REG_CONTROL_MODE, "control_mode_val"),
            (REGISTER_IDS["torque_mode"], "torque_mode_val"),
            (REG_STATUS, "status_val"),
        ):
            val = client.read_reg(reg)
            if val is not None:
                setattr(state, attr, int(val))
                if reg == REG_CONTROL_MODE:
                    state.control_mode = int(val)
                    state.open_loop = state.control_mode in OPENLOOP_MODES

    def loop(stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        refresh_status_regs()
        draw_ui(stdscr, state)
        while True:
            telem = client.poll_telemetry()
            if telem:
                state.telemetry = telem
            ch = stdscr.getch()
            if ch == -1:
                draw_ui(stdscr, state)
                time.sleep(0.01)
                continue
            if ch in (ord("q"), ord("Q")):
                stop_run(client, state)
                break  # return to main menu
            if ch == ord("f"):
                state.set_enable(True)
                state.running = True
                state.running_dir = 1
                draw_ui(stdscr, state)
                do_full_rotation(client, state, direction=1)
                state.running = False
                draw_ui(stdscr, state)
                state.set_enable(False)
            elif ch == ord("F"):
                state.set_enable(True)
                state.running = True
                state.running_dir = -1
                draw_ui(stdscr, state)
                do_full_rotation(client, state, direction=-1)
                state.running = False
                draw_ui(stdscr, state)
                state.set_enable(False)
            elif ch == ord("s"):
                state.set_enable(True)
                state.running = True
                state.running_dir = 1
                draw_ui(stdscr, state)
                do_step(client, state, direction=1)
                state.running = False
                draw_ui(stdscr, state)
                state.set_enable(False)
            elif ch == ord("S"):
                state.set_enable(True)
                state.running = True
                state.running_dir = -1
                draw_ui(stdscr, state)
                do_step(client, state, direction=-1)
                state.running = False
                draw_ui(stdscr, state)
                state.set_enable(False)
            elif ch == ord("r"):
                state.set_enable(True)
                do_run(client, state, direction=1)
            elif ch == ord("R"):
                state.set_enable(True)
                do_run(client, state, direction=-1)
            elif ch == curses.KEY_UP:
                adjust_target(client, state, 0.2)
            elif ch == curses.KEY_DOWN:
                adjust_target(client, state, -0.2)
            elif ch == ord(" "):
                stop_run(client, state)
            elif ch == ord("m"):
                current = state.control_mode if state.control_mode is not None else CONTROL_MODE_SEQUENCE[0]
                try:
                    idx = CONTROL_MODE_SEQUENCE.index(current)
                    next_mode = CONTROL_MODE_SEQUENCE[(idx + 1) % len(CONTROL_MODE_SEQUENCE)]
                except ValueError:
                    next_mode = CONTROL_MODE_SEQUENCE[0]
                state.set_control_mode(next_mode)
                state.refresh_status()
                draw_ui(stdscr, state)
            draw_ui(stdscr, state)

    curses.wrapper(loop)
