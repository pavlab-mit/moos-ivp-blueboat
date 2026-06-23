#!/usr/bin/env python3
"""
   uBBPIDTuner — pBBPID live tuner + scope (pymoos2 AppCastingMOOSApp).
   Author: Karan Mahesh

   Entry point: pAntler runs this as `uBBPIDTuner <mission>.moos [alias]`.
   The actual app lives in BBPIDTuner.py (installed alongside in bin/).
"""

import os
import sys


# ---- interpreter bootstrap -----------------------------------------------
# pAntler launches us with whatever `python3` is on PATH; if that interpreter
# lacks pymoos2/matplotlib, re-exec once into one that has them (BBPID_PYTHON
# or ~/base/bin/python3) so the same `Run = uBBPIDTuner` line just works.
def _ensure_interp():
    if os.environ.get("BBPID_TUNER_REEXEC"):
        return
    try:
        import pymoos2          # noqa: F401
        import matplotlib       # noqa: F401
        return
    except ImportError:
        pass
    for cand in (os.environ.get("BBPID_PYTHON"),
                 os.path.expanduser("~/base/bin/python3")):
        if (cand and os.path.exists(cand)
                and os.path.realpath(cand) != os.path.realpath(sys.executable)):
            os.execve(cand, [cand] + sys.argv,
                      dict(os.environ, BBPID_TUNER_REEXEC="1"))
    sys.exit("ERROR: uBBPIDTuner needs a python with pymoos2+matplotlib; "
             "set BBPID_PYTHON to one (e.g. ~/base/bin/python3).")


_ensure_interp()

import webbrowser
from pymoos2 import run
from BBPIDTuner import BBPIDTuner

PROC_NAME = "uBBPIDTuner"


def show_help():
    print(
        f"{'=' * 60}\n"
        f"Usage: {PROC_NAME} file.moos [OPTIONS]\n"
        f"{'=' * 60}\n\n"
        f"SYNOPSIS:\n  {BBPIDTuner.SYNOPSIS}\n\n"
        f"Options:\n"
        f"  --alias=<ProcessName>  Launch with the given process name.\n"
        f"  --example, -e          Display example MOOS configuration block.\n"
        f"  --help, -h             Display this help message.\n"
        f"  --interface, -i        Display MOOS publications and subscriptions.\n"
        f"  --version, -v          Display release version.\n"
    )


def show_example():
    print(BBPIDTuner.EXAMPLE_CONFIG)


def show_interface():
    subs = "\n".join(f"  {s}" for s in BBPIDTuner.SUBSCRIPTIONS)
    pubs = "\n".join(f"  {p}" for p in BBPIDTuner.PUBLICATIONS)
    print(f"{'=' * 60}\nInterface for: {PROC_NAME}\n{'=' * 60}\n\n"
          f"SUBSCRIPTIONS:\n{subs}\n\nPUBLICATIONS:\n{pubs}")


def show_version():
    print(f"{PROC_NAME} v{BBPIDTuner.VERSION} (pymoos2)")


def main():
    if len(sys.argv) < 2:
        show_help()
        return
    arg = sys.argv[1]
    if arg in ("--help", "-h"):
        show_help(); return
    elif arg in ("--example", "-e"):
        show_example(); return
    elif arg in ("--interface", "-i"):
        show_interface(); return
    elif arg in ("--version", "-v"):
        show_version(); return
    elif arg in ("--web", "-w"):
        webbrowser.open("https://pypi.org/project/pymoos2/"); return

    mission_file = sys.argv[1]
    app_name = sys.argv[2] if len(sys.argv) > 2 else PROC_NAME
    run(BBPIDTuner, app_name, mission_file)


if __name__ == "__main__":
    main()
