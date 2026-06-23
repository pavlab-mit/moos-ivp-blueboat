#!/usr/bin/env python3
"""
   pDiffThrustTuner — A MOOS Application
   Author: Raymond Turrisi
"""

import sys
import webbrowser
from pymoos2 import run
from DiffThrustTuner import DiffThrustTuner

PROC_NAME = "pDiffThrustTuner"


def show_help():
    print(
        f"{'=' * 60}\n"
        f"Usage: {PROC_NAME} file.moos [OPTIONS]\n"
        f"{'=' * 60}\n"
        f"\n"
        f"SYNOPSIS:\n"
        f"  {DiffThrustTuner.SYNOPSIS}\n"
        f"\n"
        f"Options:\n"
        f"  --alias=<ProcessName>\n"
        f"      Launch {PROC_NAME} with the given process name.\n"
        f"  --example, -e\n"
        f"      Display example MOOS configuration block.\n"
        f"  --help, -h\n"
        f"      Display this help message.\n"
        f"  --interface, -i\n"
        f"      Display MOOS publications and subscriptions.\n"
        f"  --version, -v\n"
        f"      Display release version.\n"
        f"  --web, -w\n"
        f"      Open pymoos2 on PyPI in browser.\n"
        f"\n"
        f"Note: If argv[2] does not otherwise match a known option,\n"
        f"      then it will be interpreted as a run alias. This is\n"
        f"      to support pAntler launching conventions."
    )


def show_example():
    print(DiffThrustTuner.EXAMPLE_CONFIG)


def show_interface():
    subs = "\n".join(f"  {s}" for s in DiffThrustTuner.SUBSCRIPTIONS)
    pubs = "\n".join(f"  {p}" for p in DiffThrustTuner.PUBLICATIONS)
    print(
        f"{'=' * 60}\n"
        f"Interface for: {PROC_NAME}\n"
        f"{'=' * 60}\n"
        f"\n"
        f"SUBSCRIPTIONS:\n"
        f"{subs}\n"
        f"\n"
        f"PUBLICATIONS:\n"
        f"{pubs}"
    )


def show_version():
    print(f"{PROC_NAME} v{DiffThrustTuner.VERSION} (pymoos2)")


def main():
    if len(sys.argv) < 2:
        show_help()
        return

    arg = sys.argv[1]
    if arg in ("--help", "-h"):
        show_help()
        return
    elif arg in ("--example", "-e"):
        show_example()
        return
    elif arg in ("--interface", "-i"):
        show_interface()
        return
    elif arg in ("--version", "-v"):
        show_version()
        return
    elif arg in ("--web", "-w"):
        webbrowser.open("https://pypi.org/project/pymoos2/")
        return

    mission_file = sys.argv[1]
    app_name = sys.argv[2] if len(sys.argv) > 2 else PROC_NAME

    run(DiffThrustTuner, app_name, mission_file)


if __name__ == "__main__":
    main()
