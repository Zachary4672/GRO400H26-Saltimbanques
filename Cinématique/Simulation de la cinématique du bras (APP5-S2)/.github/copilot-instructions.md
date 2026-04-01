<!-- .github/copilot-instructions.md -->
# Guidance for AI coding agents (concise)

Purpose
- Help contributors and automated agents reason about this repository's structure, runtime assumptions, and common pitfalls.

Big picture
- Core modules live in `fonction/`.
- `fonction/donnees.py` holds global configuration (robot link lengths `L1,L2,L3`, base pose `wx,wy,wz`, `sens_outil`, `config_coude`).
- `fonction/bras_robot.py` implements kinematics and exposes `Calculate(bState, fA1, fA2, fA3)` which sets globals used for display and serial output (e.g. `angles`, `p_ee_w`, `vitesse`).
- `fonction/main.py` is the runtime loop: it opens a serial port, repeatedly calls `bras_robot.Calculate` in IK (`bState=True`) or linear (`bState=False`) modes, and sends angle commands to an Arduino.
- `fonction/pick_and_place.py` contains trajectory helpers but is incomplete/buggy; treat it as work-in-progress.

Important patterns and conventions
- Globals are used extensively: `bras_robot.Calculate` mutates module-level vars (`angles`, `p_ee_w`, `p_e1_w`, `p_e2_w`, `x_tool_w`, `p_cam_w`). Read these after calling `Calculate`.
- `donnees.Donnees` is a class used as a namespace for parameters. Agents should update fields there for test scenarios (e.g. set `Donnees.x_cible`, `y_cible`, `z_cible`).
- `sens_outil` is -1 or +1 and flips wrist/tool direction; `config_coude` is "bas" or "haut" for elbow-down/up solutions.
- `bras_robot` reloads `donnees` via `importlib.reload(donnees)` to pick up runtime edits; prefer modifying `donnees.Donnees` attributes rather than re-importing modules.

Runtime / developer workflows
- To run the real loop, edit `fonction/main.py` and set `PORT` to your serial device (Linux example: `/dev/ttyUSB0` or `/dev/ttyACM0`) then run `python3 fonction/main.py` from the repository root.
- There is no test harness. For quick debugging run a small script that sets `donnees.Donnees` target values and calls `bras_robot.Calculate(True,0,0,0)` to inspect `bras_robot.angles` and position globals.

Integration points & gotchas
- Serial protocol: `main.py` expects ASCII messages from the Arduino; it parses lines where the first token can be `Done` or numeric tokens used for linear motion. Carefully mirror this format when simulating the device.
- Known issues to be aware of when editing:
  - `fonction/donnees.py` currently contains a malformed assignment near `x_home` and an extraneous `1` at file end — this will cause a SyntaxError. Fix before running.
  - `fonction/pick_and_place.py` contains undefined names (`license`, `moment` usage) and incomplete logic.
  - `bras_robot.Calculate` signature is `(bState, fA1, fA2, fA3)` but some call-sites pass different args; prefer calling with explicit all arguments.

How to contribute changes safely
- Run small unit-style checks by calling `bras_robot.Calculate` in a REPL or short script; inspect `bras_robot.angles` and `p_ee_w` for correctness.
- When changing kinematics, include a short example in the commit message showing input target and expected `angles` tuple to help future agents reproduce results.

Files to inspect for examples
- IK and camera logic: `fonction/bras_robot.py`
- Runtime loop and serial protocol: `fonction/main.py`
- Static parameters: `fonction/donnees.py`

If anything is unclear, ask for concrete run commands (serial device path, Arduino message examples) or permission to fix the small syntax bugs so tests can run.
