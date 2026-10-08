# MR2006B ground station (Python layer)

Implements `MR2006B Ground Station — Python Layer Spec`, phase A: R1–R5, R9, R10 and the
logic of R6/R7. UI (R8 live view, global hotkey, gamepad) and P1 are not built yet.

```bash
python3.12 -m venv .venv && .venv/bin/pip install -r requirements.txt
.venv/bin/python -m pytest                                  # 31 tests, ~25 s, no hardware
.venv/bin/python -m groundstation sim                       # prints a pty path
.venv/bin/python -m groundstation run --port <path|/dev/cu.usbmodemXXXX> [--baud 115200]
.venv/bin/python -m groundstation report sessions/<folder>  # exit code 2 if flagged
.venv/bin/python tools/gen_protocol.py                      # protocol.yaml -> generated/link_msgs.[ch]
```

```python
from groundstation.analysis.loader import load_session
frames = load_session("sessions/20261007-190750")   # {"TELEM_FAST": DataFrame, "ACK": ..., ...}
```

`protocol.yaml` is the single definition. Python reads it directly; each session stores a
snapshot of it, so old logs stay decodable after the protocol changes. The firmware gets
`generated/link_msgs.[ch]`. After editing the yaml, rerun the generator and the tests.
