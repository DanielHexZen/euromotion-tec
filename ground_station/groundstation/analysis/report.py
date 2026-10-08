"""AUTO run report (R10, G5): which frames did the PC send while the vehicle was in AUTO?"""

from pathlib import Path

from ..record.recorder import RX, TX
from .loader import iter_messages, open_session

# Anything else sent during AUTO counts as navigation-relevant.
ALLOWED = {"ESTOP", "HEARTBEAT"}
ALLOWED_SET_MODE = {"IDLE"}


def build_report(path: Path) -> tuple[str, bool]:
    """Returns (markdown text, clean) where clean means no navigation-relevant command."""
    meta, codec = open_session(path)
    enums = codec.schema.enums["Mode"]
    names = {v: k for k, v in enums.items()}
    mode = None                     # last mode the VEHICLE reported
    auto_s, total_tx = 0, 0
    sent, flagged = [], []
    auto_start = None
    for t, direction, msg in iter_messages(path, codec):
        if direction == RX and msg.name == "TELEM_FAST":
            mode = msg.fields["mode"]
            if mode == enums["AUTO"] and auto_start is None:
                auto_start = t
            elif mode != enums["AUTO"] and auto_start is not None:
                auto_s += (t - auto_start) / 1e9
                auto_start = None
        elif direction == TX:
            total_tx += 1
            if mode == enums["AUTO"]:
                label = msg.name + (f" {names.get(msg.fields['mode'], msg.fields['mode'])}"
                                    if msg.name == "SET_MODE" else "")
                ok = msg.name in ALLOWED or (
                    msg.name == "SET_MODE" and names.get(msg.fields["mode"]) in ALLOWED_SET_MODE)
                sent.append((t, label, ok))
                if not ok:
                    flagged.append((t, label))
    if auto_start is not None:
        auto_s += (t - auto_start) / 1e9    # session ended while still in AUTO

    out = [f"# AUTO run report: {Path(path).name}", "",
           f"- firmware {meta.get('fw_hash', '?')}, protocol {meta.get('vehicle_proto_hash', '?')}",
           f"- AUTO phase (as reported by the vehicle): {auto_s:.1f} s",
           f"- frames sent in the session: {total_tx}, while the vehicle reported AUTO: {len(sent)}", ""]
    if not sent and auto_s == 0:
        out.append("**No AUTO phase found in this log.**")
        return "\n".join(out) + "\n", True
    if flagged:
        out.append(f"**{len(flagged)} navigation-relevant command(s) sent during AUTO:**")
        out += [f"- {t} ns: {label}" for t, label in flagged]
    else:
        out.append("**No navigation-relevant command sent.**")
    if sent:
        out += ["", "All frames sent during AUTO:"] + [
            f"- {t} ns: {label}{'' if ok else '  <-- FLAGGED'}" for t, label, ok in sent]
    return "\n".join(out) + "\n", not flagged
