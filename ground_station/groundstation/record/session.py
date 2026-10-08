"""One folder per session: raw.bin, session.json, protocol.yaml snapshot."""

import json
from datetime import datetime
from pathlib import Path

from .recorder import Recorder


class Session:
    def __init__(self, root: Path, meta: dict, protocol_text: str):
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.path = Path(root) / stamp
        self.path.mkdir(parents=True, exist_ok=False)
        self.meta = {"start_time": datetime.now().astimezone().isoformat(), **meta}
        # The snapshot is what lets the loader decode this log after the protocol changed.
        (self.path / "protocol.yaml").write_text(protocol_text)
        self._write_meta()
        self.recorder = Recorder(self.path / "raw.bin")

    def update(self, **kv) -> None:
        self.meta.update(kv)
        self._write_meta()

    def _write_meta(self) -> None:
        tmp = self.path / "session.json.tmp"
        tmp.write_text(json.dumps(self.meta, indent=2))
        tmp.replace(self.path / "session.json")

    def close(self, **kv) -> None:
        self.recorder.close()
        self.update(end_time=datetime.now().astimezone().isoformat(), closed_cleanly=True, **kv)
