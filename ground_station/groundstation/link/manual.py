"""Gamepad-level driving logic (R7): sticks count only while the dead-man is held."""

from .command_manager import CommandManager


class ManualDrive:
    def __init__(self, mgr: CommandManager):
        self._mgr = mgr
        self.held = False

    def deadman(self, held: bool) -> None:
        self.held = held
        if not held:
            self._mgr.clear_periodic("DRIVE")
            self._mgr.clear_periodic("HEARTBEAT")
            self._mgr.send("DRIVE", v_mmps=0, w_mradps=0)   # stop now, do not wait for a slot

    def update(self, v_mmps: int, w_mradps: int) -> None:
        if self.held:
            self._mgr.set_periodic("DRIVE", v_mmps=v_mmps, w_mradps=w_mradps)
            self._mgr.set_periodic("HEARTBEAT")
