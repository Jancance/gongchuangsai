"""Each ready visit uses current target; static classification anchors later120."""
try:
    from .initial_observation_branch import InitialObservationRotation
except ImportError:
    from initial_observation_branch import InitialObservationRotation


class EachObservationRotation(InitialObservationRotation):
    def update(self,detection,observed_s,target,remaining):
        before=self.branch
        ready=super().update(detection,observed_s,target,remaining)
        if before is None and self.branch=='initial_stopped':
            # Earlier classification samples never count as later rotation.
            self.rotation.pause()
            self.rotation.progress=0.0
            self.rotation.opened=False
            self.rotation.update(detection,observed_s,target)
            return False
        return ready
