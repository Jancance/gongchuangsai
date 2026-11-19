"""Host read boundary only: SC2 ready epoch plus sequential-worker fence.

Exposure time and V4L2 backlog are deliberately not inferred from this metadata.
"""
import math


def camera_metadata(camera):
    worker=getattr(camera,'_worker',None)
    sequence=getattr(camera,'_sequence',None)
    completed=getattr(camera,'last_read_completed_time',None)
    pid=getattr(worker,'pid',None)
    if worker is None or not isinstance(pid,int) or pid<=0:
        return None
    return dict(generation=(id(camera),id(worker),pid),sequence=sequence,completed=completed)


class ObservationFrameEpoch:
    def __init__(self):
        self.latest=None
        self.ready=None
        self.epoch=None
        self.revision=0
        self.pending_reboot=False
        self.invalidate()

    def invalidate(self):
        self.ready=None
        self.epoch=None
        self.generation=None
        self.fence_sequence=None
        self.last_sequence=None
        self.last_completed=None
        self.accepted=False
        self.reason='not_ready'
        self.revision+=1

    @staticmethod
    def newer(value,previous):
        return 0 < ((value-previous)&0xffffffff) < 0x80000000

    def receive(self,status,received,link_was_expired=False):
        reboot_idle=all(status.get(k)==0 for k in ('run','pose','state','prep','item'))
        if reboot_idle:
            self.invalidate()
            self.pending_reboot=True
            self.reason='idle_reset_pending_preparation'
            return True
        if self.pending_reboot:
            if not (status.get('run')==1 and status.get('pose')==1 and
                    status.get('state')==1 and status.get('prep')==0 and status.get('item')==0):
                return False
            # Two-stage reset: idle never grants a ready gate; only the new
            # run's preparation signature rebinds ordering, without a timer.
            self.latest=None
            self.pending_reboot=False
        if self.latest is not None:
            old=self.latest
            if status['run']!=old['run']:
                if not self.newer(status['run'],old['run']):return False
            elif status['pose']!=old['pose']:
                if not self.newer(status['pose'],old['pose']):return False
            elif status['item']<old['item'] or status['state']<old['state']:
                # A return to observing needs the newly published pose.
                return False
        self.latest=dict(status)
        key=(status['run'],status['pose'],status['item'])
        if status['state']!=6 or status['item']>=3:
            if self.ready is not None:self.invalidate()
            return True
        if self.ready!=key:
            self.invalidate()
            self.ready=key
            self.epoch=received
            self.reason='awaiting_post_ready_read'
        return True

    def frame(self,metadata,now):
        self.accepted=False
        if self.ready is None:return False
        try:
            generation=metadata['generation'];seq=metadata['sequence'];stamp=metadata['completed']
            if (generation is None or type(seq) is not int or seq<1 or
                    not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp>now):
                raise ValueError('unknown capture metadata')
        except (TypeError,KeyError,ValueError):
            self._restart('unknown_capture_metadata')
            return False
        if self.generation is not None and generation!=self.generation:
            self._restart('capture_worker_changed')
        self.generation=generation
        if stamp<=self.epoch:
            self.reason='read_completed_before_ready'
            return False
        if self.fence_sequence is None:
            self.fence_sequence=seq
            self.last_sequence=seq
            self.last_completed=stamp
            self.reason='first_post_ready_completion_discarded'
            return False
        if seq<=self.last_sequence or stamp<=self.last_completed:
            self.reason='duplicate_or_out_of_order_capture'
            return False
        self.last_sequence=seq;self.last_completed=stamp
        self.accepted=True;self.reason='host_read_after_ready'
        return True

    def _restart(self,reason):
        self.generation=None;self.fence_sequence=None
        self.last_sequence=None;self.last_completed=None
        self.revision+=1;self.reason=reason

    def diagnostic(self):
        return dict(ready=self.ready,ready_received_monotonic_s=self.epoch,
                    generation=self.generation,fence_sequence=self.fence_sequence,
                    accepted_sequence=self.last_sequence,accepted=self.accepted,reason=self.reason,
                    sensor_exposure_epoch_verified=False,
                    prearrival_motion='UNKNOWN',premotion_witness_enabled=False,
                    moving_camera_compensation_verified=False)
