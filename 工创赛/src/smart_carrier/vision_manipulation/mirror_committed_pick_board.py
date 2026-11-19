"""Explicit mirror candidate adapter; construction performs original Flash verification."""
import hashlib
import json
from pathlib import Path
from .committed_pick_board import CommittedPickBoard


def flash_segments_digest(segments):
    digest = hashlib.sha256()
    for address, data in segments:
        digest.update(int(address).to_bytes(4, 'little'))
        digest.update(len(data).to_bytes(4, 'little'))
        digest.update(bytes(data))
    return digest.hexdigest()


class MirrorCommittedPickBoard(CommittedPickBoard):
    def __init__(self, target, symbols, flash_segments, *, candidate_manifest):
        flash_segments = tuple((address, bytes(data)) for address, data in flash_segments)
        candidate = json.loads(Path(candidate_manifest).read_text(encoding='utf-8'))
        if (candidate.get('extend_sign') != 1 or candidate.get('home_direction') != 1 or
                flash_segments_digest(flash_segments) !=
                candidate.get('images', {}).get('Vehicle_LiftRuler', {}).get('flash_segments_sha256')):
            raise ValueError('镜像适配器需要匹配的镜像候选Flash清单')
        super().__init__(target, symbols, flash_segments)

    @staticmethod
    def pose_ready(value):
        return CommittedPickBoard.pose_ready(value, extension_sign=1)
