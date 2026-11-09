"""原R算法的像素展示门；不修改检测器，不转换毫米，不发送动作。"""
import argparse
from copy import deepcopy
import math
from numbers import Real

import cv2


def parse_paper_labels(text):
    """仅接受用户显式指定的当前摆法；不实施数字OCR。"""
    try:
        labels = tuple(int(s.strip()) for s in text.split(','))
    except (ValueError, AttributeError):
        raise argparse.ArgumentTypeError('人工编号须为1、2、3的排列，例如3,2,1') from None
    if len(labels) != 3 or set(labels) != {1, 2, 3}:
        raise argparse.ArgumentTypeError('人工编号须为1、2、3的排列，例如3,2,1')
    return labels


class RingPixelPresenter:
    """把新鲜候选与稳定窗口坐标分开；每帧独立发布，失效不继承旧坐标。"""
    def __init__(self, minimum_confidence=.9, maximum_age_ms=150., paper_labels=None, ring1_side='left'):
        if not math.isfinite(minimum_confidence) or not 0 < minimum_confidence <= 1:
            raise ValueError('圆环最低置信度无效')
        if not math.isfinite(maximum_age_ms) or maximum_age_ms <= 0:
            raise ValueError('圆环像素年龄门无效')
        if ring1_side not in ('left', 'right'):
            raise ValueError('未知圆环输出顺序')
        if paper_labels is not None and (len(paper_labels) != 3 or set(paper_labels) != {1, 2, 3}):
            raise ValueError('人工标签必须是1、2、3的排列')
        self.minimum_confidence = minimum_confidence
        self.maximum_age_ms = maximum_age_ms
        self.paper_labels = tuple(paper_labels) if paper_labels is not None else None
        self.order = 'LEFT_TO_RIGHT' if ring1_side == 'left' else 'RIGHT_TO_LEFT'
        self.reset()

    def _empty(self, state, reason):
        return dict(state=state, reason=reason, units='px', pixel_ready=False,
                    candidate_centers_px=[], stable_centers_px=[], detector_stable=False,
                    confidence=None, host_read_time=None, result_host_age_ms=None,
                    order=self.order, paper_labels=list(self.paper_labels) if self.paper_labels else None,
                    label_source='MANUAL_FIXED_LAYOUT_NOT_OCR' if self.paper_labels else 'UNASSIGNED',
                    metric_valid=False, motion_allowed=False, serial='DISABLED')

    def reset(self):
        self._last_stamp = None
        self._status = self._empty('WAITING', 'NO_FRAME')

    def observe(self, result, frame_shape, read_time, finished):
        status = self._empty('LOST', 'NO_THREE_CENTERS')
        stamp_ok = isinstance(read_time, Real) and not isinstance(read_time, bool) and math.isfinite(read_time)
        increasing = stamp_ok and (self._last_stamp is None or read_time > self._last_stamp)
        if increasing:
            self._last_stamp = float(read_time)
        age = (float(finished) - read_time) * 1000 if stamp_ok else None
        status['host_read_time'] = float(read_time) if stamp_ok else None
        status['result_host_age_ms'] = float(age) if age is not None and math.isfinite(age) else None
        if not increasing or age is None or not math.isfinite(age) or not 0 <= age <= self.maximum_age_ms:
            status.update(state='STALE', reason='READ_TIMESTAMP_OR_RESULT_AGE')
            self._status = status
            return deepcopy(status)
        centers = getattr(result, 'centers_px', ())
        try:
            valid = bool(getattr(result, 'detected', False)) and len(centers) == 3 and all(
                len(p) == 2 and all(math.isfinite(float(v)) for v in p) and
                0 <= float(p[0]) < frame_shape[1] and 0 <= float(p[1]) < frame_shape[0]
                for p in centers)
        except (TypeError, ValueError, OverflowError, IndexError):
            valid = False
        if valid:
            points = [[float(x), float(y)] for x, y in centers]
            confidence = float(getattr(result, 'confidence', 0.))
            detector_stable = bool(getattr(result, 'stable', False))
            quality = math.isfinite(confidence) and confidence >= self.minimum_confidence
            ready = quality and detector_stable
            status.update(state='STABLE' if ready else 'CANDIDATE',
                          reason='WINDOW_STABLE' if ready else 'LOW_CONFIDENCE' if not quality else 'WINDOW_NOT_STABLE',
                          candidate_centers_px=points, stable_centers_px=deepcopy(points) if ready else [],
                          pixel_ready=ready, detector_stable=detector_stable,
                          confidence=confidence if math.isfinite(confidence) else None)
        self._status = status
        return deepcopy(status)

    def current(self, now):
        """绘图/终端打印时再次检查年龄；上次ready不可以无限沿用。"""
        status = deepcopy(self._status)
        if status['host_read_time'] is not None:
            age = (float(now)-status['host_read_time'])*1000
            status['result_host_age_ms'] = age if math.isfinite(age) else None
            if not math.isfinite(age) or not 0 <= age <= self.maximum_age_ms:
                status.update(state='STALE', reason='READ_TIMESTAMP_OR_RESULT_AGE', pixel_ready=False,
                              candidate_centers_px=[], stable_centers_px=[])
        return status


def draw_ring_pixels(frame, status):
    """绿色仅为稳定窗口坐标，黄色为当前候选；失检/过期不画旧十字。"""
    color = (0, 255, 0) if status['pixel_ready'] else (0, 215, 255)
    points = status['stable_centers_px'] if status['pixel_ready'] else status['candidate_centers_px']
    labels = status['paper_labels']
    for i, (x, y) in enumerate(points):
        center = (round(x), round(y))
        cv2.drawMarker(frame, center, color, cv2.MARKER_CROSS, 20, 2)
        label = f'slot{i+1}' + (f' paper{labels[i]}(manual)' if labels else ' paper?')
        label += ' STABLE' if status['pixel_ready'] else ' CANDIDATE'
        width = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .5, 1)[0][0]
        px = max(5, min(center[0]+12, frame.shape[1]-width-5))
        py = max(20, min(center[1], frame.shape[0]-10))
        cv2.putText(frame, label, (px, py), cv2.FONT_HERSHEY_SIMPLEX, .5, color, 1)
    lines = [f"RING PIXELS | {status['state']} | {status['reason']}",
             'Centers=' + str([[round(v, 1) for v in p] for p in status['stable_centers_px']]) + ' px',
             'Slots=' + status['order'] + ' PaperIDs=' + (str(labels)+' MANUAL - NOT OCR' if labels else 'UNASSIGNED'),
             'Window stability only; stationary=UNVERIFIED; metric=UNVERIFIED; serial=DISABLED']
    panel_width = min(frame.shape[1]-20, max(cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, .55, 1)[0][0] for s in lines)+20)
    cv2.rectangle(frame, (10, 64), (10+panel_width, 174), (0, 0, 0), -1)
    for i, line in enumerate(lines):
        cv2.putText(frame, line, (20, 86+i*26), cv2.FONT_HERSHEY_SIMPLEX, .55, color, 1, cv2.LINE_AA)
    return frame


def format_ring_status(status, fps):
    return (f"Mode=RING_STATION FPS={fps:.1f} Units=px State={status['state']} "
            f"Centers={status['stable_centers_px']} CandidateCenters={status['candidate_centers_px']} "
            f"PixelStable={status['pixel_ready']} Reason={status['reason']} "
            f"Order={status['order']} PaperIDs={status['paper_labels']} LabelSource={status['label_source']} "
            f"Stationary=UNVERIFIED Metric=UNVERIFIED Serial=DISABLED")
