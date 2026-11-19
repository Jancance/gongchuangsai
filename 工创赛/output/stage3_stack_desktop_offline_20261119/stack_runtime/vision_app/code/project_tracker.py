"""Conservative image-space ID demo: at most one visible item per color.

Short loss hides the target but retains an internal provisional ID. This ID
is not proof of physical identity and is never an actionable pose.
"""
from collections import defaultdict
import math
import uuid

COLORS={'red','yellow','green','blue','lightblue','black'}

def red_transition(previous_result,current_result):
    old={t['id'] for t in previous_result.get('observed',[]) if t['color']=='red'}
    new={t['id'] for t in current_result.get('observed',[]) if t['color']=='red'}
    if old and not new:return 'first-red-loss'
    if old and new and old!=new:return 'first-red-id-change'
    if new and new.intersection(previous_result.get('held_ids',[])):
        return 'first-red-provisional-reacquisition'
    return None

class TabletopTracker:
    def __init__(self,max_step_px=100,max_age_ms=150,identity_hold_ms=500,confirm_frames=3):
        if identity_hold_ms<=max_age_ms:raise ValueError('Hold must exceed visible freshness limit')
        self.max_step_px=max_step_px;self.max_age_ms=max_age_ms
        self.identity_hold_ms=identity_hold_ms
        self.confirm_frames=confirm_frames;self.session_id=uuid.uuid4().hex
        self.tracks={};self.next_id=1;self.last_stamp=None

    def clear(self):
        lost=[t['id'] for t in self.tracks.values()]
        self.tracks.clear()
        return lost

    def expire(self,now):
        lost=[]
        for color,t in list(self.tracks.items()):
            if now<t['read_time'] or (now-t['read_time'])*1000>self.identity_hold_ms:
                lost.append(t['id']);del self.tracks[color]
        return lost

    def update(self,regions,read_time):
        if self.last_stamp is not None and read_time<=self.last_stamp:
            return {'observed':[],'retired_ids':self.clear(),'reason':'NONINCREASING_TIMESTAMP'}
        self.last_stamp=read_time
        lost=self.expire(read_time)
        groups=defaultdict(list)
        for r in regions:
            if r.get('label') in COLORS and 'silhouette_center_px' in r:
                groups[r['label']].append(r)
        active={}
        ambiguous=[]
        for color,items in groups.items():
            if len(items)!=1:
                ambiguous.append(color)
                old=self.tracks.pop(color,None)
                if old is not None:lost.append(old['id'])
                continue
            r=items[0];center=r['silhouette_center_px'];area=r['area_px2']
            old=self.tracks.get(color)
            same=old is not None and math.dist(center,old['center_px'])<=self.max_step_px
            source=r.get('proposal_source','combined_silhouette')
            reference_changed=old is not None and source!=old.get('proposal_source','combined_silhouette')
            if same:
                if reference_changed:
                    # 整体轮廓与顶面面积不可直接比较，用可见直径比检查尺度。
                    diameter=min(r['bbox_px'][2:]);old_diameter=min(old['bbox_px'][2:])
                    same=.5<=diameter/max(old_diameter,1)<=2
                else:
                    same=.5<=area/max(old['area_px2'],1)<=2
            if same:
                identity=old['id']
                reacquired=not old['visible_in_last_frame'] or (read_time-old['read_time'])*1000>self.max_age_ms
                hits=1 if reacquired else old['consecutive_observations']+1
                identity_unverified=old['identity_continuity_unverified'] or reacquired
            else:
                identity=f'T{self.next_id:03d}';self.next_id+=1;hits=1
                reacquired=False;identity_unverified=False
                if old is not None:lost.append(old['id'])
            active[color]={'id':identity,'color':color,'center_px':list(center),
                           'bbox_px':r['bbox_px'],'area_px2':area,'read_time':read_time,
                           'proposal_source':r.get('proposal_source','combined_silhouette'),
                           'radius_px':r.get('radius_px'),
                           'geometry_reference_changed':bool(same and reference_changed),
                           'consecutive_observations':hits,'visible_in_last_frame':True,
                           'identity_continuity_unverified':identity_unverified,
                           'state':'REACQUIRED_UNVERIFIED' if reacquired else
                                   'OBSERVED' if hits>=self.confirm_frames else 'CANDIDATE'}
        held=[]
        for color,old in self.tracks.items():
            if color in active or color in ambiguous:continue
            old['visible_in_last_frame']=False
            active[color]=old
            held.append(old['id'])
        self.tracks=active
        return {'observed':[t for t in active.values() if t['visible_in_last_frame']],
                'retired_ids':lost,
                'held_ids':held,'ambiguous_colors':ambiguous,
                'reason':'DUPLICATE_COLOR_REJECTED' if ambiguous else
                         'NO_CURRENT_DETECTION' if held and len(active)==len(held) else 'UPDATED'}

    def visible(self,now):
        self.expire(now)
        return [t for t in self.tracks.values() if t['visible_in_last_frame']
                and (now-t['read_time'])*1000<=self.max_age_ms]
