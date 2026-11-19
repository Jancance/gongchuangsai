"""Offline rigid triangle template, in a verified common work-plane frame.

Inputs are OBSERVED base-plane centres with persistent geometric slot IDs.
The caller must project contact-plane evidence correctly; top/side centroids
are not accepted as equivalent. Predictions are search priors, not presence,
colour, freshness, motion authority, grasp permission or placement evidence.
"""
from dataclasses import dataclass
import math
import numpy as np

@dataclass(frozen=True)
class TrianglePose:
    rotation: tuple
    translation_mm: tuple
    residual_rms_mm: float
    centers_mm: tuple
    observed_slots: tuple
    predicted_slots: tuple

def fit_observed_slots(template_centers_mm, observed_base_centers_mm, *,
                       residual_limit_mm=3.0, pair_error_limit_mm=4.0):
    """Fit translation + proper rotation; NEVER scale to hide a wrong match.

    At least two distinct, labelled observed centres are necessary. Labels
    refer to original geometric slots, NOT colour, task order or renumbered
    remaining materials. Unknown labels/phase require a separate association
    provider. Equilateral120deg ambiguity cannot be solved by shape alone.
    """
    template=np.asarray(template_centers_mm,dtype=float)
    if template.shape!=(3,2) or not np.isfinite(template).all():
        raise ValueError('three finite template centres required')
    if not math.isfinite(residual_limit_mm) or not 0<residual_limit_mm<=10:
        raise ValueError('invalid residual bound')
    if not math.isfinite(pair_error_limit_mm) or not 0<pair_error_limit_mm<=10:
        raise ValueError('invalid pair-length bound')
    if not isinstance(observed_base_centers_mm,dict) or len(observed_base_centers_mm)<2:
        raise ValueError('two current labelled base-plane observations required')
    slots=tuple(sorted(observed_base_centers_mm))
    if any(type(k) is not int or not 0<=k<3 for k in slots):
        raise ValueError('persistent geometric slots must be0/1/2')
    observed=np.asarray([observed_base_centers_mm[k] for k in slots],dtype=float)
    if observed.shape!=(len(slots),2) or not np.isfinite(observed).all():
        raise ValueError('finite base-plane centres required')
    source=template[list(slots)]
    for i in range(len(slots)):
        for j in range(i+1,len(slots)):
            expected=float(np.linalg.norm(source[j]-source[i]))
            actual=float(np.linalg.norm(observed[j]-observed[i]))
            if expected<20 or actual<20 or abs(actual-expected)>pair_error_limit_mm:
                raise ValueError('inconsistent observed slot spacing')
    sc=source.mean(axis=0);oc=observed.mean(axis=0)
    u,_,vt=np.linalg.svd((source-sc).T@(observed-oc))
    correction=np.eye(2);correction[1,1]=np.linalg.det(u@vt)
    row_rotation=u@correction@vt
    translation=oc-sc@row_rotation
    projected=template@row_rotation+translation
    rms=float(np.sqrt(np.mean(np.sum((projected[list(slots)]-observed)**2,axis=1))))
    if rms>residual_limit_mm:raise ValueError('triangle fit residual too large')
    return TrianglePose(tuple(map(tuple,row_rotation)),tuple(translation),rms,
                        tuple(map(tuple,projected)),slots,tuple(k for k in range(3) if k not in slots))

def alignment_in_source_frame(pose, *, near_slot=2, retracted_radius_mm=161.0):
    """Return geometric yaw/pivot TARGET, never wheel commands.

    Source-frame +X is the outward arm axis at logical180deg, +Y is its
    transverse axis. Actual chassis frame/sign/rotation centre must be bound
    separately. Reobserve AFTER yaw before computing translation execution.
    """
    if not isinstance(pose,TrianglePose) or type(near_slot) is not int or near_slot not in (0,1,2):
        raise ValueError('pose and persistent near geometric slot required')
    if not math.isfinite(retracted_radius_mm) or not 0<retracted_radius_mm<500:
        raise ValueError('invalid retained pivot-to-TCP radius')
    centers=np.asarray(pose.centers_mm);near=centers[near_slot]
    far=centers[[i for i in range(3) if i!=near_slot]].mean(axis=0)
    direction=far-near;norm=float(np.linalg.norm(direction))
    if norm<20:raise ValueError('degenerate median')
    direction/=norm
    return dict(yaw_error_source_deg=math.degrees(math.atan2(direction[1],direction[0])),
                pivot_target_shift_source_mm=tuple(near-retracted_radius_mm*direction),
                near_slot=near_slot,near_center_is_predicted=near_slot in pose.predicted_slots,
                reobserve_after_yaw=True,motion_allowed=False,grasp_allowed=False)
