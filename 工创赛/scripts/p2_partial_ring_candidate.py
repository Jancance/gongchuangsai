"""独立离线残缺环候选：以完整环提供形状，以当前可见多环边缘确认第三环。

不修改原检测器；不使用旧帧补坐标；不输出稳定、毫米或可执行目标。
"""
import math
import time
from dataclasses import replace

import cv2
import numpy as np


def _template(anchor, observations, radius):
    ellipse = anchor.ellipse
    theta = math.radians(ellipse.angle_deg)
    rotation = np.array([[math.cos(theta), -math.sin(theta)],
                         [math.sin(theta), math.cos(theta)]])
    # 参考完整环提供当前图像中的椭圆形状，不是相机或毫米标定。
    shape = rotation @ np.diag(np.array(ellipse.axes)/2)
    inverse = np.linalg.inv(shape)
    radii = []
    for obs in observations:
        if np.linalg.norm(obs.center-anchor.center) > radius:
            continue
        value = math.sqrt(obs.major_axis*obs.minor_axis /
                          (ellipse.major_axis*ellipse.minor_axis))
        if .5 <= value <= 1.05:
            radii.append(value)
    bands = []
    for value in sorted(radii):
        if not bands or value-bands[-1][-1] > .025:
            bands.append([value])
        else:
            bands[-1].append(value)
    bands = np.array([np.mean(band) for band in bands])
    angles = np.arange(72)*2*np.pi/72
    unit = np.stack([np.cos(angles), np.sin(angles)], axis=1)
    offsets = (unit[None,:,:]*bands[:,None,None]) @ shape.T
    normals = unit @ inverse
    normals /= np.linalg.norm(normals, axis=1)[:,None]
    return offsets, normals, bands


def partial_candidate(frame, module, config):
    started = time.perf_counter()
    empty = dict(accepted=False, centers_px=[], stable=False, metric_valid=False,
                 motion_allowed=False, serial='DISABLED', units='px', method='partial_arc_template_v1')
    def reject(reason, **data):
        return dict(empty, reason=reason, processing_ms=(time.perf_counter()-started)*1000, **data)
    observations = module._extract_observations(frame, config)
    candidates = module._cluster_observations(observations, config)
    anchors = [c for c in candidates if c.support >= 10 and c.diameter_bands >= 5
               and c.ellipse.minor_axis/c.ellipse.major_axis >= .75]
    if len(anchors) < 2:
        return reject('NEED_TWO_COMPLETE_ANCHORS')
    # 仅支持两个完整相邻环 + 一个缺环，不外推两个被遮环，不实现中环遮挡。
    anchors.sort(key=lambda c: (-c.support, -c.diameter_bands, c.center[0]))
    if len(anchors) > 2:
        if anchors[2].support>=.7*anchors[1].support:
            return reject('MULTIPLE_COMPLETE_ANCHOR_PAIRS')
        anchors=anchors[:2]
    left, right = sorted(anchors, key=lambda c:c.center[0])
    spacing = np.linalg.norm(right.center-left.center)
    diameter = np.mean([c.effective_diameter_px for c in anchors])
    if not .45 <= diameter/spacing <= .9:
        return reject('ANCHOR_SPACING_OR_SIZE')
    gray = cv2.GaussianBlur(cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY),(5,5),1.2)
    edge = cv2.Canny(gray,config.canny_low,config.canny_high)
    distance = cv2.distanceTransform(255-edge,cv2.DIST_L2,cv2.DIST_MASK_5)
    gx,gy = cv2.Sobel(gray,cv2.CV_32F,1,0),cv2.Sobel(gray,cv2.CV_32F,0,1)
    magnitude = np.hypot(gx,gy)
    gx,gy = gx/np.maximum(magnitude,1),gy/np.maximum(magnitude,1)
    height,width = gray.shape
    # 弱聚类仅提供搜索种子，不改原检测器门，也不直接成为输出。
    weak=module._cluster_observations(observations,replace(config,min_cluster_support=3,min_diameter_bands=2))
    proposals = []
    for anchor, predicted in ((right,2*right.center-left.center),(left,2*left.center-right.center)):
        offsets,normals,bands = _template(anchor,observations,config.center_cluster_radius_px)
        if len(bands)<5:
            continue
        if not (0<=predicted[0]<width and 0<=predicted[1]<height):
            continue
        # 平移只生成搜索区域，接受必须来自当前帧多条可见弧。
        prior_radius = min(50.,spacing*.1)
        seeds=[c for c in weak if np.linalg.norm(c.center-predicted)<=prior_radius]
        if not seeds:
            continue
        seed=max(seeds,key=lambda c:(c.diameter_bands,c.support))
        # 局部搜索的中心来自当前帧弱轮廓，不把外推点作为已观测圆心。
        search_origin=seed.center
        radius = 12.
        coordinates = np.arange(-radius,radius+1,3.)
        xx,yy = np.meshgrid(coordinates,coordinates)
        centers = search_origin+np.stack([xx.ravel(),yy.ravel()],axis=1)
        trials=[]
        def score(points,scales,all_angles=False):
            active_offsets=offsets if all_angles else offsets[:,::2,:]
            active_normals=normals if all_angles else normals[::2,:]
            pos = points[:,None,None,:]+active_offsets[None,:,:,:]*scales[:,None,None,None]
            ix,iy = np.rint(pos[...,0]).astype(int),np.rint(pos[...,1]).astype(int)
            valid=(ix>=0)&(ix<width)&(iy>=0)&(iy<height)
            ix,iy=np.clip(ix,0,width-1),np.clip(iy,0,height-1)
            # 对每个可见弧同时要求边缘距离和法线方向；反光/物料边不能仅靠距离得分。
            agreement=np.abs(gx[iy,ix]*active_normals[None,None,:,0]+gy[iy,ix]*active_normals[None,None,:,1])
            hits=(distance[iy,ix]<=2.) & (agreement>=.8) & valid
            band_coverage=hits.mean(axis=2)
            coherent=(hits.sum(axis=1)>=min(4,len(bands))).mean(axis=1)
            # 只用交替角度拟合，另一半角度参与后续质量复核。
            training=hits[:,:,::2] if all_angles else hits
            value=.5*training.mean(axis=(1,2))+.5*(training.sum(axis=1)>=min(4,len(bands))).mean(axis=1)
            return value,band_coverage,coherent,hits,distance[iy,ix]
        for scale in np.arange(.9,1.101,.025):
            values,*_=score(centers,np.full(len(centers),scale))
            idx=np.argsort(values)
            # 每个尺度保留空间分离的局部极值，不能只保留一个峰附近的点。
            chosen=[]
            for i in idx[::-1]:
                if all(np.linalg.norm(centers[i]-centers[j])>8 for j in chosen):
                    chosen.append(i)
                if len(chosen)>=4:
                    break
            idx=chosen
            trials.extend((float(values[i]),centers[i].copy(),float(scale)) for i in idx)
        trials.sort(key=lambda t:t[0],reverse=True)
        if trials[0][0]<.5:
            continue
        # 对多个粗候选细化，避免只优化一个局部极值。
        refined=[]
        seeds=[]
        for trial in trials:
            if all(np.linalg.norm(trial[1]-t[1])>8 for t in seeds):
                seeds.append(trial)
            if len(seeds)>=4:
                break
        for _,center,scale in seeds:
            grid=np.arange(-2,2.01,.5)
            xx,yy,ss=np.meshgrid(grid,grid,np.arange(scale-.01,scale+.0101,.005),indexing='ij')
            points=center+np.stack([xx.ravel(),yy.ravel()],axis=1)
            scales=ss.ravel()
            values,*_=score(points,scales)
            idx=int(np.argmax(values))
            refined.append((float(values[idx]),points[idx],float(scales[idx])))
        best=max(refined,key=lambda t:t[0])
        val,point,scale=best
        values,coverage,coherent,hits,distances=score(point[None,:],np.array([scale]),all_angles=True)
        far=[t for t in trials+refined if np.linalg.norm(t[1]-point)>8]
        runner=max((t[0] for t in far),default=0.)
        # 候选间隔需与远离的中心比较，不把同一极值附近的亚像素采样当多解。
        band_ok=int(np.sum(coverage[0]>=.55))
        usable=hits[0]
        residual=float(np.percentile(distances[0][usable],90)) if usable.any() else None
        proposals.append(dict(center_px=point.tolist(),score=val,runner_score=runner,
                              center_margin=val-runner,scale=scale,band_coverage=coverage[0].tolist(),
                              coherent_angle_fraction=float(coherent[0]),supported_bands=band_ok,
                              heldout_coherent_angle_fraction=float((hits[0,:,1::2].sum(axis=0)>=min(4,len(bands))).mean()),
                              visible_edge_distance_p90_px=residual,
                              search_center_offset_px=(point-search_origin).tolist(),search_radius_px=radius,
                              weak_seed_center_px=search_origin.tolist(),
                              weak_seed_support=seed.support,weak_seed_bands=seed.diameter_bands,
                              anchor_centers_px=[left.center.tolist(),right.center.tolist()]))
    if not proposals:
        return reject('NO_IN_FRAME_HYPOTHESIS')
    proposals.sort(key=lambda p:p['score'],reverse=True)
    best=proposals[0]
    if best['supported_bands']<5 or best['coherent_angle_fraction']<.55 or best['heldout_coherent_angle_fraction']<.55 or best['score']<.6:
        return reject('INSUFFICIENT_VISIBLE_ARCS',proposals=proposals)
    if max(abs(v) for v in best['search_center_offset_px'])>best['search_radius_px']-4:
        return reject('SEARCH_BOUNDARY',proposals=proposals)
    if best['center_margin']<.08 or (len(proposals)>1 and best['score']-proposals[1]['score']<.08):
        return reject('AMBIGUOUS_CENTER',proposals=proposals)
    point=np.array(best['center_px'])
    points=sorted([left.center.tolist(),right.center.tolist(),point.tolist()],key=lambda p:p[0],reverse=config.ring1_side=='right')
    span=np.linalg.norm(np.array(points[2])-points[0])
    a,b=np.array(points[2])-points[0],np.array(points[1])-points[0]
    collinearity=abs(a[0]*b[1]-a[1]*b[0])/span**2
    ratio=np.linalg.norm(np.array(points[1])-points[0])/np.linalg.norm(np.array(points[2])-points[1])
    if collinearity>.025 or not .85<=ratio<=1.15:
        return reject('TRIPLE_GEOMETRY',proposals=proposals)
    return dict(empty,accepted=True,centers_px=points,reason='OBSERVED_PARTIAL_ARCS_CANDIDATE_ONLY',
                proposals=proposals,processing_ms=(time.perf_counter()-started)*1000)


def detect_candidate(frame,module,config):
    """完整环优先沿用原结果；仅原质量门失败时尝试独立残缺分支。"""
    started=time.perf_counter()
    raw=module._raw_detection(frame,config)
    if raw and raw['confidence']>=config.minimum_confidence and raw['residual']<=config.maximum_affine_residual and config.perspective_ratio_range[0]<=raw['perspective']<=config.perspective_ratio_range[1]:
        return dict(accepted=True,centers_px=[list(p) for p in raw['centers']],
                    stable=False,metric_valid=False,motion_allowed=False,serial='DISABLED',units='px',
                    method='original_quality_candidate',reason='ORIGINAL_CANDIDATE_ONLY',
                    confidence=raw['confidence'],processing_ms=(time.perf_counter()-started)*1000)
    result=partial_candidate(frame,module,config)
    result['processing_ms']=(time.perf_counter()-started)*1000
    return result
