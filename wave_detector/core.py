"""Temporal raised-hand motion per stable person ID; independent of inference/ROS."""
from dataclasses import dataclass
from collections import deque
import math
import numpy as np


@dataclass(frozen=True)
class WaveConfig:
    window_sec:float=2.
    max_gap_sec:float=.4
    min_samples:int=6
    min_amplitude:float=.25
    min_reversals:int=2
    confidence:float=.5
    direction_epsilon:float=.02
    max_tracks:int=128

    def __post_init__(self):
        for key in ['window_sec','max_gap_sec','min_amplitude','direction_epsilon']:
            value=getattr(self,key)
            if not math.isfinite(value) or value<=0:raise ValueError(key+' must be positive')
        if not 0<self.confidence<=1:raise ValueError('confidence must be in (0,1]')
        for key in ['min_samples','min_reversals','max_tracks']:
            if not isinstance(getattr(self,key),int) or isinstance(getattr(self,key),bool) or getattr(self,key)<1:raise ValueError(key+' must be a positive integer')
        if self.min_samples<3:raise ValueError('At least 3 samples are required')


class WaveDetector:
    def __init__(self,config=WaveConfig()):self.config=config;self.tracks={};self.last_time=None

    def reset(self):self.tracks.clear();self.last_time=None

    def update(self,track_id,t,keypoints,bbox,camera_moving=False):
        if not isinstance(track_id,(str,int)) or isinstance(track_id,bool):raise ValueError('Stable track_id is required')
        if not math.isfinite(t):raise ValueError('Timestamp must be finite')
        if camera_moving:self.reset();return None
        if self.last_time is not None and t<self.last_time:self.reset()
        self.last_time=t
        for identity in list(self.tracks):
            if t-self.tracks[identity]['last']>self.config.window_sec:del self.tracks[identity]
        if track_id not in self.tracks and len(self.tracks)>=self.config.max_tracks:
            del self.tracks[min(self.tracks,key=lambda k:self.tracks[k]['last'])]
        kp=np.asarray(keypoints,dtype=float);box=np.asarray(bbox,dtype=float)
        if kp.shape!=(17,3) or box.shape!=(4,) or not np.isfinite(box).all() or box[2]<=box[0] or box[3]<=box[1]:raise ValueError('Expected COCO17 x,y,confidence and xyxy box')
        item=self.tracks.setdefault(track_id,{'last':t,'hands':[deque(),deque()]})
        if t<=item['last'] and any(item['hands']):return None
        if t-item['last']>self.config.max_gap_sec:
            item['hands']=[deque(),deque()]
        item['last']=t
        if not np.isfinite(kp[[5,6]]).all() or np.any(kp[[5,6],2]<self.config.confidence):
            item['hands']=[deque(),deque()];return None
        shoulder_x=float(np.mean(kp[[5,6],0]));shoulder_y=float(np.mean(kp[[5,6],1]));width=box[2]-box[0]
        for side,wrist in enumerate([9,10]):
            history=item['hands'][side]
            if not np.isfinite(kp[wrist]).all() or kp[wrist,2]<self.config.confidence or kp[wrist,1]>shoulder_y:
                history.clear();continue
            history.append((t,(float(kp[wrist,0])-shoulder_x)/width))
            while history and t-history[0][0]>self.config.window_sec:history.popleft()
            if len(history)<self.config.min_samples:continue
            values=np.array([x for _,x in history]);deltas=np.diff(values)
            signs=np.sign(deltas[np.abs(deltas)>=self.config.direction_epsilon])
            reversals=int(np.sum(signs[1:]!=signs[:-1]))
            amplitude=float(np.max(values)-np.min(values))
            if amplitude>=self.config.min_amplitude and reversals>=self.config.min_reversals:
                return {'track_id':track_id,'stamp':t,'hand':'left' if side==0 else 'right',
                        'amplitude_body_widths':amplitude,'direction_reversals':reversals,'bbox':box.tolist()}
        return None


def position_from_depth(u,v,depth_m,fx,fy,cx,cy):
    values=[u,v,depth_m,fx,fy,cx,cy]
    if not all(math.isfinite(x) for x in values) or depth_m<=0 or fx<=0 or fy<=0:raise ValueError('Invalid depth/intrinsics')
    return [(u-cx)*depth_m/fx,(v-cy)*depth_m/fy,depth_m]


def sample_person_depth(depth,bbox,unit='m'):
    """Foreground median in the central torso ROI; invalid depth yields None.

    Aligned depth must be supplied by the caller. Like the source node, only
    the nearest 0.4m band of valid central depths is used to reduce background.
    """
    image=np.asarray(depth)
    if image.ndim!=2 or unit not in ['m','mm']:raise ValueError('Expected a 2D aligned depth image and m/mm unit')
    box=np.asarray(bbox,dtype=float)
    if box.shape!=(4,) or not np.isfinite(box).all():raise ValueError('Invalid bounding box')
    x1,y1,x2,y2=box;h,w=image.shape
    if not 0<=x1<x2<=w or not 0<=y1<y2<=h:raise ValueError('Box outside aligned depth')
    left=int(x1+(x2-x1)/3);right=max(left+1,int(x2-(x2-x1)/3))
    top=int(y1+(y2-y1)/3);bottom=max(top+1,int(y2-(y2-y1)/3))
    samples=image[top:bottom,left:right].astype(float)*(1. if unit=='m' else .001)
    valid=samples[np.isfinite(samples)&(samples>=.2)&(samples<10.)]
    if not valid.size:return None
    return float(np.median(valid[valid<=np.min(valid)+.4]))
