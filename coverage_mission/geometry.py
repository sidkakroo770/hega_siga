"""Calibrated image rays intersect a known flat plane; no learned model."""
from dataclasses import dataclass
from collections import deque
import math
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


@dataclass(frozen=True)
class Pose:
    t: float
    n: float
    e: float
    alt: float
    roll: float = 0.
    pitch: float = 0.
    yaw: float = 0.
    vn: float = 0.
    ve: float = 0.
    vd: float = 0.

    @property
    def xy(self):
        return np.array([self.n, self.e])

    def valid(self):
        return all(math.isfinite(v) for v in self.__dict__.values())


class PoseHistory:
    def __init__(self, seconds=30.):
        self.samples = deque()
        self.seconds = seconds

    def add(self, pose):
        if not pose.valid():
            raise ValueError('Invalid pose')
        if self.samples and pose.t < self.samples[-1].t:
            self.samples.clear()
            raise ValueError('Pose clock reset; map must be discarded')
        if self.samples and pose.t == self.samples[-1].t:
            return
        self.samples.append(pose)
        while self.samples and pose.t - self.samples[0].t > self.seconds:
            self.samples.popleft()

    def at(self, t, max_gap=.15):
        pts = list(self.samples)
        for p in pts:
            if abs(p.t-t) < 1e-8:
                return p
        for a, b in zip(pts, pts[1:]):
            if a.t <= t <= b.t:
                if b.t-a.t > max_gap:
                    return None
                f = (t-a.t)/(b.t-a.t)
                rot = Rotation.from_euler('xyz', [[a.roll,a.pitch,a.yaw],
                                                   [b.roll,b.pitch,b.yaw]])
                r,p,y = Slerp([a.t,b.t],rot)([t]).as_euler('xyz')[0]
                vals = [getattr(a,k)*(1-f)+getattr(b,k)*f
                        for k in ('n','e','alt')]
                vel = [getattr(a,k)*(1-f)+getattr(b,k)*f
                       for k in ('vn','ve','vd')]
                return Pose(t,*vals,r,p,y,*vel)
        return None  # Never extrapolate a stale/latest pose onto an image.


class Projector:
    def __init__(self, cfg):
        self.cfg = cfg
        c = cfg.camera
        self.K = np.array([[c.fx,0,c.cx], [0,c.fy,c.cy], [0,0,1.]])
        optical_to_body = np.array([[0,-1,0],[1,0,0],[0,0,1.]])
        self.mount = Rotation.from_euler('xyz',
            [c.mount_roll,c.mount_pitch,c.mount_yaw]).as_matrix() @ optical_to_body

    def project(self, pixels, pose):
        if not pose.valid():
            raise ValueError('Invalid pose')
        c = self.cfg.camera
        h = pose.alt - self.cfg.ground_above_home - c.down_offset
        if h <= 0 or abs(pose.roll)>self.cfg.max_tilt or abs(pose.pitch)>self.cfg.max_tilt:
            raise ValueError('Ground/attitude outside projection envelope')
        pixels = np.asarray(pixels, np.float64).reshape(-1,1,2)
        uv = cv2.undistortPoints(pixels, self.K, np.asarray(c.distortion)).reshape(-1,2)
        rays = np.column_stack([uv,np.ones(len(uv))])
        R = Rotation.from_euler('xyz',[pose.roll,pose.pitch,pose.yaw]).as_matrix()
        directions = rays @ (R @ self.mount).T
        if np.any(directions[:,2] <= .2):
            raise ValueError('Ground rays near/above horizon')
        return pose.xy + directions[:,:2] * (h/directions[:,2,None])

    def footprint(self, pose):
        c=self.cfg.camera
        return self.project([[0,0],[c.width,0],[c.width,c.height],[0,c.height]],pose)


class UnusableImage(ValueError):
    pass


def validated_hsv(frame, cfg):
    if frame.shape != (cfg.camera.height,cfg.camera.width,3) or frame.dtype != np.uint8:
        raise ValueError('Image dimensions/encoding do not match calibration')
    hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    if not cfg.assume_nonred_ground_clear:
        usable=(hsv[:,:,2]>=cfg.red_value_min)&((hsv[:,:,2]<=250)|(hsv[:,:,1]>=cfg.red_saturation_min))
        if np.count_nonzero(usable)<frame.shape[0]*frame.shape[1]*.02:
            raise UnusableImage('Dark/saturated frame is not free-ground evidence')
    return hsv


def red_mask(hsv,cfg):
    return (cv2.inRange(hsv,(0,cfg.red_saturation_min,cfg.red_value_min),
                        (cfg.red_hue_low_max,255,255)) |
            cv2.inRange(hsv,(cfg.red_hue_high_min,cfg.red_saturation_min,cfg.red_value_min),
                        (179,255,255)))


def red_regions(frame, cfg):
    hsv=validated_hsv(frame,cfg)
    mask=red_mask(hsv,cfg)
    # Do not erode small hazards away. Ignore only sub-pixel/degenerate contours.
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    polygons=[]
    for contour in contours:
        if cv2.contourArea(contour)<2:
            continue
        polygon=cv2.approxPolyDP(contour,.6,True).reshape(-1,2)
        if len(polygon)>=3:
            polygons.append(polygon)
    return mask,polygons


def decode_image(msg):
    """Gazebo packed or padded RGB/BGR only; grey cannot support red detection."""
    if msg.pixel_format_type not in (3,8):
        raise ValueError('RGB_INT8 or BGR_INT8 required')
    w,h=int(msg.width),int(msg.height)
    stride=int(msg.step) or w*3
    if w<=0 or h<=0 or stride<w*3 or len(msg.data)!=h*stride:
        raise ValueError('Malformed image stride/length')
    data=np.frombuffer(msg.data,np.uint8).reshape(h,stride)[:,:w*3].reshape(h,w,3)
    return cv2.cvtColor(data,cv2.COLOR_RGB2BGR) if msg.pixel_format_type==3 else data.copy()
