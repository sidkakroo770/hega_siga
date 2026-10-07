"""Rigid 2-D registration between FC local NED and the surveyed field rectangle."""
import math
import numpy as np
from dataclasses import replace


class FieldFrame:
    def __init__(self,cfg):
        self.origin=np.array([cfg.field_origin_n,cfg.field_origin_e],float)
        self.yaw=cfg.field_yaw
        c,s=math.cos(self.yaw),math.sin(self.yaw)
        self.to_local=np.array([[c,-s],[s,c]])

    def point(self,local_ne):
        return self.to_local.T @ (np.asarray(local_ne)-self.origin)

    def local_point(self,field_ne):
        return self.origin+self.to_local @ np.asarray(field_ne)

    def vector(self,local_ne):
        return self.to_local.T @ np.asarray(local_ne)

    def local_vector(self,field_ne):
        return self.to_local @ np.asarray(field_ne)

    def pose(self,pose):
        n,e=self.point([pose.n,pose.e])
        vn,ve=self.vector([pose.vn,pose.ve])
        return replace(pose,n=n,e=e,vn=vn,ve=ve,yaw=pose.yaw-self.yaw)
