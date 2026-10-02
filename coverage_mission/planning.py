"""Small conservative ground grid, serpentine obligations and safe connectors."""
import heapq
import math
import cv2
import numpy as np
from scipy import ndimage


def disk(radius, resolution):
    k=math.ceil(radius/resolution)
    y,x=np.mgrid[-k:k+1,-k:k+1]
    return ((x*x+y*y)*resolution**2 <= (radius+resolution*.71)**2).astype(np.uint8)


class GroundMap:
    def __init__(self,cfg):
        self.cfg=cfg
        self.shape=(math.ceil((cfg.n_max-cfg.n_min)/cfg.resolution),
                    math.ceil((cfg.e_max-cfg.e_min)/cfg.resolution))
        self.observed=np.zeros(self.shape,bool)
        self.red=np.zeros(self.shape,bool)
        self.confirmed=np.zeros(self.shape,bool)
        self.red_hits=np.zeros(self.shape,np.uint8)
        self.clear_hits=np.zeros(self.shape,np.uint8)
        self.visits=np.zeros(self.shape,np.uint16)
        self.last_t=-math.inf
        nn,ee=np.indices(self.shape)
        n=cfg.n_min+(nn+.5)*cfg.resolution
        e=cfg.e_min+(ee+.5)*cfg.resolution
        self.inset=(n>=cfg.n_min+cfg.clearance+cfg.resolution*.71)&(n<=cfg.n_max-cfg.clearance-cfg.resolution*.71)&(e>=cfg.e_min+cfg.clearance+cfg.resolution*.71)&(e<=cfg.e_max-cfg.clearance-cfg.resolution*.71)
        self.inflated=np.zeros(self.shape,bool)
        self.free=np.zeros(self.shape,bool)
        self.physical_red=np.zeros(self.shape,bool)
        self.enclosed=np.zeros(self.shape,bool)

    def cell(self,xy):
        return tuple(np.floor((np.asarray(xy)-[self.cfg.n_min,self.cfg.e_min])/self.cfg.resolution).astype(int))

    def point(self,cell):
        return np.array([self.cfg.n_min,self.cfg.e_min])+(np.asarray(cell)+.5)*self.cfg.resolution

    def contains(self,cell):
        return 0<=cell[0]<self.shape[0] and 0<=cell[1]<self.shape[1]

    def raster(self,polygon,padding=0):
        grid=np.zeros(tuple(s+2*padding for s in self.shape),np.uint8)
        pts=(np.asarray(polygon)-[self.cfg.n_min,self.cfg.e_min])/self.cfg.resolution-.5+padding
        if np.any(~np.isfinite(pts)):
            raise ValueError('Non-finite ground polygon')
        cv2.fillPoly(grid,[np.rint(pts[:,::-1]).astype(np.int32)],1)
        return grid

    def observe(self,footprint,reds,t):
        if t<=self.last_t:
            return False
        # Raster beyond field boundaries before shrinking the footprint. Otherwise
        # erosion would permanently hide the outermost field cells.
        visible=self.raster(footprint,padding=3)
        # Shrink visibility: full cell and a small optical edge reserve must be seen.
        visible=cv2.erode(visible,disk(self.cfg.resolution,self.cfg.resolution),borderType=cv2.BORDER_CONSTANT,borderValue=0)
        visible=visible[3:-3,3:-3].astype(bool)
        self.observed |= visible
        evidence=np.zeros(self.shape,bool)
        for polygon in reds:
            evidence |= self.raster(polygon).astype(bool)
        self.red_hits[evidence]=np.minimum(2,self.red_hits[evidence]+1)
        self.confirmed |= self.red_hits>=2
        self.clear_hits[evidence]=0
        contradicted=visible & ~evidence & (self.red_hits>0) & ~self.confirmed
        self.clear_hits[contradicted]=np.minimum(3,self.clear_hits[contradicted]+1)
        self.red_hits[self.clear_hits>=3]=0
        self.red=self.confirmed | (self.red_hits>0)
        self.last_t=t
        self.refresh()
        return True

    def refresh(self):
        r=self.cfg.resolution
        # A non-red island completely sealed by repeatedly confirmed red is
        # impossible to traverse without entering red. Unknown pixels alone
        # never form a barrier, and one-frame red evidence never exempts work.
        components,_=ndimage.label(~self.confirmed,structure=np.ones((3,3),np.uint8))
        boundary=np.unique(np.concatenate((components[0],components[-1],
                                           components[:,0],components[:,-1])))
        self.enclosed=(components>0)&~np.isin(components,boundary)
        self.inflated=cv2.dilate(self.red.astype(np.uint8),disk(self.cfg.clearance,r)).astype(bool)
        self.physical_red=cv2.dilate(self.red.astype(np.uint8),disk(self.cfg.body_radius,r)).astype(bool)
        known=cv2.erode(self.observed.astype(np.uint8),disk(self.cfg.clearance,r),
                       borderType=cv2.BORDER_CONSTANT,borderValue=0).astype(bool)
        self.free=known & self.inset & ~self.inflated

    def inside_red(self,xy):
        c=self.cell(xy)
        return bool(self.physical_red[c]) if self.contains(c) else None

    def line_clear(self,a,b,mask=None):
        mask=self.free if mask is None else mask
        # Supercover: check every touched grid cell, including exact diagonal corners.
        a=np.asarray(a); b=np.asarray(b)
        steps=max(1,math.ceil(np.linalg.norm(b-a)/(self.cfg.resolution*.25)))
        prev=None
        for xy in np.linspace(a,b,steps+1):
            c=self.cell(xy)
            if not self.contains(c) or not mask[c]: return False
            if prev and c[0]!=prev[0] and c[1]!=prev[1]:
                if not mask[c[0],prev[1]] or not mask[prev[0],c[1]]: return False
            prev=c
        return True


def astar(mask,start,goal,clearance=None,preference=.3):
    if not mask[start] or not mask[goal]: return []
    heap=[(math.dist(start,goal),0.,start)]
    best={start:0.}; parent={}
    rows,cols=mask.shape
    while heap:
        _,cost,p=heapq.heappop(heap)
        if cost>best[p]+1e-9: continue
        if p==goal:
            path=[p]
            while p!=start:
                p=parent[p]; path.append(p)
            return path[::-1]
        for di,dj in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
            q=(p[0]+di,p[1]+dj)
            if not (0<=q[0]<rows and 0<=q[1]<cols and mask[q]): continue
            if di and dj and not (mask[p[0]+di,p[1]] and mask[p[0],p[1]+dj]): continue
            trial=cost+math.hypot(di,dj)
            if clearance is not None:
                trial+=math.hypot(di,dj)*preference/max(clearance[q],.05)
            if trial+1e-9<best.get(q,math.inf):
                best[q]=trial; parent[q]=p
                heapq.heappush(heap,(trial+math.dist(q,goal),trial,q))
    return []


class CoveragePlan:
    def __init__(self,cfg):
        self.cfg=cfg
        # Adapted formula from coverage_ws/resolve_coverage_geometry.
        w,l=cfg.camera.footprint(cfg.altitude-cfg.altitude_tolerance-cfg.ground_above_home)
        w-=2*cfg.coverage_reserve; l-=2*cfg.coverage_reserve
        if min(w,l)<=2*cfg.clearance: raise ValueError('Footprint too small for safe coverage')
        self.spacing=w*(1-cfg.overlap)
        lo=cfg.e_min+w/2; hi=cfg.e_max-w/2
        lanes=np.linspace(lo,hi,max(1,math.ceil(max(0,hi-lo)/self.spacing)+1)) if hi>lo else [(cfg.e_min+cfg.e_max)/2]
        nlo=cfg.n_min+l/2; nhi=cfg.n_max-l/2
        if nhi<nlo: nlo=nhi=(cfg.n_min+cfg.n_max)/2
        self.points=[]; self.lanes=[]
        for i,e in enumerate(lanes):
            ns=np.linspace(nlo,nhi,max(2,math.ceil((nhi-nlo)/cfg.resolution)+1))
            if i%2: ns=ns[::-1]
            self.points.extend((n,e) for n in ns); self.lanes.extend([i]*len(ns))
        self.points=np.asarray(self.points)
        self.cells=np.floor((self.points-[cfg.n_min,cfg.e_min])/cfg.resolution).astype(int)
        self.done=np.zeros(len(self.points),bool)
        self.excluded=np.zeros(len(self.points),bool)
        self.last=None

    def update(self,pose,ground):
        cells=self.cells
        self.excluded=(ground.inflated[cells[:,0],cells[:,1]] |
                       ground.enclosed[cells[:,0],cells[:,1]])
        # Measured traversal only. Never credit a jump after missing observations.
        b=pose.xy
        a=b if self.last is None else self.last.xy
        if self.last is not None and (pose.t-self.last.t>.5 or np.linalg.norm(b-a)>1.): a=b
        delta=b-a; denom=float(delta@delta)
        f=np.clip((self.points-a)@delta/max(denom,1e-12),0,1)
        dist=np.linalg.norm(self.points-(a+f[:,None]*delta),axis=1)
        self.done |= (dist<=self.cfg.arrival)&~self.excluded&ground.observed[cells[:,0],cells[:,1]]
        self.last=pose

    def pending(self):
        return np.flatnonzero(~self.done & ~self.excluded)

    def reachable_target(self,ground,current,pending=None):
        """Prefer measured, reachable coverage over a frontier for an unknown row.

        A red region can split an ordered lane. Its first unknown point must not
        monopolize the planner while other required points are already safe to
        visit. Unknown obligations remain pending and are revisited by repair.
        """
        if pending is None: pending=self.pending()
        if not len(pending): return None
        start=ground.cell(current)
        if not ground.contains(start) or not ground.free[start]: return self.points[pending[0]]
        labels,_=ndimage.label(ground.free)
        cells=self.cells[pending]
        reachable=labels[cells[:,0],cells[:,1]]==labels[start]
        candidates=pending[reachable]
        if not len(candidates): return self.points[pending[0]]
        distances=np.linalg.norm(self.points[candidates]-np.asarray(current),axis=1)
        nearest=candidates[int(np.argmin(distances))]
        first=pending[0]
        # Keep the normal ordered frontier when it is nearby. A distant
        # obligation must not pull the aircraft across the field, even if a
        # long checked connector exists, while safe work lies underneath it.
        # One camera half-swath
        # is the natural scale for that choice, independent of the lens.
        width,length=self.cfg.camera.footprint(self.cfg.altitude-self.cfg.altitude_tolerance-self.cfg.ground_above_home)
        if np.linalg.norm(self.points[first]-np.asarray(current)) <= distances.min()+max(width,length)/2:
            return self.points[first]
        return self.points[nearest]


def route(ground,current,target):
    start=ground.cell(current); goal=ground.cell(target)
    if not ground.contains(start) or not ground.free[start]: return []
    if ground.contains(goal) and ground.line_clear(current,target): return [np.asarray(target)]
    # Four-connected components agree with no-corner-cutting routes.
    labels,_=ndimage.label(ground.free)
    reachable=labels==labels[start]
    clearance=ndimage.distance_transform_edt(ground.free)*ground.cfg.resolution
    if not (ground.contains(goal) and reachable[goal]):
        # Frontier viewpoints have unseen ground in a camera-sized neighbourhood.
        w,l=ground.cfg.camera.footprint(ground.cfg.altitude-ground.cfg.altitude_tolerance)
        window=(max(3,int(l/ground.cfg.resolution)),max(3,int(w/ground.cfg.resolution)))
        gain=ndimage.uniform_filter((~ground.observed).astype(float),size=window,mode='constant')
        candidates=np.argwhere(reachable & (gain>.01))
        if not len(candidates): return []
        points=np.array([ground.point(c) for c in candidates])
        # On a field spanning many camera swaths, an ordered unknown row can
        # repeatedly force whole-field A* searches. Extend the observed map
        # locally there; keep the ordered-frontier preference on compact test
        # fields. No obligation is credited until actually traversed.
        large_field=max(ground.cfg.n_max-ground.cfg.n_min,
                        ground.cfg.e_max-ground.cfg.e_min)>4*max(w,l)
        if large_field:
            scores=np.linalg.norm(points-current,axis=1)+.15*np.linalg.norm(points-target,axis=1)
        else:
            scores=np.linalg.norm(points-target,axis=1)+.15*np.linalg.norm(points-current,axis=1)
        scores+=ground.visits[candidates[:,0],candidates[:,1]]*.3
        scores-=gain[candidates[:,0],candidates[:,1]]*2
        # Prefer observable viewpoints with tracking room over a one-cell sliver.
        # This is a route cost, not removal of required coverage or smaller margins.
        scores+=ground.cfg.uncertainty/np.maximum(clearance[candidates[:,0],candidates[:,1]],.05)
        goal=tuple(candidates[int(np.argmin(scores))])
    path=astar(ground.free,start,goal,clearance,ground.cfg.uncertainty)
    if not path: return []
    points=[ground.point(c) for c in path[1:]] or [ground.point(goal)]
    # Only shorten if the entire connector remains checked.
    out=[]; anchor=np.asarray(current); i=0
    while i<len(points):
        j=i
        while j+1<len(points):
            reserve=min(ground.cfg.uncertainty,clearance[ground.cell(anchor)],clearance[ground.cell(points[j+1])])
            if not ground.line_clear(anchor,points[j+1],ground.free & (clearance>=reserve-1e-9)): break
            j+=1
        out.append(points[j]); anchor=points[j]; i=j+1
    return out
