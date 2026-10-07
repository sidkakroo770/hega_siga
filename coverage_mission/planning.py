"""Small conservative ground grid, serpentine obligations and safe connectors."""
import heapq
import math
import time
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
        self.contextual_clear=np.zeros(self.shape,bool)
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
        self._raster_scratch=np.zeros(self.shape,np.uint8)

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
        evidence=self._raster_scratch
        evidence.fill(0)
        for polygon in reds:
            pts=(np.asarray(polygon)-[self.cfg.n_min,self.cfg.e_min])/self.cfg.resolution-.5
            if np.any(~np.isfinite(pts)): raise ValueError('Non-finite ground polygon')
            cv2.fillPoly(evidence,[np.rint(pts[:,::-1]).astype(np.int32)],1)
        return self._apply_observation(visible,evidence.astype(bool),t)

    def observe_pixels(self,footprint,red_pixels,usable_pixels,t):
        """Project a planar image once; unreadable pixels never certify clear ground.

        This is the pinhole/flat-ground homography. It replaces hundreds of
        contour projections/raster allocations on fragmented red imagery.
        """
        if t<=self.last_t: return False
        h,w=red_pixels.shape
        source=np.float32([[0,0],[w-1,0],[w-1,h-1],[0,h-1]])
        projected=np.asarray(footprint,np.float32)
        destination=np.float32([[(e-self.cfg.e_min)/self.cfg.resolution-.5,
                                 (n-self.cfg.n_min)/self.cfg.resolution-.5]
                                for n,e in projected])
        transform=cv2.getPerspectiveTransform(source,destination)
        size=(self.shape[1],self.shape[0])
        visible=cv2.warpPerspective(usable_pixels,transform,size,flags=cv2.INTER_NEAREST)>0
        visible=cv2.erode(visible.astype(np.uint8),disk(self.cfg.resolution,self.cfg.resolution),
                          borderType=cv2.BORDER_CONSTANT,borderValue=1).astype(bool)
        # A one-pixel image reserve avoids dropping thin red boundaries during
        # nearest-neighbour sampling; no expensive contour extraction is needed.
        red=cv2.dilate(red_pixels,np.ones((3,3),np.uint8))
        evidence=cv2.warpPerspective(red,transform,size,flags=cv2.INTER_NEAREST)>0
        return self._apply_observation(visible,evidence,t)

    def _apply_observation(self,visible,evidence,t):
        self.observed |= visible
        evidence &= visible
        self.red_hits[evidence]=np.minimum(self.cfg.red_confirm_frames,self.red_hits[evidence]+1)
        self.confirmed |= self.red_hits>=self.cfg.red_confirm_frames
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
        # A body-clearance-inflated confirmed zone can seal even a narrow raw
        # opening. Unknown ground is NOT an exclusion barrier. One-frame red
        # remains a motion barrier, but cannot exempt coverage obligations.
        confirmed_barrier=cv2.dilate(self.confirmed.astype(np.uint8),
                                     disk(self.cfg.clearance,r)).astype(bool)
        components,_=ndimage.label(~confirmed_barrier)
        boundary=np.unique(np.concatenate((components[0],components[-1],
                                           components[:,0],components[:,-1])))
        self.enclosed=confirmed_barrier | ((components>0)&~np.isin(components,boundary))
        self.inflated=cv2.dilate(self.red.astype(np.uint8),disk(self.cfg.clearance,r)).astype(bool)
        self.physical_red=cv2.dilate(self.red.astype(np.uint8),disk(self.cfg.body_radius,r)).astype(bool)
        self.contextual_clear[:]=False
        if self.cfg.small_unknown_hole_area:
            # A few unreadable cells fully surrounded by observed non-red
            # ground (such as black modules on a printed marker) cannot block
            # an entire lane after the clearance erosion. Keep the inference
            # separate from direct observation; never bridge a large, edge-
            # connected, or red-adjacent unknown region.
            components,count=ndimage.label(~self.observed)
            if count:
                sizes=np.bincount(components.ravel())
                boundary=np.unique(np.concatenate((components[0],components[-1],
                                                   components[:,0],components[:,-1])))
                small=sizes<=max(1,int(self.cfg.small_unknown_hole_area/(r*r)))
                small[boundary]=False
                small[np.unique(components[self.inflated])]=False
                small[0]=False
                self.contextual_clear=small[components]
        known=cv2.erode((self.observed|self.contextual_clear).astype(np.uint8),disk(self.cfg.clearance,r),
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
        samples=a[None,:]+np.linspace(0.,1.,steps+1)[:,None]*(b-a)
        cells=np.floor((samples-[self.cfg.n_min,self.cfg.e_min])/self.cfg.resolution).astype(int)
        if (np.any(cells<0) or np.any(cells[:,0]>=self.shape[0])
                or np.any(cells[:,1]>=self.shape[1])): return False
        if not np.all(mask[cells[:,0],cells[:,1]]): return False
        diagonal=np.flatnonzero(np.all(cells[1:]!=cells[:-1],axis=1))+1
        if not len(diagonal): return True
        previous=cells[diagonal-1]; current=cells[diagonal]
        return bool(np.all(mask[current[:,0],previous[:,1]])
                    and np.all(mask[previous[:,0],current[:,1]]))


def astar(mask,start,goal,clearance=None,preference=.3,deadline=None):
    if not mask[start] or not mask[goal]: return []
    heap=[(math.dist(start,goal),0.,start)]
    best={start:0.}; parent={}
    rows,cols=mask.shape
    expanded=0
    while heap:
        expanded+=1
        if deadline is not None and expanded%128==0 and time.monotonic()>=deadline:
            return None  # Distinct from a proven no-route result.
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
        self.done |= ((dist<=self.cfg.arrival)&~self.excluded&
                      (ground.observed|ground.contextual_clear)[cells[:,0],cells[:,1]])
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
        chosen=(first if reachable[0] and np.linalg.norm(self.points[first]-np.asarray(current))
                <= distances.min()+max(width,length)/2 else nearest)
        # Dense 0.1 m points are traversal evidence, not stop-and-go flight
        # waypoints. On a checked straight lane, pursue a visible look-ahead
        # point while the intervening obligations remain measured normally.
        # Never look across a lane turn, exclusion or unchecked ground.
        if not ground.free[tuple(self.cells[chosen])] or not ground.line_clear(current,self.points[chosen]):
            return self.points[chosen]
        limit=max(1,int(max(width,length)/(2*self.cfg.resolution)))
        end=min(len(self.points),chosen+limit+1)
        best=chosen
        for index in range(chosen+1,end):
            if self.lanes[index]!=self.lanes[chosen] or self.excluded[index] or self.done[index]: break
            if not ground.free[tuple(self.cells[index])] or not ground.line_clear(current,self.points[index]): break
            best=index
        return self.points[best]


def route(ground,current,target,budget_s=None):
    deadline=time.monotonic()+budget_s if budget_s is not None else None
    def over_budget():
        return deadline is not None and time.monotonic()>=deadline
    start=ground.cell(current); goal=ground.cell(target)
    if not ground.contains(start) or not ground.free[start]: return []
    if ground.contains(goal) and ground.line_clear(current,target): return [np.asarray(target)]
    # Four-connected components agree with no-corner-cutting routes.
    labels,_=ndimage.label(ground.free)
    reachable=labels==labels[start]
    clearance=ndimage.distance_transform_edt(ground.free)*ground.cfg.resolution
    if (ground.contains(goal) and not reachable[goal] and ground.observed[goal]
            and not ground.inflated[goal] and not ground.enclosed[goal]):
        # A required lane point can be observed but its *cell* unavailable
        # because the clearance erosion touches a tiny unreadable patch.
        # Traversal is credited by measured position within the arrival disk,
        # so target a genuinely free, connected cell in that disk if one
        # exists. Never route to the unfree cell or credit it from the map.
        radius=math.ceil(ground.cfg.arrival/ground.cfg.resolution)
        lo=np.maximum(np.asarray(goal)-radius,0)
        hi=np.minimum(np.asarray(goal)+radius+1,ground.shape)
        nearby=np.argwhere(reachable[lo[0]:hi[0],lo[1]:hi[1]])+lo
        if len(nearby):
            points=np.array([ground.point(c) for c in nearby])
            distances=np.linalg.norm(points-np.asarray(target),axis=1)
            within=np.flatnonzero(distances<=ground.cfg.arrival-ground.cfg.resolution/3)
            if len(within): goal=tuple(nearby[within[int(np.argmin(distances[within]))]])
    if not (ground.contains(goal) and reachable[goal]):
        # Frontier viewpoints have unseen ground in a camera-sized neighbourhood.
        w,l=ground.cfg.camera.footprint(ground.cfg.altitude-ground.cfg.altitude_tolerance)
        window=(max(3,int(l/ground.cfg.resolution)),max(3,int(w/ground.cfg.resolution)))
        unknown=~ground.observed & ~ground.contextual_clear
        components,_=ndimage.label(unknown)
        boundary=np.unique(np.concatenate((components[0],components[-1],
                                           components[:,0],components[:,-1])))
        exterior=unknown & np.isin(components,boundary)
        # Prefer the still-unexplored field perimeter to small dark texture
        # holes inside an otherwise observed swath. Those holes remain unknown
        # obligations; they are only considered after exterior exploration.
        frontier=exterior if exterior.any() else unknown
        gain=ndimage.uniform_filter(frontier.astype(float),size=window,mode='constant')
        candidates=np.argwhere(reachable & (gain>.01))
        if not len(candidates): return []
        points=np.array([ground.point(c) for c in candidates])
        # A viewpoint inside the present arrival disk cannot expose a new
        # swath. Selecting the current grid cell otherwise yields a permanent
        # self-route even while distant safe frontier cells exist.
        minimum=max(2*ground.cfg.arrival,min(w,l)/4)
        useful=np.linalg.norm(points-np.asarray(current),axis=1)>=minimum
        if not np.any(useful): return []
        candidates=candidates[useful]; points=points[useful]
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
        # A nearby checked viewpoint needs no graph search. This matters on
        # large fields: a remote high-gain frontier can consume the whole
        # planner budget while a directly visible frontier still offers safe
        # progress. Inspect only the best few scores to bound CPU work.
        count=min(24,len(scores))
        shortlist=np.argpartition(scores,count-1)[:count]
        shortlist=shortlist[np.argsort(scores[shortlist])]
        for index in shortlist:
            viewpoint=points[index]
            if ground.line_clear(current,viewpoint):
                return [viewpoint]
        goal=tuple(candidates[int(shortlist[0])])
    if over_budget(): return None
    points=None
    # A conservative hierarchy reduces Python heap work on large fields.
    # Every coarse cell requires all its fine cells free, and every emitted
    # connector is checked on the original grid. Narrow passages fall back to
    # full-resolution search; no coarse approximation earns clearance credit.
    if min(ground.shape)>=100:
        for factor in (3,2):
            if over_budget(): return None
            nr,nc=ground.shape[0]//factor,ground.shape[1]//factor
            blocks=ground.free[:nr*factor,:nc*factor].reshape(nr,factor,nc,factor)
            coarse=blocks.all(axis=(1,3))
            cs=(start[0]//factor,start[1]//factor)
            cg=(goal[0]//factor,goal[1]//factor)
            if not (cs[0]<nr and cs[1]<nc and cg[0]<nr and cg[1]<nc
                    and coarse[cs] and coarse[cg]):
                continue
            coarse_clearance=ndimage.distance_transform_edt(coarse)*factor*ground.cfg.resolution
            # The free mask already enforces body plus uncertainty clearance.
            # A further inverse-clearance path cost makes A* expand most of a
            # large field for a distant frontier, repeatedly exhausting the
            # worker's wall-time budget. Keep the hard clearance and checked
            # segments; do not pay that soft cost during search.
            coarse_path=astar(coarse,cs,cg,coarse_clearance,0.,
                              deadline=deadline)
            if coarse_path is None: return None
            if coarse_path:
                offset=(factor-1)*ground.cfg.resolution*.5
                candidate=[ground.point((factor*i,factor*j))+np.array([offset,offset])
                           for i,j in coarse_path[1:]]
                candidate.append(ground.point(goal))
                anchor=np.asarray(current)
                if all(ground.line_clear(a,b) for a,b in
                       zip([anchor]+candidate[:-1],candidate)):
                    points=candidate
                    break
    if points is None:
        if over_budget(): return None
        path=astar(ground.free,start,goal,clearance,0.,
                   deadline=deadline)
        if path is None: return None
        if not path: return []
        points=[ground.point(c) for c in path[1:]] or [ground.point(goal)]
    # Only shorten if the entire connector remains checked. Binary search
    # bounds visibility probes on long routes; a non-monotone visibility
    # geometry can leave extra waypoints, never an unchecked shortcut.
    out=[]; anchor=np.asarray(current); i=0
    while i<len(points):
        if over_budget(): return None
        low=i; high=len(points)-1; j=i
        anchor_clearance=clearance[ground.cell(anchor)]
        while low<=high:
            mid=(low+high)//2
            reserve=min(ground.cfg.uncertainty,anchor_clearance,clearance[ground.cell(points[mid])])
            if ground.line_clear(anchor,points[mid],ground.free & (clearance>=reserve-1e-9)):
                j=mid; low=mid+1
            else:
                high=mid-1
        out.append(points[j]); anchor=points[j]; i=j+1
    return out
