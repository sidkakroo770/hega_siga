"""Geometry-only discovery and explicitly authorized stationary QR decoding."""
import cv2
import numpy as np


class QRDetector:
    def __init__(self, limits=None):
        self.detector = cv2.QRCodeDetector()
        self.limits = limits

    def valid_quad(self, quad, image_shape):
        """Reject extrapolated/degenerate finder geometry before projection.

        The downward camera admits near-level ground observations. These broad
        shape gates tolerate rotation and modest perspective, without assuming
        a physical print size, QR version, or payload.
        """
        p=np.asarray(quad,np.float32)
        if p.shape!=(4,2) or not np.isfinite(p).all(): return False
        h,w=image_shape[:2]
        margin=getattr(self.limits,'quad_frame_margin_px',3.)
        if (p[:,0].min()<margin or p[:,0].max()>w-1-margin or
                p[:,1].min()<margin or p[:,1].max()>h-1-margin): return False
        if not cv2.isContourConvex(p) or cv2.contourArea(p)<64: return False
        edges=np.roll(p,-1,axis=0)-p
        sides=np.linalg.norm(edges,axis=1)
        if (sides.min()<getattr(self.limits,'quad_min_side_px',8.) or
                sides.max()/sides.min()>getattr(self.limits,'quad_max_side_ratio',1.8)):
            return False
        cosine=np.sum(edges*(-np.roll(edges,1,axis=0)),axis=1)/(sides*np.roll(sides,1))
        bound=np.cos(np.deg2rad(getattr(self.limits,'quad_min_angle_deg',45.)))
        return bool(np.max(np.abs(cosine))<=bound)

    def candidates(self, frame):
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, points = self.detector.detectMulti(gray)
        if not ok or points is None:
            ok, points = self.detector.detect(gray)
        if not ok or points is None: return []
        result = []
        for quad in np.asarray(points).reshape(-1, 4, 2):
            if not self.valid_quad(quad,gray.shape): continue
            result.append({'quad': quad.tolist(), 'center': quad.mean(axis=0).tolist(),
                           'complete': True})
        return result

    @staticmethod
    def decode_selected(frame, quad):
        """Bounded selected-marker crop; strict UTF-8, no online URL fetch."""
        from pyzbar.pyzbar import decode, ZBarSymbol
        quad = np.asarray(quad, np.float32)
        margin = max(8, int(np.max(np.ptp(quad, axis=0))*.22))
        lo = np.maximum(0, np.floor(quad.min(axis=0)).astype(int)-margin)
        hi = np.minimum([frame.shape[1], frame.shape[0]],
                        np.ceil(quad.max(axis=0)).astype(int)+margin+1)
        crop = frame[lo[1]:hi[1], lo[0]:hi[0]]
        gray = crop if crop.ndim == 2 else cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
        codes = decode(gray, symbols=[ZBarSymbol.QRCODE])
        if len(codes) != 1: return None, 'unreadable' if not codes else 'ambiguous'
        rect = codes[0].rect
        center = np.array([rect.left+rect.width/2, rect.top+rect.height/2])+lo
        if np.linalg.norm(center-quad.mean(axis=0)) > max(8, .25*np.max(np.ptp(quad,axis=0))):
            return None, 'different marker in crop'
        try: text = codes[0].data.decode('utf-8', errors='strict')
        except UnicodeDecodeError: return None, 'unsupported binary payload'
        return (text, 'text') if text else (None, 'empty payload')

    def detect(self, frame, *, allow_decode=False, debug=True):
        found = self.candidates(frame)
        first = found[0] if found else None
        center = first['center'] if first else None
        payload = ''
        if first and first['complete'] and allow_decode:
            payload = self.decode_selected(frame, first['quad'])[0] or ''
        view = frame.copy() if debug else None
        if debug and first:
            cv2.polylines(view, [np.asarray(first['quad'],np.int32)], True, (0,255,0), 2)
        return {'detected': bool(first), 'qr_data': payload, 'center': center,
                'error_x': center[0]-frame.shape[1]/2 if center else 0,
                'error_y': center[1]-frame.shape[0]/2 if center else 0,
                'debug_frame': view, 'candidates': found}
