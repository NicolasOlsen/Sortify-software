"""
Module: shape_tracker.py
Handles: Object ID tracking, Kalman filtering, association logic
Owns: Track creation, update, matching, loss handling
Calls: config sliders, optional scipy
Does not contain: detection, color masking, 3D position estimation
"""

import numpy as np, cv2
from dataclasses import dataclass, field
from typing import Dict, List, Any, Tuple
from scipy.optimize import linear_sum_assignment




@dataclass
class ShapeTrack:
    data: Dict[str, Any]
    age: int = 1
    lost: int = 0
    id: int = -1
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    kf: Any = field(default=None, repr=False)

class ShapeTracker:
    id_counter: int = 0

    def __init__(self) -> None:
        self.tracks: Dict[str, List[ShapeTrack]] = {}
        self._candidates: Dict[str, List[Dict[str, Any]]] = {}

    @classmethod
    def next_id(cls) -> int:
        cls.id_counter += 1
        return cls.id_counter

    @staticmethod
    def track_distance(tr: ShapeTrack, det: Dict[str, Any]) -> float:
        if {"x", "y", "z"} <= det.keys() <= tr.data.keys():
            dx, dy, dz = tr.data["x"]-det["x"], tr.data["y"]-det["y"], tr.data["z"]-det["z"]
            return float(np.sqrt(dx*dx+dy*dy+dz*dz))
        cx_tr, cy_tr = tr.data.get("cx", tr.data.get("x",0.0)), tr.data.get("cy", tr.data.get("y",0.0))
        cx_dt, cy_dt = det.get("cx", det.get("x",0.0)), det.get("cy", det.get("y",0.0))
        return float(np.hypot(cx_tr-cx_dt, cy_tr-cy_dt))
    
    @staticmethod
    def distance(d1: Dict[str,Any], d2:Dict[str,Any]) -> float:
        if {"x","y","z"} <= d1.keys() <= d2.keys():
            dx,dy,dz = d1["x"]-d2["x"], d1["y"]-d2["y"], d1["z"]-d2["z"]
            return float(np.sqrt(dx*dx+dy*dy+dz*dz))
        cx1, cy1 = d1.get("cx", d1.get("x",0.0)), d1.get("cy", d1.get("y",0.0))
        cx2, cy2 = d2.get("cx", d2.get("x",0.0)), d2.get("cy", d2.get("y",0.0))
        return float(np.hypot(cx1-cx2, cy1-cy2))

    @staticmethod
    def _init_kf(det, q2d, r2d, q3d, r3d) -> cv2.KalmanFilter:
        if {"x","y","z"} <= det.keys():
            kf = cv2.KalmanFilter(6,3,0,cv2.CV_32F)
            dt=1.0
            kf.transitionMatrix = np.array(
                [[1,0,0,dt,0,0],[0,1,0,0,dt,0],[0,0,1,0,0,dt],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]],
                dtype=np.float32)
            kf.measurementMatrix = np.eye(3,6,dtype=np.float32)
            kf.processNoiseCov   = np.eye(6,dtype=np.float32)*q3d
            kf.measurementNoiseCov = np.eye(3,dtype=np.float32)*r3d
            kf.errorCovPost = np.eye(6,dtype=np.float32)
            kf.statePost[:3,0] = np.array([det["x"],det["y"],det["z"]],dtype=np.float32)
            return kf
        kf = cv2.KalmanFilter(4,2,0,cv2.CV_32F); dt=1.0
        kf.transitionMatrix = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]],dtype=np.float32)
        kf.measurementMatrix = np.eye(2,4,dtype=np.float32)
        kf.processNoiseCov   = np.eye(4,dtype=np.float32)*q2d
        kf.measurementNoiseCov = np.eye(2,dtype=np.float32)*r2d
        kf.errorCovPost = np.eye(4,dtype=np.float32)
        kf.statePost[:2,0] = np.array([det.get("cx",det.get("x",0)),det.get("cy",det.get("y",0))],dtype=np.float32)
        return kf

    def clear(self) -> None:
        self.tracks.clear(); self._candidates.clear(); ShapeTracker.id_counter=0

    def track(self, shape:str, color:str, detections:List[Dict[str,Any]], params:Dict[str,Any]) -> List[Dict[str,Any]]:
        key=f"{shape}_{color}"
        tracks=self.tracks.get(key,[])
        cands=self._candidates.get(key,[])

        alpha      = params.get("TRAlpha",5)/100.0
        base_valid = params.get("MatchDist",1500)
        max_lost   = params.get("MaxLost",30)
        spawn_need = params.get("SpawnPersist",3)
        speed_gain = params.get("SpeedGain",0)/10.0
        lost_gain  = params.get("LostGain",5)
        stable_age = params.get("StableAge",2)
        q2d,r2d,q3d,r3d = (params.get("KF Q 2D",2)/1000.0,
                           params.get("KF R 2D",5)/1000.0,
                           params.get("KF Q 3D",10)/1000.0,
                           params.get("KF R 3D",9)/1000.0)

        for tr in tracks:
            if tr.kf is None: continue
            if tr.kf.statePre.shape[0]==6:
                tr.kf.processNoiseCov   = np.eye(6,dtype=np.float32)*q3d
                tr.kf.measurementNoiseCov=np.eye(3,dtype=np.float32)*r3d
            else:
                tr.kf.processNoiseCov   = np.eye(4,dtype=np.float32)*q2d
                tr.kf.measurementNoiseCov=np.eye(2,dtype=np.float32)*r2d
            pred=tr.kf.predict()
            if pred.shape[0]==6:
                tr.data.update({"x":float(pred[0]),"y":float(pred[1]),"z":float(pred[2])})
                tr.vx,tr.vy,tr.vz=map(float,pred[3:6,0])
            else:
                tr.data.update({"cx":float(pred[0]),"cy":float(pred[1])})
                tr.vx,tr.vy=float(pred[2]),float(pred[3]); tr.vz=0.0

        matched_t, matched_d=set(),set()
        if tracks and detections and linear_sum_assignment is not None:
            m,n=len(tracks),len(detections); BIG=1e6
            cost=np.full((m,n),BIG,dtype=np.float32)
            for ti,tr in enumerate(tracks):
                adapt_th=base_valid+speed_gain*np.linalg.norm([tr.vx,tr.vy,tr.vz])+lost_gain*tr.lost
                for di,det in enumerate(detections):
                    d=self.track_distance(tr,det)
                    if d<=adapt_th: cost[ti,di]=d
            for ti,di in zip(*linear_sum_assignment(cost)):
                if cost[ti,di]>=BIG: continue
                tr=tracks[ti]; det=detections[di]

                z = []
                for k in ("x", "y", "z"):
                    if k in det:
                        z.append(det[k])
                    elif k == "x" and "cx" in det:
                        z.append(det["cx"])
                    elif k == "y" and "cy" in det:
                        z.append(det["cy"])
                z = np.array(z, dtype=np.float32).reshape(-1, 1)

                if tr.kf is not None and z.size == tr.kf.measurementMatrix.shape[0]:
                    tr.kf.correct(z)                
                for k,v in det.items():
                    tr.data[k]=v if k in("x","y","z","cx","cy") else (tr.data.get(k,0)*(1-alpha)+v*alpha if isinstance(v,(int,float)) else v)
                tr.age+=1; tr.lost=0; matched_t.add(ti); matched_d.add(di)

        for ti,tr in enumerate(tracks):
            if ti not in matched_t: tr.lost+=1

        new_cands=[]
        for di,det in enumerate(detections):
            if di in matched_d or det.get("r",0)<25: continue
            for cand in cands:
                if self.distance(cand["det"],det)<=base_valid:
                    cand["det"]=det; cand["seen"]+=1; break
            else:
                new_cands.append({"det":det,"seen":1})
        updated_cands=[]
        for cand in cands+new_cands:
            if cand["seen"]>=spawn_need:
                tracks.append(ShapeTrack(data=dict(cand["det"]),id=self.next_id(),kf=self._init_kf(cand["det"],q2d,r2d,q3d,r3d)))
            else:
                updated_cands.append(cand)

        self._candidates[key]=updated_cands
        self.tracks[key]=[tr for tr in tracks if tr.lost<=max_lost]
        return [dict(tr.data,track_id=tr.id,age=tr.age) for tr in self.tracks[key] if tr.lost==0 and tr.age>=stable_age]