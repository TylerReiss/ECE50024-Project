# SceneModelClass.py
# Class that models the scene

from types import SimpleNamespace
from scipy.io import loadmat
import numpy as np


class SceneModelClass:
    def __init__(self,radarList,targetList,updatePeriod,simTime,stdTargetAccuracy_m):
        self.RadarList      = radarList
        self.targetList     = targetList
        self.t_s            = 0
        self.updatePeriod_s = updatePeriod
        self.simTime_s      = simTime
        self.stdTargetAccuracy_m = stdTargetAccuracy_m
        self.aspect_deg_dict    = {target: {radar: None for radar in self.RadarList} for target in self.targetList}
        np.random.seed(513)

    def updateScene(self):
        self.t_s = self.t_s + self.updatePeriod_s
        continueSim = True
        if(self.t_s > self.simTime_s):
            continueSim = False
        else:
            for target in self.targetList:
                target.update(self.updatePeriod_s)
                self.updateAspects()

        return continueSim
    
    def obtainRCSmeasurements(self):
        RCSdict = {target: {txRadar: {rxRadar: None for rxRadar in self.RadarList} for txRadar in self.RadarList} for target in self.targetList}
        for target in RCSdict.keys():
            for txRadar in RCSdict[target].keys():
                for rxRadar in RCSdict[target][txRadar].keys():
                    RCSdict[target][txRadar][rxRadar] = self.getRCSmeasurement(target,txRadar,rxRadar)

        return RCSdict

    def getRCSmeasurement(self,target,txRadar,rxRadar):
        if(txRadar == rxRadar):
            aspect = self.computeAspectDeg(target,txRadar)
        else:
            aspect = self.computeAspectDeg(target,txRadar,rxRadar)

        ACcenterList = target.RCSModel.aspectClassCenterList
        ACdist       = np.abs(ACcenterList - aspect)
        ACidx        = np.argmin(ACdist)
        RCS          = target.RCSModel.RCSperturbed[ACidx] + np.random.normal(0,target.RCSModel.stdProcessing)
        return       RCS       
    
    def updateAspects(self):
        for target, radarDict in self.aspect_deg_dict.items():
            for radar in radarDict:
                self.aspect_deg_dict[target][radar] = self.computeAspectDeg(target,radar)

    def computeAspectDeg(self,target,*radar):
        bpv = target.bpv
        targetPos_m = np.array([target.pos.xm,target.pos.ym,target.pos.zm])
        if(len(radar) == 1):
            radarPos_m  = np.array([radar[0].xkm,radar[0].ykm,radar[0].zkm]) * 1e3
            rlos = -(targetPos_m - radarPos_m) 
            rlos = rlos / np.linalg.norm(rlos) #rlos = reverse line of sight vector
        elif len(radar) == 2:
            radarPos_m1  = np.array([radar[0].xkm,radar[0].ykm,radar[0].zkm]) * 1e3
            rlos1 = -(targetPos_m - radarPos_m1) 
            rlos1 = rlos1 / np.linalg.norm(rlos1) 

            radarPos_m2  = np.array([radar[1].xkm,radar[1].ykm,radar[1].zkm]) * 1e3
            rlos2 = -(targetPos_m - radarPos_m2) 
            rlos2 = rlos2 / np.linalg.norm(rlos2) 

            rlos = rlos1 + rlos2
            rlos = rlos / np.linalg.norm(rlos)
        else:
            raise ValueError("Invalid number of RLOS arguments")
        
        cosa = np.dot(bpv,rlos)
        arad = np.arccos(cosa)
        adeg = np.rad2deg(arad)
        return adeg 
    
    def computePitchRollFromOrientation(self,BPV):
        x1 = BPV[0]
        x2 = BPV[1]
        x3 = BPV[2]
        pitch = np.asin(-x3)
        yaw = np.atan(x2/x1)

        yawDeg   = np.rad2deg(yaw)
        pitchDeg = np.rad2deg(pitch)
        if(yawDeg < 0):
            yawDeg+= 360

        return pitchDeg, yawDeg
    

class TargetModelCass:
    def __init__(self, targPos0,targVel,RCSModel):
        self.pos = SimpleNamespace()
        self.pos.xm = targPos0.xkm * 1e3
        self.pos.ym = targPos0.ykm * 1e3
        self.pos.zm = targPos0.zkm * 1e3

        self.vel = targVel
        velvector = np.array([targVel.xms,targVel.yms,targVel.zms])

        self.bpv = velvector / np.linalg.norm(velvector)
        self.RCSModel = RCSModel

    def update(self,updatePeriod_s):
        self.pos.xm = self.pos.xm + self.vel.xms*updatePeriod_s
        self.pos.ym = self.pos.ym + self.vel.yms*updatePeriod_s
        self.pos.zm = self.pos.zm + self.vel.zms*updatePeriod_s

class RCSModelClass:
    def __init__(self,minRcs,maxRcs,stdPertubation,stdProcessing):
        #specifying a seed for repeatability
        np.random.seed(613)
        
        coneRCSmat = loadmat('C:\\Users\\tyler\\OneDrive\\Documents\\College\\Graduate\\Machine Learning\\Project\\Data\\rcsCone.mat')
        coneRCS    = coneRCSmat['rcsConedBsm']

        coneRCS    = np.linspace(0.5,179.5,180)

        #self.RCSunPerturbed = np.random.uniform(minRcs,maxRcs,180)
        self.RCSunPerturbed = np.array(coneRCS)
        self.RCSperturbed = self.RCSunPerturbed \
            + stdPertubation*np.random.randn(*self.RCSunPerturbed.shape)
        
        self.stdProcessing = stdProcessing
        self.stdPertubation = stdPertubation
        self.stdProcessing  = stdProcessing
        self.aspectClassCenterList = np.arange(0.5,179.6,1)
        if(len(self.aspectClassCenterList) != len(self.RCSunPerturbed)):
            raise ValueError("RCS not correctly computed")

