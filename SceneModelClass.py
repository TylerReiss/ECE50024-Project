# SceneModelClass.py
# Class that models the scene

from types import SimpleNamespace
from scipy.io import loadmat
from scipy.linalg import null_space
from scipy.interpolate import splrep, splev
from scipy.optimize    import fsolve
import numpy as np
from itertools import product
from itertools import combinations


class SceneModelClass:
    def __init__(self,radarList,targetList,updatePeriod,simTime,stdTargetAccuracy_m):
        self.RadarList      = radarList
        self.targetList     = targetList
        self.t_s            = 0
        self.updatePeriod_s = updatePeriod
        self.simTime_s      = simTime
        self.stdTargetAccuracy_m = stdTargetAccuracy_m
        self.aspect_deg_dict    = {target: {radar: None for radar in self.RadarList} for target in self.targetList}
        self.degenerateBPVs   = {target: None for target in self.targetList}
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
                self.updateDegenerateBPVs()

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
            aspect = SceneModelClass.computeAspectDegFromBpv(target.bpv,target,txRadar)
        else:
            aspect = SceneModelClass.computeAspectDegFromBpv(target.bpv,target,txRadar,rxRadar)

        ACcenterList = target.RCSModel.aspectClassCenterList
        ACdist       = np.abs(ACcenterList - aspect)
        ACidx        = np.argmin(ACdist)
        RCS          = target.RCSModel.RCSperturbed[ACidx] + np.random.normal(0,target.RCSModel.stdProcessing)
        # RCS          = aspect
        return       RCS   

    def updateAspects(self):
        for target, radarDict in self.aspect_deg_dict.items():
            for radar in radarDict:
                self.aspect_deg_dict[target][radar] = SceneModelClass.computeAspectDegFromBpv(target.bpv,target,radar)  

    def updateDegenerateBPVs(self):
        for target, radarDict in self.aspect_deg_dict.items():
            RLOS = np.zeros((0,3))
            
            for radar in radarDict:
                rlos = SceneModelClass.computeRLOS(target,radar)
                RLOS = np.vstack((RLOS,rlos))       

            if(len(radarDict) == 2):
                X0 = target.bpv
                nullVec = null_space(RLOS)
                nullVec = nullVec[:,0]
                c = -2 * np.dot(X0,nullVec) / np.dot(nullVec,nullVec)
                self.degenerateBPVs[target] = np.array([X0 +  c * nullVec])

            elif(len(radarDict) > 2):
                radarkeys = list(radarDict.keys())
                unique_comb = combinations(radarkeys,2)
                X0 = target.bpv
                X2 = np.zeros((0,3))
                for radar1, radar2 in unique_comb:
                    rlos1 = SceneModelClass.computeRLOS(target,radar1)
                    rlos2 = SceneModelClass.computeRLOS(target,radar2)
                    subRLOS = np.vstack((rlos1,rlos2))
                    nullVec = null_space(subRLOS)
                    nullVec = nullVec[:,0]
                    c = -2 * np.dot(X0,nullVec) / np.dot(nullVec,nullVec)
                    X = np.array([X0 +  c * nullVec])
                    X2 = np.vstack((X2,X))

                self.degenerateBPVs[target] = X2

                # ***** All of this code was to analyze possible degenerate aspects for non ideal targets. However I did not see any trends in following these degeneracies 
                # Atrue  = np.zeros((0,1))
                # RCS    = np.zeros((0,1))

                # possibleAspectsDict   = {radar: None for radar in radarDict}
                # AClist = target.RCSModel.aspectClassCenterList
                # ACmin = AClist[0]
                # ACmax = AClist[-1]
                # rcsTrueSpline = splrep(AClist,target.RCSModel.RCSunPerturbed)

                # for radar in radarDict:
                #     aspect = self.aspect_deg_dict[target][radar]
                #     Atrue    = np.vstack((Atrue,aspect)) 

                #     rcs = splev(aspect,rcsTrueSpline)
                #     RCS = np.vstack((RCS,rcs)) 

                #     posAspList = self.findAspectGivenTrueRCS(rcsTrueSpline,AClist,rcs) 
                #     idx = (posAspList >= ACmin) & (posAspList <= ACmax)
                #     posAspList = posAspList[idx]
                #     possibleAspectsDict[radar] = posAspList  


                # posAspListAll = list(possibleAspectsDict.values())
                # posAspComb = product(*posAspListAll)
                # X = []
                # for combination in posAspComb:
                #     A = np.array([np.cos(np.deg2rad(asp)) for asp in combination])
                #     bpv,_,_,_ = np.linalg.lstsq(RLOS,A,rcond=None)
                #     X.append(bpv)
                # X = np.array(X)

                # pitch = []
                # for i in range(X.shape[0]):
                #     p,_ = SceneModelClass.computePitchRollFromOrientation(X[i,:])
                #     pitch.append(p)

                # pitch = np.array(pitch)
                # idxPossible = ~np.isnan(pitch)
                # pitch = pitch[idxPossible]
                # X = X[idxPossible,:]
                # Xtrue = target.bpv
                # idxNotTrue = np.linalg.norm(np.abs(X-Xtrue),axis=1) > 1e-3
                # X = X[idxNotTrue,:]

                # self.degenerateBPVs[target] = X
            
    def findAspectGivenTrueRCS(self,rcsTrueSpline,AClist,rcs):
        #function such that f(a) = rcs (true distribution - rcs)
        def f(a):
            return splev(a,rcsTrueSpline) - rcs

        tol1 = 1e-3
        tol2 = 1e-4
        roots = []
        for AC in AClist:
            try:
                root = fsolve(f, AC)[0]
                if np.isnan(root):
                    continue
                if np.abs(f(root)) > tol1: #We want to make sure we actually get 0s, not just local minimums
                    continue
                if len(roots) == 0 or np.all(np.abs(np.array(roots) - root) > tol2):
                    roots.append(root)
            except RuntimeError:
                continue

        return np.array(roots)


    @staticmethod
    def computePitchRollFromOrientation(BPV):
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
    
    @staticmethod
    def computeOrientationVector(pitch,yaw):
        pitchRad = np.deg2rad(pitch)
        yawRad   = np.deg2rad(yaw)
        return np.array([
            np.cos(pitchRad)*np.cos(yawRad),
            np.cos(pitchRad)*np.sin(yawRad),
            -np.sin(pitchRad)
        ])
    
    @staticmethod
    def computeAspectDegFromBpv(bpv, target,*radar):
        rlos = SceneModelClass.computeRLOS(target,*radar)
        
        cosa = np.dot(bpv,rlos)
        arad = np.arccos(cosa)
        adeg = np.rad2deg(arad)
        return adeg

    @staticmethod
    def computeRLOS(target,*radar):
        #TODO Add noise to target position
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
        return rlos
    
    @staticmethod
    def normalizePitch(pitch):
        pitch = pitch % 360
        if(pitch > 90 and pitch < 270):
            pitch = 180 - pitch
        elif(pitch >= 270):
            pitch -= 360
        return pitch
    
    @staticmethod
    def computeAngularDistance(bpv1,bpv2):
        distDeg = np.rad2deg(np.arccos(np.dot(bpv1,bpv2)))
        return distDeg
    

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
    def __init__(self,stdPertubation,stdProcessing,useIdealDistribution):
        #specifying a seed for repeatability
        np.random.seed(613)
        
        if useIdealDistribution:
            RCS    = np.linspace(0.5,179.5,180)
        else:
            RCSmat = loadmat('C:\\Users\\tyler\\OneDrive\\Documents\\College\\Graduate\\Machine Learning\\Project\\Data\\rcsCone.mat')
            RCS    = RCSmat['rcsConedBsm']

        #This is some old code to design a truly random RCS
        #self.RCSunPerturbed = np.random.uniform(minRcs,maxRcs,180)

        self.RCSunPerturbed = np.array(RCS)
        self.RCSperturbed = self.RCSunPerturbed \
            + stdPertubation*np.random.randn(*self.RCSunPerturbed.shape)
        
        self.stdProcessing = stdProcessing
        self.stdPertubation = stdPertubation
        self.aspectClassCenterList = np.arange(0.5,179.6,1)
        if(len(self.aspectClassCenterList) != len(self.RCSunPerturbed)):
            raise ValueError("RCS not correctly computed")
        
def find_nullspace_dist_for_degen_vector(X0, nullVec):
    a = np.dot(nullVec,nullVec)
    b = 2*np.dot(X0,nullVec)
    c = np.dot(X0,X0) - 1
    coeffs = [a, b, c]
    dist_values = np.roots(coeffs)
    return dist_values

