from SceneModelClass  import *
from BayesianNetworkClass import *
from types import SimpleNamespace
from collections import namedtuple
import matplotlib.pyplot as plt

Radar = namedtuple('Radar',['xkm','ykm','zkm'])

def namespace_to_tuple(ns):
    return tuple(sorted(ns.__dict__.items()))

def generateRadarList():
    Radar1 = Radar(xkm=-25,ykm=0,zkm=0)
    Radar2 = Radar(xkm=25,ykm=0,zkm=0)
    return [Radar1,Radar2]

def generateTargetList():
    #Target Params
    TargPos0 = SimpleNamespace()
    TargPos0.xkm = -10
    TargPos0.ykm = 5
    TargPos0.zkm = 2

    TargVel = SimpleNamespace()
    TargVel.xms = 250
    TargVel.yms = -50
    TargVel.zms = 100    

    RCSmodel = generateRCSModel()

    target1 = TargetModelCass(TargPos0,TargVel,RCSmodel)
    return [target1]

def generateRCSModel():
    minRCS = -30
    maxRCS = 10
    stdPertubation = 3
    stdProcessing = 1

    #stdPertubation = 0
    #stdProcessing  = 0
    RCSmodel = RCSModelClass(minRCS,maxRCS,stdPertubation,stdProcessing)
    return RCSmodel

def generateSceneModel():
    RadarList = generateRadarList()
    targets    = generateTargetList()
    # Scene Model Params
    updateFreqHz = 4
    updatePeriod_s = 1/updateFreqHz
    simTime_s = 30
    simTime_s = 20
    stdTargetAccuracy_m = 10
    sceneModel = SceneModelClass(RadarList,targets,updatePeriod_s,simTime_s,stdTargetAccuracy_m)
    return sceneModel

def generateBayesianNetwork(sceneModel):
    RCSmodel = sceneModel.targetList[0].RCSModel
    rcsCone = RCSmodel.RCSunPerturbed
    stdPertubation = RCSmodel.stdPertubation
    stdProcessing  = RCSmodel.stdProcessing
    ACcenterList   = RCSmodel.aspectClassCenterList
    numMCs = 10000
    #numMCs = 1

    bayesianNetwork = BayesianNetworkClass(rcsCone,stdPertubation,stdProcessing,ACcenterList,numMCs)
    return bayesianNetwork

def generateOrientationPredictionMethod2(sceneModel,bayesianNetwork):
    RCSdict = sceneModel.obtainRCSmeasurements()
    orientationDict = {target : None for target in sceneModel.targetList}
    
    for target, radarRCSdict in RCSdict.items():
        orientationDict[target] = bayesianNetwork.computeOrientationMethod2(radarRCSdict,target)

    return orientationDict

def main():
    sceneModel = generateSceneModel()
    bayesianNetwork = generateBayesianNetwork(sceneModel)
    #Start Simulation

    aspectDeg = np.array([])
    yawM2Deg       = np.array([])
    pitchM2Deg     = np.array([])
    yawTruthDeg    = np.array([])
    pitchTruthDeg  = np.array([])

    t_s    = np.array([])
    while sceneModel.updateScene():
        print(f"Time is: {sceneModel.t_s}")
        t_s = np.append(t_s,sceneModel.t_s)
    
        bpvTarget1 = next(iter(sceneModel.targetList)).bpv
        pitchTruth, yawTruth = sceneModel.computePitchRollFromOrientation(bpvTarget1)

        orientationDict = generateOrientationPredictionMethod2(sceneModel,bayesianNetwork)
        _, orientationM2 = next(iter(orientationDict.items()))
        pitchM2, yawM2    = orientationM2

        yawTruthDeg   = np.append(yawTruthDeg,yawTruth)
        pitchTruthDeg = np.append(pitchTruthDeg,pitchTruth)

        yawM2Deg   = np.append(yawM2Deg,yawM2)
        pitchM2Deg = np.append(pitchM2Deg,pitchM2)
        

        _, radarDict = next(iter(sceneModel.aspect_deg_dict.items())) 
        _, newAspectDeg = next(iter(radarDict.items()))
        aspectDeg = np.append(aspectDeg,newAspectDeg)

    plt.figure(1)
    plt.plot(t_s,aspectDeg,'r',linewidth=1)
    plt.show(block=False)

    plt.figure(2)
    plt.plot(t_s,yawTruthDeg,label='true yaw')
    plt.scatter(t_s,yawM2Deg,label='measured yaw')
    plt.xlabel('times (s)')
    plt.ylabel('yaw (deg)')
    plt.legend()
    plt.show(block=False)

    plt.figure(3)
    plt.plot(t_s,pitchTruthDeg,label='true pitch')
    plt.scatter(t_s,pitchM2Deg,label='measured pitch')
    plt.xlabel('times (s)')
    plt.ylabel('pitch (deg)')
    plt.legend()
    plt.show(block=True)


if __name__ == "__main__":
    main()