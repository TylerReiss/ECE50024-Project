from SceneModelClass  import *
from BayesianNetworkClass import *
from EvaluatorClass import EvaluatorClass
from types import SimpleNamespace
from collections import namedtuple

Radar = namedtuple('Radar',['xkm','ykm','zkm'])

def namespace_to_tuple(ns):
    return tuple(sorted(ns.__dict__.items()))

# Generate radar sensor array
def generateRadarList():

    numRadars = 2

    Radar1 = Radar(xkm=-25,ykm=0,zkm=0)
    Radar2 = Radar(xkm=25,ykm=0,zkm=0)
    Radar3 = Radar(xkm=50,ykm=10,zkm=0)
    Radar4 = Radar(xkm=-50,ykm=10,zkm=0)

    if numRadars == 2:
        return [Radar1,Radar2]
    elif numRadars == 3:
        return [Radar1,Radar2,Radar3]
    elif numRadars == 4:
        return [Radar1,Radar2,Radar3, Radar4]
    else:
        raise ValueError("Unacceptable number of radars")

# Genererate targets 
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

#Generate RCS model
def generateRCSModel():

    lowNoise = False

    if lowNoise:
        stdPertubation = 1.5
        stdProcessing  = 0.5
    else:
        stdPertubation = 3
        stdProcessing = 1

    useIdealDistribution = False

    RCSmodel = RCSModelClass(stdPertubation,stdProcessing,useIdealDistribution)
    return RCSmodel

#Generate Scene Model
def generateSceneModel():
    RadarList = generateRadarList()
    targets    = generateTargetList()
    # Scene Model Params
    updateFreqHz = 4
    updatePeriod_s = 1/updateFreqHz
    simTime_s = 20
    # simTime_s = 2
    stdTargetAccuracy_m = 10
    sceneModel = SceneModelClass(RadarList,targets,updatePeriod_s,simTime_s,stdTargetAccuracy_m)
    return sceneModel

#Generate Bayesian Network for orientation classification
def generateBayesianNetwork(sceneModel):
    RCSmodel = sceneModel.targetList[0].RCSModel
    rcsUnperturbed = RCSmodel.RCSunPerturbed
    stdPertubation = RCSmodel.stdPertubation
    stdProcessing  = RCSmodel.stdProcessing
    ACcenterList   = RCSmodel.aspectClassCenterList

    numMCs = 10000

    if(RCSmodel.stdProcessing + RCSmodel.stdPertubation == 0):
        numMCs = 1

    bayesianNetwork = BayesianNetworkClass(rcsUnperturbed,stdPertubation,stdProcessing,ACcenterList,numMCs)
    return bayesianNetwork

# Generate an orientation classification from bayesian network
def generateOrientationClassification(sceneModel,bayesianNetwork):

    classificationMethod = 2

    RCSdict = sceneModel.obtainRCSmeasurements()
    orientationDict = {target : None for target in sceneModel.targetList}
    
    for target, radarRCSdict in RCSdict.items():
        if classificationMethod == 1:
            orientationDict[target] = bayesianNetwork.computeOrientationMethod1(radarRCSdict,target)
        elif classificationMethod == 2:
            orientationDict[target] = bayesianNetwork.computeOrientationMethod2(radarRCSdict,target)
        else:
            raise ValueError("Unknown classification method")

    return orientationDict

def storeDegeneratePitchandYaws(degenBPVStruct, degenBPVs,t):
    tthres_s = 0.5
    athres_deg = 2

    if len(degenBPVStruct) == 0:
        for i in range(len(degenBPVs)):
            degenBPVclass  = []
            degenBPVclass.append((t,degenBPVs[i])) 
            degenBPVStruct.append(degenBPVclass)
    
    else:
        #Check whether current yaw and pitch fall within the threshold
        for i in range(len(degenBPVs)):
            
            foundDegenClass = False
            classNum = -1
            for j in range(len(degenBPVStruct)):
                tclass,bpvDegenClass = degenBPVStruct[j][-1]
                adist = SceneModelClass.computeAngularDistance(degenBPVs[i],bpvDegenClass)
                tdist = t - tclass
                if(tdist < tthres_s and tdist != 0 and adist < athres_deg):
                    classNum = j
                    foundDegenClass = True
                    break
            
            if(foundDegenClass):
                degenBPVStruct[classNum].append((t,degenBPVs[i]))
            else:
                degenBPVclass  = []
                degenBPVclass.append((t,degenBPVs[i])) 
                degenBPVStruct.append(degenBPVclass)

    return degenBPVStruct




def main():
    sceneModel = generateSceneModel()
    bayesianNetwork = generateBayesianNetwork(sceneModel)
    #Start Simulation

    #Initialize variables
    aspectVec = np.array([])
    yawClassVec       = np.array([])
    pitchClassVec     = np.array([])
    yawTruthVec    = np.array([])
    pitchTruthVec  = np.array([])

    bpvTruth = []
    bpvClassified = []
    degenBPVstruct = []

    t_s    = np.array([])

    evaluator = EvaluatorClass()
    

    while sceneModel.updateScene():
        #Record new times
        print(f"Time is: {sceneModel.t_s}")
        t_s = np.append(t_s,sceneModel.t_s)
    
        #Record true body poniting vector
        bpvTarget1 = next(iter(sceneModel.targetList)).bpv
        bpvTruth.append(bpvTarget1)

        #DELETE THIS
        pitchTruth, yawTruth = SceneModelClass.computePitchRollFromOrientation(bpvTarget1)
        yawTruthVec   = np.append(yawTruthVec,yawTruth)
        pitchTruthVec = np.append(pitchTruthVec,pitchTruth)

        #Compute body pointing vector via probalistic methods
        orientationDict = generateOrientationClassification(sceneModel,bayesianNetwork)
        _, classifiedOrientation = next(iter(orientationDict.items()))
        classifiedPitch, classifiedYaw    = classifiedOrientation
        classifiedBPV = SceneModelClass.computeOrientationVector(classifiedPitch,classifiedYaw)
        bpvClassified.append(classifiedBPV)

        #Record degenerate body pointing vectors and compute bpv distance from degeneracies
        _, bpvDegenerate = next(iter(sceneModel.degenerateBPVs.items()))

        evaluator.computeAngularError(classifiedBPV,bpvTarget1,bpvDegenerate)
        degenBPVstruct = storeDegeneratePitchandYaws(degenBPVstruct, bpvDegenerate,sceneModel.t_s)

        yawClassVec   = np.append(yawClassVec,classifiedYaw)
        pitchClassVec = np.append(pitchClassVec,classifiedPitch)
        
        _, radarDict = next(iter(sceneModel.aspect_deg_dict.items())) 
        _, newAspectDeg = next(iter(radarDict.items()))
        aspectVec = np.append(aspectVec,newAspectDeg)


    PlottingClass.plotAspect(t_s,aspectVec)
    # PlottingClass.plotOrientationVsTime(t_s,yawTruthVec,pitchTruthVec,yawClassVec,pitchClassVec,yawDegenVec,pitchDegenVec)
    PlottingClass.plotOrientationVsTime3(t_s,bpvTruth,bpvClassified,degenBPVstruct,evaluator.idxNonDegen)
    # PlottingClass.plotOrientationVsTime2(t_s,yawTruthVec,pitchTruthVec,yawClassVec,pitchClassVec,yawDegenVec2,pitchDegenVec2,yawDegenVec3,pitchDegenVec3)
    PlottingClass.plotAngularDistanceHistogram(evaluator.angularDistVec,evaluator.degenAngularDistVec)


if __name__ == "__main__":
    main()