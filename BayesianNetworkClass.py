import numpy as np
from tqdm import tqdm
from scipy.interpolate import RegularGridInterpolator
from scipy.optimize import minimize
from PlottingClass import *
from SceneModelClass import SceneModelClass

class BayesianNetworkClass:
    def __init__(self,noiselessRCS,stdPertubation,stdProcessing,ACcenterList,numMCs):
        np.random.seed(413)
        self.noiselessRCS = noiselessRCS
        self.stdPerturbation = stdPertubation
        self.stdProcessing  = stdProcessing
        self.numMCs         = numMCs
        self.aspectClassCenterList = ACcenterList
        self.orientationClassCentersList = [(pitch,yaw) for pitch in np.arange(-89.5,89.5,1) for yaw in np.arange(0.5,359.5,1)] 
        self.rcsCenterList    = []
        self.trainNetwork()
        

    def trainNetwork(self):
        minUnperturbed = np.min(self.noiselessRCS)
        maxUnperturbed = np.max(self.noiselessRCS)
        stdNoise = self.stdPerturbation + self.stdProcessing
        minVal = minUnperturbed - 3*stdNoise # extend up to 3 sigma
        maxVal = maxUnperturbed + 3*stdNoise
        self.rcsCenterList = np.arange(minVal,maxVal+1,1)

        self.probFgivenACdiscrete = np.zeros((len(self.aspectClassCenterList),len(self.rcsCenterList)))

        for acIdx, acCenter in tqdm(enumerate(self.aspectClassCenterList)):
            for mc in range(self.numMCs):
                unpeturbedRCS = self.noiselessRCS[acIdx]
                noiseRCS = unpeturbedRCS + np.random.normal(0,stdNoise)
                distances = np.abs(self.rcsCenterList - noiseRCS)
                idxRCSNeighbor = np.argmin(distances)
                self.probFgivenACdiscrete[acIdx,idxRCSNeighbor] += 1/self.numMCs

        #TODO - Do proper circular interpolation for edges!!
        aspectStep = self.aspectClassCenterList[1] - self.aspectClassCenterList[0]
        aspectClassCenterListCircular = np.concatenate(([self.aspectClassCenterList[0]-aspectStep],self.aspectClassCenterList,[self.aspectClassCenterList[-1] + aspectStep]))
        probFgivenACdiscreteCircular = np.vstack((self.probFgivenACdiscrete[-1,:],self.probFgivenACdiscrete,self.probFgivenACdiscrete[0,:]))
        self.proFgivenAlinearInterp = RegularGridInterpolator((aspectClassCenterListCircular,self.rcsCenterList),probFgivenACdiscreteCircular)

        # PlottingClass.plotInterpProb(self.aspectClassCenterList,self.rcsCenterList,self.proFgivenAlinearInterp)
    
    def computeOrientationMethod1(self,radarRCSdict,target):
        
        costFunctionList = {xccenter: 0.0 for xccenter in self.orientationClassCentersList}

        #Computer numerator
        for xccenter in self.orientationClassCentersList:
            pr_xccenter = np.array([])
            for txRadar in radarRCSdict.keys():
                for rxRadar in radarRCSdict[txRadar].keys():
                    rcs =  radarRCSdict[txRadar][rxRadar]
                    pitch, yaw = xccenter
                    if(pitch == -21.5 and yaw == 348.5):
                        None

                    bpv = SceneModelClass.computeOrientationVector(pitch,yaw)
                    if(txRadar == rxRadar):
                        aspect = SceneModelClass.computeAspectDegFromBpv(bpv,target,txRadar)
                    else:
                        aspect = SceneModelClass.computeAspectDegFromBpv(bpv,target,txRadar,rxRadar)

                    #Compute F class
                    Fdistances = np.abs(self.rcsCenterList - rcs)
                    idxFNeighbor = np.argmin(Fdistances)

                    #Compute AC Class
                    ACdistances = np.abs(self.aspectClassCenterList - aspect)
                    idxACNeighbor = np.argmin(ACdistances)

                    pr_xccenter = np.append(pr_xccenter,-np.log(self.probFgivenACdiscrete[idxACNeighbor,idxFNeighbor]))
                    #pr_xccenter = np.append(pr_xccenter,-np.log(self.proFgivenA_spline((aspect,rcs)))))

            costFunctionList[xccenter] = np.sum(pr_xccenter)

        likelyXC = min(costFunctionList, key=costFunctionList.get)

        return likelyXC

    def computeOrientationMethod2(self,radarRCSdict,target):
        #First get the RCS vector and pre compute
        XCguessList = self.findProbableXCguesses(radarRCSdict,target)
        minCost = float('inf')
        pitchLikely = float('nan')
        yawLikely   = float('nan')
        print(len(XCguessList))
        for xccenter in XCguessList:
            pitch, yaw = xccenter
            X_initial_guess = np.array([pitch,yaw])
            results = minimize(self.method2Likelihood,X_initial_guess,args=(radarRCSdict,target),method='Powell')
            if(results.fun < minCost):
                minCost = results.fun
                likelyX = results.x
                pitchLikely = SceneModelClass.normalizePitch(likelyX[0])
                yawLikely   = likelyX[1] % 360

        return (pitchLikely,yawLikely)
    
    def findProbableXCguesses(self,radarRCSdict,target):
        minNXguess = 10
        maxNXguess = 100
        # initialThres = 0.04 #good for ideal
        initialThres  = 0.06
        thres = initialThres
        step = 0.01
        dir = 0
        NXguessInRange = False
        while(not NXguessInRange):
            Xguess = []
            for xccenter in self.orientationClassCentersList :
                possibleXC = True
                for txRadar in radarRCSdict.keys():
                    for rxRadar in radarRCSdict[txRadar].keys():
                        if not possibleXC:
                            continue

                        rcs = radarRCSdict[txRadar][rxRadar]
                        pitch, yaw = xccenter
                        bpv = SceneModelClass.computeOrientationVector(pitch,yaw)
                        if(txRadar == rxRadar):
                            aspect = SceneModelClass.computeAspectDegFromBpv(bpv,target,txRadar)
                        else:
                            aspect = SceneModelClass.computeAspectDegFromBpv(bpv,target,txRadar,rxRadar)

                        # #Compute F class
                        # Fdistances = np.abs(self.rcsCenterList - rcs)
                        # idxFNeighbor = np.argmin(Fdistances)
                        # F = self.rcsCenterList[idxFNeighbor]

                        pr = self.proFgivenAlinearInterp((aspect,rcs))
                        if(pr < thres):
                            possibleXC = False
                if possibleXC:
                    Xguess.append(xccenter)
            if(len(Xguess) > maxNXguess):
                #we overshot and let in too many
                if(dir == 1):
                    step = step/2
                    
                thres = thres + step
                dir = -1
            elif(len(Xguess) < minNXguess):
                if(dir == -1):
                    step = step/2
                thres = thres - step
                dir = 1
            else:
                NXguessInRange = True

        return Xguess
        
    
    def method2Likelihood(self, X, radarRCSdict, target):
        Likelihood = 0
        pitch = SceneModelClass.normalizePitch(X[0])
        yaw   = X[1] % 360
        bpv = SceneModelClass.computeOrientationVector(pitch,yaw)
        for txRadar in radarRCSdict.keys():
            for rxRadar in radarRCSdict[txRadar].keys():
                rcs = radarRCSdict[txRadar][rxRadar]
                if txRadar == rxRadar:
                    aspect = SceneModelClass.computeAspectDegFromBpv(bpv, target, txRadar)
                else:
                    aspect = SceneModelClass.computeAspectDegFromBpv(bpv, target, txRadar, rxRadar)
                Likelihood -= np.log(self.proFgivenAlinearInterp((aspect, rcs)))   
                #Likelihood -= np.log(self.testProbFgivenA(aspect,rcs))                 
        return Likelihood


        