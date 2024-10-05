import numpy as np
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor, as_completed

class BayesianNetworkClass:
    def __init__(self,rcsCone,stdPertubation,stdProcessing,ACcenterList,numMCs):
        np.random.seed(413)
        self.rcsCone = rcsCone
        self.stdPerubtation = stdPertubation
        self.stdProcessing  = stdProcessing
        self.numMCs         = numMCs
        self.aspectClassCenterList = ACcenterList
        self.orientationClassCentersList = [(pitch,yaw) for pitch in np.arange(-89.5,89.5,1) for yaw in np.arange(0.5,359.5,1)] 
        self.rcsCenterList    = []
        self.trainNetwork()
        

    def trainNetwork(self):
        minUnperturbed = np.min(self.rcsCone)
        maxUnperturbed = np.max(self.rcsCone)
        stdNoise = self.stdPerubtation + self.stdProcessing
        minVal = minUnperturbed - 3*stdNoise # extend up to 3 sigma
        maxVal = maxUnperturbed + 3*stdNoise
        self.rcsCenterList = np.arange(minVal,maxVal+1,1)


        self.probFgivenAC = {accenter : {RCS : 0.0 for RCS in self.rcsCenterList} for accenter in self.aspectClassCenterList}

        for acIdx, acCenter in tqdm(enumerate(self.aspectClassCenterList)):
            for mc in range(self.numMCs):
                unpeturbedRCS = self.rcsCone[acIdx]
                noiseRCS = unpeturbedRCS + np.random.normal(0,stdNoise)
                distances = np.abs(self.rcsCenterList - noiseRCS)
                idxNeighbor = np.argmin(distances)
                rcsKey = self.rcsCenterList[idxNeighbor]
                self.probFgivenAC[acCenter][rcsKey] += 1/self.numMCs

    
    def computeOrientationMethod2(self,radarRCSdict,target):
        
        probXCgivenFlist = {xccenter: 0.0 for xccenter in self.orientationClassCentersList}

        #Computer numerator
        for xccenter in self.orientationClassCentersList:
            pr_xccenter = np.array([])
            for txRadar in radarRCSdict.keys():
                for rxRadar in radarRCSdict[txRadar].keys():
                    rcs =  radarRCSdict[txRadar][rxRadar]
                    pitch, yaw = xccenter
                    if(pitch == -21.5 and yaw == 348.5):
                        None

                    bpv = self.computeOrientationVector(pitch,yaw)
                    if(txRadar == rxRadar):
                        aspect = self.computeAspect(bpv,target,txRadar)
                    else:
                        aspect = self.computeAspect(bpv,target,txRadar,rxRadar)

                    #Compute F class
                    Fdistances = np.abs(self.rcsCenterList - rcs)
                    idxFNeighbor = np.argmin(Fdistances)
                    F = self.rcsCenterList[idxFNeighbor]

                    #Compute AC Class
                    ACdistances = np.abs(self.aspectClassCenterList - aspect)
                    idxACNeighbor = np.argmin(ACdistances)
                    AC = self.aspectClassCenterList[idxACNeighbor]
                   
                    if(self.probFgivenAC[AC][F]>0):
                        None

                    pr_xccenter = np.append(pr_xccenter,self.probFgivenAC[AC][F])

            pr_xccenter = np.append(pr_xccenter, 1/len(self.orientationClassCentersList))
            probXCgivenFlist[xccenter] = np.prod(pr_xccenter)

        #Now we need to sum together all the elements in the dictionary to normalize
        summand = 0.0
        for unNoramlizedProb in probXCgivenFlist.values():
            summand += unNoramlizedProb

        for xccenter in probXCgivenFlist.keys():
            probXCgivenFlist[xccenter] /= summand

        likelyXC = max(probXCgivenFlist, key=probXCgivenFlist.get)

        numMax = 0
        maxProb = 0
        for xccenter in probXCgivenFlist.keys():
            if(maxProb < probXCgivenFlist[xccenter]):
                maxProb = probXCgivenFlist[xccenter]
                numMax = 1
            elif(maxProb == probXCgivenFlist[xccenter]):
                numMax += 1
                
        print(maxProb)
        print(numMax)

        return likelyXC
    
    def computeOrientationMethod2parallel(self,radarRCSdict,target):
        
        probXCgivenFlist = {xccenter: 0.0 for xccenter in self.orientationClassCentersList}

        with ThreadPoolExecutor() as executor:
            futures = [executor.submit(self.compute_for_xccenter,xccenter,radarRCSdict,target)
                       for xccenter in self.orientationClassCentersList]
            
            for future in as_completed(futures):
                xccenter, prob = future.result()
                probXCgivenFlist[xccenter] = prob

        likelyXC = max(probXCgivenFlist, key=probXCgivenFlist.get)

        # numMax = 0
        # maxProb = 0
        # for xccenter in probXCgivenFlist.keys():
        #     if(maxProb < probXCgivenFlist[xccenter]):
        #         maxProb = probXCgivenFlist[xccenter]
        #         numMax = 1
        #     elif(maxProb == probXCgivenFlist[xccenter]):
        #         numMax += 1
                
        # print(maxProb)
        # print(numMax)

        return likelyXC
    
    @staticmethod
    def compute_for_xccenter(self,xccenter, radarRCSdict, target):
        pr_xccenter = np.array([])
        for txRadar in radarRCSdict.keys():
            for rxRadar in radarRCSdict[txRadar].keys():
                rcs =  radarRCSdict[txRadar][rxRadar]
                pitch, yaw = xccenter
                if(pitch == -21.5 and yaw == 348.5):
                    None

                bpv = self.computeOrientationVector(pitch,yaw)
                if(txRadar == rxRadar):
                    aspect = self.computeAspect(bpv,target,txRadar)
                else:
                    aspect = self.computeAspect(bpv,target,txRadar,rxRadar)

                #Compute F class
                Fdistances = np.abs(self.rcsCenterList - rcs)
                idxFNeighbor = np.argmin(Fdistances)
                F = self.rcsCenterList[idxFNeighbor]

                #Compute AC Class
                ACdistances = np.abs(self.aspectClassCenterList - aspect)
                idxACNeighbor = np.argmin(ACdistances)
                AC = self.aspectClassCenterList[idxACNeighbor]
                
                if(self.probFgivenAC[AC][F]>0):
                    None

                pr_xccenter = np.append(pr_xccenter,self.probFgivenAC[AC][F])

        pr_xccenter = np.append(pr_xccenter, 1/len(self.orientationClassCentersList))
        return xccenter, np.prod(pr_xccenter)

    def computeOrientationVector(self,pitch,yaw):
        pitchRad = np.deg2rad(pitch)
        yawRad   = np.deg2rad(yaw)
        return np.array([
            np.cos(pitchRad)*np.cos(yawRad),
            np.cos(pitchRad)*np.sin(yawRad),
            -np.sin(pitchRad)
        ])
    
    def computeAspect(self,bpvEstimate,target,*radar):
        #TODO Add noise to target position
        bpvEstimate
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
        
        cosa = np.dot(bpvEstimate,rlos)
        arad = np.arccos(cosa)
        adeg = np.rad2deg(arad)
        return adeg 

    # def computeAspect(self,XC,RLOS):
    #     cosa = np.dot(XC,RLOS)
    #     arad = np.acos(cosa)
    #     AC   = np.rad2deg(arad)
    #     return AC
    
    # def computeHalfAspect(self,XC,RLOS1,RLOS2):
    #     RH = RLOS1 + RLOS2
    #     RH = RH / np.linalg.norm(RH)
    #     AC_H = self.computeAspect(XC,RH) 
    #     return AC_H  

    # def trainXCNewtorkGivenRLOS(self,*RLOS):
    #     probFgivenXC = {xccenter : {RCS: 0.0 for RCS in self.rcsCenterList} for xccenter in self.orientationClassCentersList}      

    #     for center in self.orientationClassCentersList:
    #         pitch, yaw = center
    #         XC = self.computeOrientationVector(pitch,yaw)
    #         if(len(RLOS) == 1):
    #             AC = self.computeAspect(XC,RLOS[0])
    #         elif len(RLOS) == 2:
    #             AC = self.computeHalfAspect(XC,RLOS[0],RLOS[1])
    #         else:
    #             raise ValueError("Invalid number of RLOS arguments")

    #         ACdistances = np.abs(AC - self.aspectClassCenterList)
    #         idxACNeighbor = np.argmin(ACdistances)
    #         ACcenter = self.aspectClassCenterList[idxACNeighbor]
    #         for rcsKey in self.rcsCenterList:
    #             probFgivenXC[center][rcsKey] += self.probFgivenAC[ACcenter][rcsKey]

    #     return probFgivenXC     


        