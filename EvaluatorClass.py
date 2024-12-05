import numpy as np
from SceneModelClass  import *

class EvaluatorClass:
    def __init__(self):
        self.angularDistVec = []
        self.degenAngularDistVec = []
        self.idxNonDegen = np.zeros(0,dtype=bool)

    def computeAngularError(self, classBpv, bpvTruth, bpvDegen):
        distTruth = SceneModelClass.computeAngularDistance(classBpv,bpvTruth)

        degenClassification = False
        minDist = distTruth
        
        for i in range(len(bpvDegen)):
            distDegen = SceneModelClass.computeAngularDistance(classBpv,bpvDegen[i])
            if(distDegen < minDist):
                degenClassification = True
                minDist = distDegen

        if degenClassification:
            self.degenAngularDistVec = np.append(self.degenAngularDistVec,minDist)
            self.idxNonDegen = np.append(self.idxNonDegen,False)
        else:
            self.angularDistVec = np.append(self.angularDistVec,minDist)
            self.idxNonDegen = np.append(self.idxNonDegen,True)


