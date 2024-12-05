import matplotlib.pyplot as plt
import numpy as np
from SceneModelClass  import *

class PlottingClass:
    @staticmethod
    def plotInterpProb(aspectClassCenterList, rcsCenterList, proFgivenAlinearInterp):
        # Define the grid to evaluate the interpolator
        minAC = min(aspectClassCenterList)
        maxAC = max(aspectClassCenterList)
        minF  = min(rcsCenterList)
        maxF  = max(rcsCenterList)

        Astep  = 0.05
        RCSstep = 0.05

        Alist   = np.arange(minAC,maxAC,Astep)
        RCSlist = np.arange(minF,maxF,RCSstep)

        aspect_grid, rcs_grid = np.meshgrid(Alist, RCSlist)
        
        # Evaluate the interpolator over the grid
        interp_values = proFgivenAlinearInterp((aspect_grid, rcs_grid))

        # Plot the heatmap
        plt.figure(figsize=(8, 6))
        plt.contourf(aspect_grid, rcs_grid, interp_values, cmap='jet', levels=50)
        plt.colorbar(label='Interpolated Probability Value')
        plt.xlabel('Aspect Class Center')
        plt.ylabel('RCS Center')
        plt.title('Heat Map of Interpolator')
        plt.show(block=True)

    @staticmethod
    def plotAspect(t_s,aspectVec):
        plt.figure()
        plt.plot(t_s,aspectVec,'r',linewidth=1)
        plt.show(block=False)

    @staticmethod
    def plotOrientationVsTime(t_s,yawTruthVec,pitchTruthVec,yawClassVec,pitchClassVec, yawDegenVec, pitchDegenVec):
        plt.figure()
        plt.plot(t_s,yawTruthVec,label='true yaw')
        if(len(yawDegenVec != 0)):
            plt.plot(t_s,yawDegenVec,label='degenerate yaw')
        plt.scatter(t_s,yawClassVec,label='measured yaw')
        plt.xlabel('times (s)')
        plt.ylabel('yaw (deg)')
        plt.legend()
        plt.show(block=False)

        plt.figure()
        plt.plot(t_s,pitchTruthVec,label='true pitch')
        if(len(pitchDegenVec != 0)):
            plt.plot(t_s,pitchDegenVec,label='degenerate pitch')
        plt.scatter(t_s,pitchClassVec,label='measured pitch')
        plt.xlabel('times (s)')
        plt.ylabel('pitch (deg)')
        plt.legend()
        plt.show(block=False)

    @staticmethod
    def plotOrientationVsTime3(t_s,bpvTruth,bpvClass,bpvDegenStruct,idxNonDegen):
        pitch_yaws_truth = list(map(lambda bpv: SceneModelClass.computePitchRollFromOrientation(bpv), bpvTruth))
        pTruth, yTruth   = zip(*pitch_yaws_truth)

        pitch_yaws_class = list(map(lambda bpv: SceneModelClass.computePitchRollFromOrientation(bpv), bpvClass))
        pClass, yClass  = zip(*pitch_yaws_class)
        pClass          = np.array(pClass)
        yClass          = np.array(yClass)

        degen_t_p_y = []
        for i in range(len(bpvDegenStruct)):
            degen_t_p_y.append(list(map(lambda pair: (pair[0], *SceneModelClass.computePitchRollFromOrientation(pair[1])),bpvDegenStruct[i])))

        # plt.figure(figsize=(6, 6))
        plt.figure(figsize=(5, 5.3))
        plt.plot(t_s,yTruth,label='true yaw')

        for i in range(len(degen_t_p_y)):
            tdegen,_,ydegen = zip(*degen_t_p_y[i])
            if i == 0:
                plt.plot(tdegen,ydegen,color='red',label='degenerate yaws')
            else:
                plt.plot(tdegen,ydegen,color='red')

        plt.scatter(t_s[idxNonDegen],yClass[idxNonDegen],color='blue',label='measured yaw - correct classification')
        plt.scatter(t_s[~idxNonDegen],yClass[~idxNonDegen],color='red',label='measured yaw - degenerate classification')
        plt.xlabel('times (s)')
        plt.ylabel('yaw (deg)')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.11))
        plt.tight_layout(rect=[0, 0, 1, 1])
        plt.show(block=False)

        plt.figure(figsize=(5, 5.3))
        plt.plot(t_s,pTruth,label='true pitch')
        for i in range(len(degen_t_p_y)):
            tdegen,pdegen,_ = zip(*degen_t_p_y[i])
            if i == 0:
                plt.plot(tdegen,pdegen,color='green',label='degenerate pitches')
            else:
                plt.plot(tdegen,pdegen,color='green')

        plt.scatter(t_s[idxNonDegen],pClass[idxNonDegen],color='blue',label='measured pitch - correct classification')
        plt.scatter(t_s[~idxNonDegen],pClass[~idxNonDegen],color='red',label='measured pitch - degenerate classification')
        plt.xlabel('times (s)')
        plt.ylabel('pitch (deg)')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.11))
        plt.tight_layout(rect=[0, 0, 1, 1])
        plt.show(block=True)


    @staticmethod
    def plotOrientationVsTime2(t_s,yawTruthVec,pitchTruthVec,yawClassVec,pitchClassVec, yawDegenVec, pitchDegenVec, yawDegenVec2, pitchDegenVec2):
        yvec = []
        pvec = []
        yvec2 = []
        pvec2 = []

        max_degens = max(len(sublist) for sublist in yawDegenVec)
        for _ in range(max_degens):
            yvec.append({'yaw':[],'time':[]})
            pvec.append({'pitch':[],'time':[]})

        max_degens2 = max(len(sublist) for sublist in yawDegenVec2)
        for _ in range(max_degens2):
            yvec2.append({'yaw':[],'time':[]})
            pvec2.append({'pitch':[],'time':[]})

        for tidx in range(len(yawDegenVec)):
            t = t_s[tidx]
            for j in range(len(yawDegenVec[tidx])):
                yvec[j]['yaw'].append(yawDegenVec[tidx][j])
                yvec[j]['time'].append(t)
                pvec[j]['pitch'].append(pitchDegenVec[tidx][j])
                pvec[j]['time'].append(t)


        for tidx in range(len(yawDegenVec2)):
            t = t_s[tidx]
            for j in range(len(yawDegenVec2[tidx])):
                yvec2[j]['yaw'].append(yawDegenVec2[tidx][j])
                yvec2[j]['time'].append(t)
                pvec2[j]['pitch'].append(pitchDegenVec2[tidx][j])
                pvec2[j]['time'].append(t)

        plt.figure()
        plt.plot(t_s,yawTruthVec,label='true yaw')

        for i in range(len(yvec)):
            entry = yvec[i]
            if i == 0:
                plt.scatter(entry['time'],entry['yaw'],color='red',label='degenerate yaws') 
            else:
                plt.scatter(entry['time'],entry['yaw'],color='red') 

        for i in range(len(yvec2)):
            entry = yvec2[i]
            if i == 0:
                plt.plot(entry['time'],entry['yaw'],color='green',label='degenerate yaws 2') 
            else:
                plt.plot(entry['time'],entry['yaw'],color='green')
                    

        plt.scatter(t_s,yawClassVec,label='measured yaw')
        plt.xlabel('times (s)')
        plt.ylabel('yaw (deg)')
        plt.legend()
        plt.show(block=False)

        plt.figure()
        plt.plot(t_s,pitchTruthVec,label='true pitch')

        for i in range(len(pvec)):
            entry = pvec[i]
            if i == 0:
                plt.scatter(entry['time'],entry['pitch'],color='red',label='degenerate pitches') 
            else:
                plt.scatter(entry['time'],entry['pitch'],color='red')     

        for i in range(len(pvec2)):
            entry = pvec2[i]
            if i == 0:
                plt.plot(entry['time'],entry['pitch'],color='green',label='degenerate pitches 2') 
            else:
                plt.plot(entry['time'],entry['pitch'],color='green') 

        plt.scatter(t_s,pitchClassVec,label='measured pitch')
        plt.xlabel('times (s)')
        plt.ylabel('pitch (deg)')
        plt.legend()
        plt.show(block=True)

    @staticmethod
    def plotAngularDistanceHistogram(angularDistVec,degenAngularDistvec):
        plt.figure()
        plt.hist(angularDistVec,bins=10, edgecolor='black', color='blue')
        plt.xlabel('Angular Distance from True Orientation (degrees)')
        plt.ylabel('Frequency')
        plt.title("Histogram of angular distance from Truth")
        plt.show(block=True)

        meanADV = np.mean(angularDistVec)
        stdADV  = np.std(angularDistVec)
        print(f"The mean of the ADV is {meanADV} with std {stdADV}")

        if(len(degenAngularDistvec) > 0):
            Ntotal = len(angularDistVec) + len(degenAngularDistvec)
            fracNonDegen = len(angularDistVec)/Ntotal
            fracDegen = len(degenAngularDistvec)/Ntotal

            print(f"The frac non degen is {fracNonDegen} and the frac degen is {fracDegen}")

            meanDegen = np.mean(degenAngularDistvec)
            stdDegen   = np.std(degenAngularDistvec)

            print(f"The mean of the degen is {meanDegen} with std {stdDegen}")

            plt.figure()
            plt.hist(degenAngularDistvec,bins=10, edgecolor='black', color='red')
            plt.xlabel('Angular Distance from Degenerate Orientation (degrees)')
            plt.ylabel('Frequency')
            plt.title("Histogram of angular distance from Degenerate Orientation")
            plt.show(block=True)
            None
