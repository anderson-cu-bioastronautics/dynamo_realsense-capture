import os 
import numpy as np 
import ezc3d
import scipy
from scipy.spatial import cKDTree
from scipy.optimize import leastsq, fmin, least_squares
from scipy.linalg import lstsq
from gias2.common import transform3D

def group_consecutives(vals, step=1):
    """Return list of consecutive lists of numbers from vals (number list)."""
    run = []
    result = [run]
    expect = None
    for v in vals:
        if (v == expect) or (expect is None):
            run.append(v)
        else:
            run = [v]
            result.append(run)
        expect = v + step
    return result #what is this?

folderPath = r'D:\9.26\SB\walk1.75' #what about back/forward slash?
files = os.listdir(folderPath)

order = ['822512060522', '822512060553', '822512060853', '822512061105','823112060112','823112060874'] #what does this order signify

#cameras = [file for file in files if file.endswith(".npy")]


#cameraTransforms = np.load('cameraTransforms.npy', allow_pickle=True).item()
cameraMarkers = {}
for camera in order:
    serial = camera.split(".")[0]
    cameraFiles = [file for file in files if file.startswith(serial)] #bro...
    cameraMarkers[serial] = [file.split(".")[0][12:] for file in cameraFiles if file.endswith(".c3d")]

markers = {}
allPoints = np.zeros((3,0,1)) #is this is all the points (in a frame?)
for i, camera in enumerate(order): #needs explanation through line 64
    for marker in cameraMarkers[camera]:
        source = ezc3d.c3d(os.path.join(folderPath,camera+marker+".c3d"))
        sourceAll = np.asarray([source['data']['points'][0], source['data']['points'][1], source['data']['points'][2]]).squeeze()
        
        if sourceAll.shape[1] != 899:
            sourceAll = np.delete(sourceAll, -1, axis=1)
        if not np.any(np.isnan(sourceAll)):
            #sourceAll = sourceAll[:,~np.all(sourceAll==0., axis=0)]
            try:
                sourceAll = np.vstack((sourceAll, np.ones(sourceAll.shape[1])))
            except:
                print('fail')
            #sourceAll = np.matmul(cameraTransforms[camera], sourceAll)
            if marker in list(markers.keys()):
                existArray = markers[marker]
                for f in range(0, sourceAll.shape[1]):
                    if (existArray[:,f] == np.array([0.,0.,0.,1.])).all():
                        existArray[:,f] = sourceAll[:,f]
                    elif (sourceAll[:,f] != np.array([0.,0.,0.,1.])).all():
                        existArray[:,f] = (existArray[:,f]+sourceAll[:,f])/2
                markers[marker] = existArray
            else:
                    markers[marker] = sourceAll


for mark in markers:
    arr = markers[mark][:3,:]
    arr[np.abs(arr) < 1] = 0
    gaps = []
    i = 0
    for a in arr.T:
        v = a == np.array([0.,0.,0.,])
        if v.all():
            gaps.append(i) #what does this do?
        i+=1
    if gaps!=[]: #and this?
        gaps = group_consecutives(gaps)
        print(mark, gaps)
        for g in range(0, len(gaps)):
            gap = gaps[g]
            steps = len(gap)
            if gap[0]>1 and gap[-1]<898:
                start = arr[:,gap[0]-1]
                end = arr[:,gap[-1]+1]
                stepLength = np.asarray([(end[q]-start[q])/steps for q in range(0,3)])
                for p in range(0,len(gap)):
                    arr[:,gap[p]] = start + ((p+1) * stepLength)
            elif gap[0]<=1:
                for p in range(0,len(gap)):
                    arr[:,gap[p]] = arr[:,gap[-1]+1]          
            elif gap[-1]>=898:
                for p in range(0,len(gap)):
                    arr[:,gap[p]] = arr[:,gap[0]-1]          
    markers[mark] = arr
'''
for mark in markers:
    arr = markers[mark][:3,:]
    for axis in range(0,3):
        idx = np.where(arr[axis]==0.)[0]
        frames = list(range(0,len(arr[axis])))
        framesN0 = np.delete(frames,idx[0])
        

    for f,x in enumerate(xes):
'''        




c3dtocopy = ezc3d.c3d("2019_05_07_01.c3d") #don't really understand below here
c3d = ezc3d.c3d()

markerList = list(markers.keys())
allPoints = np.zeros((3, len(markerList), 899))

for m, marker in enumerate(markers):
    allPoints[:,m,:] = markers[marker][:3,:]

c3d['parameters']['POINT'] = c3dtocopy['parameters']['POINT']

c3d['parameters']['POINT']['RATE']['value'] = [90]
c3d['parameters']['POINT']['LABELS']['value'] = tuple(markerList)
c3d['data']['points'] = allPoints
#c3d.add_parameter('POINT', 'DATA_START', 13)
c3d.add_parameter('POINT', 'FRAMES', 899)
c3d.add_parameter('POINT', 'SCALE', -0.01)
#c3d.add_parameter('POINT', 'USED', len(lm_list))
c3d.write(os.path.join(folderPath,'static'+".c3d"))
#np.save('cameraTransforms', cameraTransforms, allow_pickle=True)