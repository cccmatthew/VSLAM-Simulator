# https://github.com/stevenlovegrove/Pangolin/tree/master/examples/HelloPangolin

import OpenGL.GL as gl
import sim3, pangolin, g2o, json
from wrjson import readmap, io
import numpy as np
from scipy.spatial.transform import Rotation as R

estimateCameraPose = dict()
estCameraPose = list()

def initEstCamera():
    frame = readmap.keyframe()
    keyFrame = list(frame.keys())
    numKeyFrame = len(keyFrame)

    for i in range(numKeyFrame):
        rotation = np.array(frame[keyFrame[i]]['rot_cw'])
        translation = frame[keyFrame[i]]['trans_cw']

        rotMatrix = g2o.Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])
        pose = g2o.SE3Quat(rotMatrix, translation)
        poseInv = g2o.SE3Quat.inverse(pose)
        centre = g2o.SE3Quat.translation(poseInv)

        estimateCameraPose[frame[keyFrame[i]]['src_frm_id']] = [centre[0], centre[1], centre[2]]
        estCameraPose.append([centre[0], centre[1], centre[2]])

    return estimateCameraPose

def estimateCamera(pose, frame):
    id = 0
    key = list(estimateCameraPose.keys())
    lastFrame = key[len(key)-1]

    if frame < lastFrame:
        for x in range(len(key)):
            if key[x] == frame:
                id = key[x]
                pose = estimateCameraPose[id]
        return pose
    else:
        id = key[len(key)-1]
        pose = estimateCameraPose[id]
    return pose

def groundTruthCamera(pose, frame):
    groundTruth = io.readGroundTruth()
    groundTruthPose = np.array(groundTruth['poses'])

    pose[0,3] = groundTruthPose[frame][0]
    pose[1,3] = groundTruthPose[frame][1]
    pose[2,3] = groundTruthPose[frame][2]

    return pose

def cameraInit():
    pose = np.identity(4)
    groundTruth = io.readGroundTruth()
    initpose = np.array(groundTruth['poses'])

    pose[0,3] = initpose[0][0]
    pose[1,3] = initpose[0][1]
    pose[2,3] = initpose[0][2]
    # cx, cy, fx, fy, k1, k2, k3, p1, p2 = readmap.cameraConstraints()
    # pose[0,2] = cx
    # pose[1,2] = cy
    # pose[0,0] = fx
    # pose[1,1] = fy
    # pose[2,2] = 1
    # print(pose)
    return pose

def cameraMSE():
    groundTruth = io.readGroundTruth()
    error = 0
    groundTruthPose = np.array(groundTruth['poses'])
    key = list(estimateCameraPose.keys())

    for i in range(len(estimateCameraPose)):
        print("In keyframe ", key[i])
        print("Ground truth pose", groundTruthPose[key[i]])
        print("Estimated pose", estimateCameraPose[key[i]][:3,3])
        error += (estimateCameraPose[key[i]][0,3] - groundTruthPose[key[i]][0])**2 + (estimateCameraPose[key[i]][1,3] - groundTruthPose[key[i]][1])**2 + (estimateCameraPose[key[i]][2,3] - groundTruthPose[key[i]][2])**2
    
    error = error / len(estimateCameraPose)
    print("Camera pose MSE:", error)
    return None

def merge(cameraCentre):
    key = list(estimateCameraPose.keys())
    for i in range(len(key)):
        estpose = np.identity(4)
        estpose[0,3] = cameraCentre[i][0]
        estpose[1,3] = cameraCentre[i][1]
        estpose[2,3] = cameraCentre[i][2]

        estimateCameraPose[key[i]] = estpose
    return estimateCameraPose

def run():
    pangolin.CreateWindowAndBind('VSLAM Simulator', 640, 480)
    gl.glEnable(gl.GL_DEPTH_TEST)
    gl.glEnable(gl.GL_BLEND)
    gl.glBlendFunc (gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
    initialize = False

    groundTruthPose = cameraInit()
    path = [groundTruthPose[:3,3].copy().tolist()]
    frame = 0
    counter = 0
    estPoints = list()

    '''define projection and initial ModelView matrix'''
    scam = pangolin.OpenGlRenderState(
        pangolin.ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 200),
        pangolin.ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin.AxisDirection.AxisY))
    handler = pangolin.Handler3D(scam)

    '''create interactive view in window'''
    dcam = pangolin.CreateDisplay()
    dcam.SetBounds(0.0, 1.0, 0.0, 1.0, -640.0/480.0)
    dcam.SetHandler(handler)

    tree = pangolin.Renderable()
    tree.Add(pangolin.Axis())

    while not pangolin.ShouldQuit():
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glClearColor(0.0, 0.0, 0.0, 0.0)
        dcam.Activate(scam)
        tree.Render()

        if  initialize == False:
            
            groundTruth = io.readGroundTruth()
            points = np.array(groundTruth['landmarks'])
            poses = groundTruth['poses']
            numPose = len(poses)

            estimation = readmap.combine()
            initEstCamera()

            '''vslam sim3'''
            for point in estimation:
                estPoints.append(estimation[point]['Estimated'])

            pointCentre = sim3.centroid(points)
            estPointCentre =  sim3.centroid(estPoints)

            poseCentre = sim3.centroid(poses)
            estPoseCentre = sim3.centroid(estCameraPose)

            recentrePoints = sim3.recentre(pointCentre, points)
            recentreEstPoints = sim3.recentre(estPointCentre, estPoints)

            recentrePose = sim3.recentre(poseCentre, poses)
            recentreEstPose = sim3.recentre(estPoseCentre, estCameraPose)

            scaledEstpoints = sim3.scale(recentrePoints, recentreEstPoints)
            scaledEstPose = sim3.scale(recentrePose, recentreEstPose)

            for point, i in zip(estimation, range(len(scaledEstpoints))):
                estimation[point]['Estimated'] = scaledEstpoints[i]
            
            print("3D point MSE:", sim3.meanSquareError(estimation, recentrePoints))

            scaledEstpoints = sim3.reverse(pointCentre, scaledEstpoints)
            scaledEstPose = sim3.reverse(poseCentre, scaledEstPose)

            merge(scaledEstPose)

            estimatePose = estimateCameraPose[0]

            initialize = True

        if initialize == True:
            colorBlue = np.zeros((len(points), 3))
            colorBlue[:, 2] = 1
            colorRed = np.zeros((len(points), 3))
            colorRed[:, 0] = 1
            gl.glPointSize(3)
            pangolin.DrawPoints(points, colorBlue)
            pangolin.DrawPoints(scaledEstpoints, colorRed)

            '''estimated camera pose '''
            estimatePose = estimateCamera(estimatePose, frame)
            gl.glLineWidth(1)
            gl.glColor3f(1.0, 0.0, 0.0)
            pangolin.DrawCamera(estimatePose, 0.5, 0.75, 0.8)

            '''ground truth camera pose '''
            groundTruthPose = groundTruthCamera(groundTruthPose, frame)
            gl.glLineWidth(1)
            gl.glColor3f(0.0, 0.0, 1.0)
            pangolin.DrawCamera(groundTruthPose, 0.5, 0.75, 0.8)

            '''path'''
            gl.glLineWidth(1)
            gl.glColor3f(1.0, 1.0, 1.0)
            currentPoint = groundTruthPose[:3,3].copy().tolist()
            path.append(currentPoint)
            path = np.array(path)
            pangolin.DrawLine(path)
            path = path.tolist()

            '''frame'''
            if counter < 3:
                counter = counter + 1
            elif counter == 3 and frame < (numPose-1):
                frame = frame + 1
                counter = 0
            elif frame == (numPose - 1):
                cameraMSE() 
                frame = 0
                counter = 0
        
        pangolin.FinishFrame()
if __name__ == "__main__":
    run()
