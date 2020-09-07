'''sim3'''
import numpy as np
from wrjson import readmap

def centroid(pointCloud):
    x = 0
    y = 0
    z = 0
    pointCloud = np.array(pointCloud)
    centroid = list()
    for i in range(len(pointCloud)):
        x += pointCloud[i][0]
        y += pointCloud[i][1]
        z += pointCloud[i][2]
    centroid.append(x / len(pointCloud))
    centroid.append(y / len(pointCloud))
    centroid.append(z / len(pointCloud))
    return centroid

def recentre(centroid, pointCloud):
    pointCloud = np.array(pointCloud)
    for i in range(len(pointCloud)):
        pointCloud[i][0] = pointCloud[i][0] - centroid[0]
        pointCloud[i][1] = pointCloud[i][1] - centroid[1]
        pointCloud[i][2] = pointCloud[i][2] - centroid[2]

    return pointCloud

def scale(pointCloud, pointCloudX):
    sumDistance = 0
    sumDistanceX = 0
    error = 0
    pointCloud = np.array(pointCloud)
    pointCloudX = np.array(pointCloudX)

    for i, j in zip(list(range(len(pointCloud))),list(range(len(pointCloudX)))):
        sumDistance += np.sqrt((pointCloud[i][0])**2 + (pointCloud[i][1])**2 + (pointCloud[i][2])**2)
        sumDistanceX += np.sqrt((pointCloudX[i][0])**2 + (pointCloudX[i][1])**2 + (pointCloudX[i][2])**2)

    scale = sumDistance / sumDistanceX
    pointCloudX = pointCloudX * scale

    return pointCloudX

def reverse(centroid, pointCloud):
    pointCloud = np.array(pointCloud)
    for i in range(len(pointCloud)):
        pointCloud[i][0] = pointCloud[i][0] + centroid[0]
        pointCloud[i][1] = pointCloud[i][1] + centroid[1]
        pointCloud[i][2] = pointCloud[i][2] + centroid[2]
    return pointCloud

def meanSquareError(estimation, pointCloud):
    error = 0
    pointCloud = np.array(pointCloud)
    for point in estimation:
        Id = estimation[point]
        error += (pointCloud[point][0] - Id['Estimated'][0])**2 + (pointCloud[point][1] - Id['Estimated'][1])**2 + (pointCloud[point][2] - Id['Estimated'][2])**2
    error = error / len(estimation)
    return error
