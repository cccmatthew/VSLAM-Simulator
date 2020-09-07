import json

def cameraConstraints():
    fname = open('docs/map.msg','r')
    data = fname.read()
    result = json.loads(data)
    constriants = result['cameras']['g2o virtual']
    cx = constriants['cx']
    cy = constriants['cy']
    fx = constriants['fx']
    fy = constriants['fy']
    k1 = constriants['k1']
    k2 = constriants['k2']
    k3 = constriants['k3']
    p1 = constriants['p1']
    p2 = constriants['p2']
    return cx, cy, fx, fy, k1, k2, k3, p1, p2 

def landmark():
    landmark = []
    fname = open('docs/map.msg','r')
    data = fname.read()
    result = json.loads(data)
    landmarks = result['landmarks']
    for key in landmarks:
        landmark.append(landmarks[key]['pos_w'])
    return landmark, landmarks

def keyframe():
    fname = open('docs/map.msg','r')
    data = fname.read()
    landmarkId = list()
    result = json.loads(data)

    keyframes = result['keyframes']

    return keyframes

def combine():
    '''combine ground truth id with the vslam id'''
    combineID = dict()
    point, points = landmark()
    fname = open('docs/map.msg','r')
    data = fname.read()
    result = json.loads(data)
    keyframes = result['keyframes']
    
    for keyframe in keyframes:
        key = keyframes[keyframe]
        for descs, lm_id in zip(key['descs'], range(len(key['lm_ids']))):
            if key['lm_ids'][lm_id] != -1:
                lmid = str(key['lm_ids'][lm_id])
                combineID[descs[1]] = {
                    'VslamId': key['lm_ids'][lm_id],
                    'Estimated' :  points[lmid]['pos_w']
                }
    return combineID

def frameToLandmark():
    tmp = dict()
    fname = open('docs/detection.json','r')
    data = fname.read()
    result = json.loads(data)
    landmarks = result['result'].copy()

    fname = open('docs/groundTruth.json','r')
    data = fname.read()
    result = json.loads(data)
    print(len(result['poses']))

    for i in range(len(result['poses'])):
        for landmark in landmarks:
            if i == landmark['visibleFrame']:
                print(landmark['imagePoint'])
    return tmp


if __name__ == "__main__":
    # combine()
    # keyframe()
    # landmark()
    # print(cameraConstraints())
    print(frameToLandmark())