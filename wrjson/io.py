''' JSON file manipulation'''

import json
import base64

def encodeJSON():
    fname = open('detection.json','r')
    data = fname.read()
    datastr = json.dumps(data)
    encoded = base64.b64encode(datastr.encode('utf-8'))
    print(encoded)

def showResult():
    fname = open('detection.json','r')
    data = fname.read()
    result = json.loads(data)
    print(result['cameras'])
    for points in result:
        print("Imagepoint: ",points['imagePoint'])
        for frames in points['visibleFrame']:
            print("Frame: ",frames)
            print("X-coordinate", points['visibleFrame'][frames]['x'])
            print("Y-coorindate", points['visibleFrame'][frames]['y'])

def clearVslam():
    fname = 'docs/detection.json'
    template = {
        "result":[]
    }
    with open(fname,'w') as fw:
        json.dump(template,fw,indent=2)


def writeToVslam(imagePoint, frame):
    fname = 'docs/detection.json'
    result = dict()
    coordinate = dict()
    visibleFrame = dict()
    data = dict()

    with open(fname) as fp:
        render = json.load(fp)
        tmp = render['result']
        data['imagePoint'] = imagePoint

        for x in range(len(frame)):
            coordinate['x'] = frame[x][1][0]
            coordinate['y'] = frame[x][1][1]
            visibleFrame[frame[x][0]] = coordinate.copy()

        data['visibleFrame'] = visibleFrame
        tmp.append(data)
        result['result'] = tmp
        
    with open(fname,'w') as fw:
        json.dump(result,fw,indent=4)

    return None

def readGroundTruth():
    fname = open('docs/groundTruth.json','r')
    data = fname.read()
    result = json.loads(data)
    return result

def writeGroundTruth(groundTruth):
    fname = "docs/groundTruth.json"
    with open(fname,'w') as fw:
        json.dump(groundTruth, fw, indent = 4)
    
    return None

if __name__ == "__main__":
    showResult()

    



        


