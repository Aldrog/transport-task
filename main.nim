import algorithm, path, math, linalg, strutils, times
import dataprocessing, classification
{.passL:"-lcblas".}

const startPoint = (x: 11038.08464497, y: 8253.17542416)
const endPoint = (x: 283.08479678, y: 163.45489494)
const middleStop = (x: 3425.67079005, y: 3469.94198377)
const fragmentSize = 10.0

proc fragmentNumber(relativePosition: float): int =
    floor(relativePosition / fragmentSize).int

type RouteSegment = object
    positions: array[2, float]
    timestamps: array[2, Time]

let sourceData = readData("train.txt")
let busRoutes = splitToBusRoutes(sourceData)
var pathInput: File
if not pathInput.open("pathVertices.txt"):
    raise new IOError
var pathVert = newSeq[gpsPoint](0)
while not pathInput.endOfFile:
    let vertex = pathInput.readLine().split()
    pathVert.add((vertex[0].parseFloat(), vertex[1].parseFloat()))
let startPosition = pointPositionWithNext(pathVert, startPoint, endPoint)
let endPosition = pointPositionWithPrev(pathVert, endPoint, startPoint)
let routeLen = endPosition - startPosition

let middleStopPosition = pointPositionWithPrev(pathVert, middleStop, startPoint) - startPosition

var segments = newSeq[RouteSegment](0)
for i in 0..<busRoutes.len:
    for j in 1..<busRoutes[i].locations.len:
        var curPos = pointPositionWithPrev(pathVert, busRoutes[i].locations[j],
                                           busRoutes[i].locations[j - 1])
        var prevPos = pointPositionWithNext(pathVert, busRoutes[i].locations[j - 1],
                                            busRoutes[i].locations[j])
        if curPos >= 0 and prevPos >= 0 and curPos >= prevPos:
            curPos -= startPosition
            prevPos -= startPosition
            if prevPos <= routeLen and curPos >= 0:
                let curTime = busRoutes[i].time[j]
                let prevTime = busRoutes[i].time[j - 1]
                segments.add RouteSegment(positions: [prevPos, curPos],
                                          timestamps: [prevTime, curTime])
echo segments.len
proc price(x: seq[int]): float =
    if x.len != 37:
        return -1.0
    else:
        result = (x[1] - x[0]).float
        for i in 1..<x.high:
            if (x[i + 1] - x[i]).float < result:
                result = (x[i + 1] - x[i]).float
        #result /= (abs(x.len - 39) + 1).float

var endRes: seq[int]
ransac(segments, seq[int], 1000, 10, price, dataSet, res, endRes):
    res.newSeq(0)
    var classificationData = newSeq[ClassificationPoint](fragmentNumber(routeLen) + 1)
    var dsCount = newSeq[int](classificationData.len)
    for i in 0..<classificationData.len:
        classificationData[i] = zeros(2)
        dsCount[i] = 0
    for segm in dataSet:
        let speed = (segm.positions[1] - segm.positions[0]) /
                    (segm.timestamps[1] - segm.timestamps[0]).float
        #if speed > 1800:
        #    echo "Speed limit violation! Speed: ", speed
        #    echo "Position between ", segm.positions[0],
        #         " and ", segm.positions[1]
        for dsInd in max(0, min(fragmentNumber(segm.positions[0]),
                                fragmentNumber(segm.positions[1]))) ..
                     min(fragmentNumber(routeLen), max(fragmentNumber(segm.positions[0]),
                                                       fragmentNumber(segm.positions[1]))):
            classificationData[dsInd][0] += speed
            if speed > classificationData[dsInd][1]:
                classificationData[dsInd][1] = speed
            inc dsCount[dsInd]
    for i in 0..<classificationData.len:
        if dsCount[i] != 0:
            classificationData[i][0] /= dsCount[i].float
    var k = 2
    var ok = true
    var clusters = newSeq[Cluster](0)
    while ok:
        let curClusters = splitIntoClusters(classificationData, k)
        if curClusters.whichCluster(classificationData[0]) ==
           curClusters.whichCluster(classificationData[fragmentNumber(routeLen)]) and
           curClusters.whichCluster(classificationData[0]) ==
           curClusters.whichCluster(classificationData[fragmentNumber(middleStopPosition)]):
            clusters = curClusters
        else:
            ok = false
        inc k
    #echo k - 1
    #echo clusters
    let stopCluster = clusters.whichCluster(classificationData[fragmentNumber(middleStopPosition)])
    var possibleStops = newSeq[int](0)
    if clusters.len > 0:
        for i in 0..<classificationData.len:
            if clusters.whichCluster(classificationData[i]) == stopCluster:
                possibleStops.add i
    if possibleStops.len > 0:
        possibleStops.delete(possibleStops.high)
        possibleStops.delete(0)
        possibleStops.sort do (x, y: int) -> int:
            cmp(classification.dist(classificationData[x], clusters[stopCluster].center),
                classification.dist(classificationData[y], clusters[stopCluster].center))
        for i in possibleStops:
            if (i - 1) notin possibleStops and (i + 1) notin possibleStops:
                res.add i
        while res.len > 37:
            res.delete(res.high)
        var ind = 0
        while res.len < 37:
            if possibleStops[ind] notin res and
              (possibleStops[ind] - 1) notin res and (possibleStops[ind] + 1) notin res:
                res.add possibleStops[ind]
            inc ind
        res.sort(cmp[int])
echo endRes
var output: File
if not output.open("output.txt", fmWrite):
    raise new IOError
for stop in endRes:
    let coords = globalPosition(pathVert, stop.float * fragmentSize +
                                          fragmentSize/2 + startPosition)
    output.writeLine(coords.x, "\t", coords.y)

