import math, seqUtils, algorithm, typetraits
import dataprocessing, path

const maxNeighbourDistance = 100.0

type Angle = distinct float
proc `<`(a, b: Angle): bool {.borrow.}
proc `-`(x: Angle): Angle {.borrow.}

proc `<>`(x: Angle, bounds: tuple[l, u: Angle]): bool =
    if bounds.l < bounds.u:
        x > bounds.l and x < bounds.u
    else:
        x < bounds.l and x > bounds.u

proc `+`(a, b: Angle): Angle =
    result = (a.float + b.float).Angle
    if result > Pi.Angle:
        result = (result.float - Pi).Angle
    if result < (-Pi).Angle:
        result = (result.float + Pi).Angle

proc `-`(a, b: Angle): Angle =
    a + (-b)

proc approximatePath*(data: seq[BusRoute]): Path =
    var maxCoords = (x: 0.0, y: 0.0)
    for route in data:
        for p in route.locations:
            if p.x > maxCoords.x:
                maxCoords.x = p.x
            if p.y > maxCoords.y:
                maxCoords.y = p.y
    echo "field size: ", maxCoords
    var possibleIndeces = newSeq[tuple[rIndex, pIndex: int]](0)
    for ri in 0..<data.len:
        for pi in 1..<data[ri].locations.len:
            possibleIndeces.add((ri, pi))

    proc price(polygon: Path): float =
        for entry in possibleIndeces:
            if polygon.pointPositionWithPrev(data[entry.rIndex].locations[entry.pIndex],
                                      data[entry.rIndex].locations[entry.pIndex - 1]) >= 0:
                result += 1
        echo result
        if result < 10000:
            result = -1
        else:
            result /= polygon.len.float - 1
    ransac(possibleIndeces, Path, 10000, 1, price, dataSet, res, result):
        res = newSeq[gpsPoint](0)
        var cornerPoint = (x: 59.672867, y: 51.720605)#maxCoords#(x: 0.0, y: 0.0)
        #for entry in dataSet:
        #    let p = data[entry.rIndex].data[entry.pIndex]
        #    if p.x < cornerPoint.x or
        #      (p.x == cornerPoint.x and p.y < cornerPoint.y):
        #        cornerPoint = p
        #cornerPoint = (x: 283.08479678, y: 163.45489494)
        res.add cornerPoint
        echo cornerPoint
        dataSet.keepIf(proc(item: tuple[rIndex, pIndex: int]): bool =
            let p = data[item.rIndex].locations[item.pIndex]
            return p != cornerPoint)
        var w_l: Angle
        var w_u: Angle
        while res.len < 2 or res[res.high] != res[res.high - 1]:
            dataSet.sort(proc(a, b: tuple[rIndex, pIndex: int]): int =
                let p1 = data[a.rIndex].locations[a.pIndex]
                let p2 = data[b.rIndex].locations[b.pIndex]
                cmp(distance(p1, cornerPoint), distance(p2, cornerPoint)) )
            #echo "closest: ", approxData[0].coords
            var tempSet = dataSet
            var bestVal = 0
            var qCond = false
            while not qCond:
                qCond = true
                var currentPoint = cornerPoint
                if res.len < 2:
                    w_l = (-Pi).Angle
                    w_u = Pi.Angle
                else:
                    let w_base = arctan2(res[res.high].y - res[res.high - 1].y,
                                         res[res.high].x - res[res.high - 1].x).Angle
                    # Don't allow more than 150 degree turns
                    w_l = w_base - (5*Pi/6).Angle
                    w_u = w_base + (5*Pi/6).Angle
                var i = 0
                while i < tempSet.len:
                    let
                        p = data[tempSet[i].rIndex].locations[tempSet[i].pIndex]
                        #prevPoint = data[dataSet[i].rIndex].data[dataSet[i].pIndex - 1]
                        x = p.x - cornerPoint.x
                        y = p.y - cornerPoint.y
                        #prevPos = positionOnPath(res & currentPoint, prevPoint)
                    if arctan2(y, x).Angle <> (w_l, w_u) and
                       #(res.len <= 2 or prevPos >= 0) and
                       distance(p, currentPoint) < maxNeighbourDistance:
                        #echo x, " ", y
                        #echo prevPos
                        let wu_candidate = arctan2(y + lineWidth * x/sqrt(pow(x,2)+pow(y,2)),
                                               x - lineWidth * y/sqrt(pow(x,2)+pow(y,2))).Angle
                        let wl_candidate = arctan2(y - lineWidth * x/sqrt(pow(x,2)+pow(y,2)),
                                               x + lineWidth * y/sqrt(pow(x,2)+pow(y,2))).Angle
                        #if x == 0 and y == 0:
                        #    echo "CORNER IS IN THE DATASET"
                        if wu_candidate <> (w_l, w_u):
                            w_u = wu_candidate
                        if wl_candidate <> (w_l, w_u):
                            w_l = wl_candidate
                        currentPoint = p
                        qCond = false
                        #cpInd = dataSet[i]
                        tempSet.delete(i)
                        dec i
                        for entry in dataSet:
                            let point = data[entry.rIndex].locations[entry.pIndex]
                            let prevPoint = data[entry.rIndex].locations[entry.pIndex - 1]
                            var closePs = 0
                            var linePs = 0
                            if pointPositionWithPrev(res & p, point, prevPoint) >= 0 and
                               distance(currentPoint, point) < lineWidth*4:
                                inc closePs
                                if arctan2(point.y - cornerPoint.y,
                                           point.x - cornerPoint.x).Angle <> (w_l, w_u):
                                    inc linePs
                            if closePs > 2*linePs:
                                # set corner
                                break
                    inc i
                var val = 0
                for entry in dataSet:
                    if pointPositionWithPrev(res & currentPoint,
                                      data[entry.rIndex].locations[entry.pIndex],
                                      data[entry.rIndex].locations[entry.pIndex - 1]) >= 0:
                        inc val
                if val > bestVal:
                    cornerPoint = currentPoint
                    bestVal = val
            res.add cornerPoint
            #echo res.len, " ", dataSet.len
            #echo cornerPoint.x, " ", cornerPoint.y
            var i = 0
            while i < dataSet.len:
                let pPos = pointPositionWithPrev(res,
                           data[dataSet[i].rIndex].locations[dataSet[i].pIndex],
                           data[dataSet[i].rIndex].locations[dataSet[i].pIndex - 1])
                #if data[dataSet[i].rIndex].data[dataSet[i].pIndex] == cornerPoint:
                #    echo pPos, " ", prevPPos
                if dataSet[i].pIndex < data[dataSet[i].rIndex].locations.high:
                    let nextPPos = pointPositionWithNext(res,
                                   data[dataSet[i].rIndex].locations[dataSet[i].pIndex],
                                   data[dataSet[i].rIndex].locations[dataSet[i].pIndex + 1])
                    if nextPPos >= 0 or
                       pPos >= 0:
                        dataSet.delete(i)
                        dec i
                else:
                    if pPos >= 0:
                        dataSet.delete(i)
                        dec i
                inc i
    echo result

when isMainModule:
    let data = readData("train.txt")
    let approxPath = approximatePath(data.splitToBusRoutes())
    var output: File
    if not output.open("pathVertices.txt", fmWrite):
        raise new IOError
    for vertex in approxPath:
        output.writeLine($vertex.x & "\t" & $vertex.y)

