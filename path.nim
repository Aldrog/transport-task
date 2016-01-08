import math
import dataprocessing

const lineWidth* = 25.0

type Path* = seq[gpsPoint]

proc distance*(a, b: gpsPoint): float =
    sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2))

proc pointPositionWithPrev*(path: Path, p: gpsPoint, prevP: gpsPoint): float =
    result = -1.0
    var prevFound = false
    var justStarted = true
    for dummyIndex in 0..1:
        var passedDist = 0.0
        for i in 0..<path.high:
            let partLen = distance(path[i + 1], path[i])
            let k1 = (path[i + 1].y - path[i].y) / (path[i + 1].x - path[i].x)
            let b1 = path[i].y - k1*path[i].x
            let k2 = -1 / ((path[i + 1].y - path[i].y) / (path[i + 1].x - path[i].x))
            var prevPosOnPart = -1.0
            if not prevFound:
                let b2 = prevP.y - k2*prevP.x
                let proection = (x: (b2 - b1) / (k1 - k2), y: (k1*b2 - k2*b1) / (k1 - k2))
                prevPosOnPart = distance(proection, path[i])
                # Double check sign for case when line is absolutely vertical
                if (proection.x - path[i].x) * (path[i + 1].x - path[i].x) < 0 or
                   (proection.y - path[i].y) * (path[i + 1].y - path[i].y) < 0:
                    prevPosOnPart *= -1
                if distance(proection, prevP) < lineWidth and
                   prevPosOnPart >= 0 and prevPosOnPart <= partLen:
                    prevFound = true
                else:
                    prevPosOnPart = -1
            if not prevFound and distance(prevP, path[i]) < lineWidth:
                prevFound = true
            if prevFound:
                let pB2 = p.y - k2*p.x
                let proection = (x: (pB2 - b1) / (k1 - k2), y: (k1*pB2 - k2*b1) / (k1 - k2))
                var posOnPart = distance(proection, path[i])
                # Double check sign for case when line is absolutely vertical
                if (proection.x - path[i].x) * (path[i + 1].x - path[i].x) < 0 or
                   (proection.y - path[i].y) * (path[i + 1].y - path[i].y) < 0:
                    posOnPart *= -1
                if distance(proection, p) < lineWidth and
                   posOnPart >= 0 and posOnPart <= partLen and
                   posOnPart > prevPosOnPart:
                    return passedDist + posOnPart
            if not justStarted:
                if prevFound and distance(p, path[i]) < lineWidth:
                    return passedDist
            else:
                justStarted = false
            passedDist += partLen

proc pointPositionWithNext*(path: Path, p: gpsPoint, nextP: gpsPoint): float =
    result = -1.0
    var justStarted = true
    var pointPos = -1.0
    var nextFound = false
    for dummyIndex in 0..1:
        var passedDist = 0.0
        for i in 0..<path.high:
            let partLen = distance(path[i + 1], path[i])
            let k1 = (path[i + 1].y - path[i].y) / (path[i + 1].x - path[i].x)
            let b1 = path[i].y - k1*path[i].x
            let k2 = -1 / ((path[i + 1].y - path[i].y) / (path[i + 1].x - path[i].x))
            let b2 = p.y - k2*p.x
            let proection = (x: (b2 - b1) / (k1 - k2), y: (k1*b2 - k2*b1) / (k1 - k2))
            var posOnPart = distance(proection, path[i])
            # Double check sign for case when line is absolutely vertical
            if (proection.x - path[i].x) * (path[i + 1].x - path[i].x) < 0 or
               (proection.y - path[i].y) * (path[i + 1].y - path[i].y) < 0:
                posOnPart *= -1
            if distance(proection, p) < lineWidth and
               posOnPart >= 0 and posOnPart <= partLen:
                pointPos = passedDist + posOnPart
            else:
                posOnPart = -1.0
            if pointPos >= 0:
                let nextB2 = nextP.y - k2*nextP.x
                let proection = (x: (nextB2 - b1) / (k1 - k2), y: (k1*nextB2 - k2*b1) / (k1 - k2))
                var nextPosOnPart = distance(proection, path[i])
                # Double check sign for case when line is absolutely vertical
                if (proection.x - path[i].x) * (path[i + 1].x - path[i].x) < 0 or
                   (proection.y - path[i].y) * (path[i + 1].y - path[i].y) < 0:
                    nextPosOnPart *= -1
                if distance(proection, nextP) < lineWidth and
                   nextPosOnPart >= 0 and nextPosOnPart <= partLen and
                   nextPosOnPart > posOnPart:
                    nextFound = true
            if not justStarted:
                if pointPos >= 0 and pointPos < passedDist and distance(nextP, path[i]) < lineWidth:
                    nextFound = true
                if pointPos < passedDist and distance(p, path[i]) < lineWidth:
                    pointPos = passedDist
            else:
                justStarted = false
            passedDist += partLen
            if nextFound and pointPos >= 0:
                return pointPos

proc globalPosition*(path: Path, posOnPath: float): gpsPoint =
    var passedDist = 0.0
    for i in 0..<path.high:
        var partLen = distance(path[i], path[i + 1])
        if posOnPath < passedDist + partLen:
            let sinus = (path[i + 1].y - path[i].y) / partLen
            let cosinus = (path[i + 1].x - path[i].x) / partLen
            let offsetX = path[i].x + sinus * 5
            let offsetY = path[i].y - cosinus * 5
            return (x: offsetX + cosinus * (posOnPath - passedDist),
                    y: offsetY + sinus * (posOnPath - passedDist))
        passedDist += partLen

