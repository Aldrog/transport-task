import math, linalg
#import dataprocessing, approximation

const maxIterations = 100
const maxAcceptedError = 0.05
const dataComplexity = 2

type ClassificationPoint* = Vector64[dataComplexity]
type Cluster* = object
    center*: ClassificationPoint

proc dist*(a, b: ClassificationPoint): float =
    for i in 0..<a.len:
        result += pow(a[i] - b[i], 2)

proc splitIntoClusters*(data: seq[ClassificationPoint], K: int): seq[Cluster] =
    result.newSeq(K)
    for k in 0..<K:
        result[k] = Cluster(center: random(data))
    for i in 1..maxIterations:
        var newCenter = newSeq[Cluster](K)
        for k in 0..<K:
            newCenter[k] = Cluster(center: zeros(dataComplexity))
        var kNumber = newSeq[int](K)
        for entry in data:
            var closest = 0
            var minD = dist(entry, result[0].center)
            for k in 1..<K:
                if dist(entry, result[k].center) < minD:
                    closest = k
                    minD = dist(entry, result[k].center)
            newCenter[closest].center += entry
            inc kNumber[closest]
        for k in 0..<K:
            if kNumber[k] != 0:
                newCenter[k].center /= kNumber[k].float
        var maxDist = 0.0
        for k in 0..<K:
            if dist(result[k].center, newCenter[k].center) > maxDist:
                maxDist += dist(result[k].center, newCenter[k].center)
        if maxDist < maxAcceptedError:
            return newCenter
        result = newCenter

proc whichCluster*(clusters: seq[Cluster], point: ClassificationPoint): int =
    result = 0
    for i in 1..<clusters.len:
        if dist(point, clusters[i].center) < dist(point, clusters[result].center):
            result = i

