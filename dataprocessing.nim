import strutils, seqUtils, times, colors, tables, algorithm, math

type gpsPoint* = tuple[x, y: float]
const startPoint = (x: 11038.08464497, y: 8253.17542416)
const endPoint = (x: 283.08479678, y: 163.45489494)

type DataEntry* = object
    id*: string
    time*: Time
    coords*: gpsPoint

type Direction* = enum
    There, Back

type Route* = object
    data*: seq[DataEntry]
    direction*: Direction
    busID*: string
    color*: Color

type BusRoute* = object
    locations*: seq[gpsPoint]
    time*: seq[Time]
    busID*: string
    color*: Color

proc readData*(filename: string): seq[DataEntry] =
    result.newSeq(0)
    var source: File
    if not source.open(filename):
        raise new IOError
    while not source.endOfFile:
        let entry = source.readLine().split()
        # Casting to Time works because Time is implemented as UNIX time
        result.add DataEntry(time: Time(entry[0].parseInt()),
                             coords: (entry[1].parseFloat(), entry[2].parseFloat()),
                             id: entry[3])
    source.close()

proc splitToBusRoutes*(data: seq[DataEntry]): seq[BusRoute] =
    var tempRoutes = initTable[string, BusRoute]()
    for entry in data:
        if not tempRoutes.hasKey(entry.id):
            tempRoutes[entry.id] = BusRoute(locations: newSeq[gpsPoint](0),
                                            time: newSeq[Time](0),
                                            busID: entry.id,
                                            color: parseColor("#" & entry.id))
        tempRoutes[entry.id].locations.add entry.coords
        tempRoutes[entry.id].time.add entry.time
    return toSeq(tempRoutes.values)

proc splitToRoutes*(data: seq[DataEntry]): seq[Route] =
    result.newSeq(0)
    var tempRoutes = initTable[string, Route]()
    for entry in data:
        if not tempRoutes.hasKey(entry.id):
            tempRoutes[entry.id] = Route(data: newSeq[DataEntry](0),
                                         busID: entry.id,
                                         color: parseColor("#" & entry.id))
        tempRoutes[entry.id].data.add entry
    for k in tempRoutes.keys:
        tempRoutes[k].data.sort(proc(a, b: DataEntry): int = cmp(a.time, b.time))
    for tmpRoute in tempRoutes.values:
        var route = newSeq[DataEntry](0)
        for i in tmpRoute.data.low + 1 .. tmpRoute.data.high:
            if tmpRoute.data[i].coords.x > startPoint.x and
               tmpRoute.data[i].coords.y > startPoint.y and
               (tmpRoute.data[i - 1].coords.x < startPoint.x or
               tmpRoute.data[i - 1].coords.y < startPoint.y):
                result.add Route(data: route,
                                 direction: Back,
                                 busID: tmpRoute.busID,
                                 color: rgb(255,
                                            tmpRoute.color.extractRGB.g,
                                            tmpRoute.color.extractRGB.b))
                route.newSeq(0)
            if tmpRoute.data[i].coords.x < endPoint.x and
               tmpRoute.data[i].coords.y < endPoint.y and
               (tmpRoute.data[i - 1].coords.x > endPoint.x or
               tmpRoute.data[i - 1].coords.y > endPoint.y):
                result.add Route(data: route,
                                 direction: There,
                                 busID: tmpRoute.busID,
                                 color: rgb(0,
                                            tmpRoute.color.extractRGB.g,
                                            tmpRoute.color.extractRGB.b))
                route.newSeq(0)
            if (tmpRoute.data[i].coords.x < startPoint.x or
               tmpRoute.data[i].coords.y < startPoint.y) and
               (tmpRoute.data[i - 1].coords.x > startPoint.x and
               tmpRoute.data[i - 1].coords.y > startPoint.y):
                route.newSeq(0)
            if (tmpRoute.data[i].coords.x > endPoint.x or
               tmpRoute.data[i].coords.y > endPoint.y) and
               (tmpRoute.data[i - 1].coords.x < endPoint.x and
               tmpRoute.data[i - 1].coords.y < endPoint.y):
                route.newSeq(0)
            route.add tmpRoute.data[i]

randomize()
template ransac*(data: seq[expr]; resType: typedesc; setSize, reqCount: int;
         price: (proc(x: resType): float); dataSet, res, endRes: expr; resEval: stmt): stmt {.
         immediate.} =
    #var endRes: resType
    var dataSet: data.type
    var results: seq[resType]
    var prices = newSeq[float](0)
    results.newSeq(0)
    while results.len < reqCount:
        dataSet.newSeq(0)
        while dataSet.len < setSize:
            let entry = data[random(data.len)]
            if entry notin dataSet:
                dataSet.add entry
        var res: resType
        resEval
        if price(res) > 0:
            results.add res
            prices.add price(res)
            echo results.len, " ", price(res)
    var maxPrice = 0.0
    for i in 0..results.high:
        if prices[i] >= maxPrice:
            endRes = results[i]
            maxPrice = prices[i]
            echo prices[i]

