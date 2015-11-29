import strutils, times, colors, tables, algorithm

type DataEntry* = object
    id*: string
    time*: Time
    coords*: tuple[x, y: float]
type Route* = object
    data*: seq[DataEntry]
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

proc splitToRoutes*(data: seq[DataEntry]): Table[string, Route] =
    result = initTable[string, Route]()
    for entry in data:
        if not result.hasKey(entry.id):
            result[entry.id] = Route(data: newSeq[DataEntry](0), color: parseColor("#" & entry.id))
        result[entry.id].data.add entry
    for k in result.keys:
        result[k].data.sort(proc(a, b: DataEntry): int = cmp(a.time, b.time))

