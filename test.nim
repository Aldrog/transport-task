import math, graphics, dataprocessing, strutils, colors
from sdl import Surface

var file: File
var data = newSeq[gpsPoint](0)
if not file.open("output.txt"):
    raise new IOError
while not file.endOfFile:
    let entry = file.readLine().split()
    data.add((x: parseFloat(entry[0]), y: parseFloat(entry[1])))
const
    width = 1200
    height = 800
var surf = newScreenSurface(width, height)
surf.fillSurface(colWhite)
for p in data:
    surf[floor(p.x / 12).int, height - 1 - floor(p.y / 12).int] = colBlack
sdl.updateRect(surf.s, 0, 0, width, height)
surf.writeToBmp("found-points.bmp")
withEvents(surf, event):
    var eventp = addr(event)
    case event.kind:
        of sdl.QUITEV:
            break
        else: discard
    sdl.updateRect(surf.s, 0, 0, width, height)

