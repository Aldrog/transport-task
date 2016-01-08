import math, graphics, dataprocessing, strutils, colors
from sdl import Surface

var file: File
var data = newSeq[gpsPoint](0)
if not file.open("output.txt"):
    raise new IOError
while not file.endOfFile:
    let entry = file.readLine().split()
    data.add((x: parseFloat(entry[0]), y: parseFloat(entry[1])))
var pathInput: File
if not pathInput.open("pathVertices.txt"):
    raise new IOError
var pathVert = newSeq[gpsPoint](0)
while not pathInput.endOfFile:
    let vertex = pathInput.readLine().split()
    pathVert.add((vertex[0].parseFloat(), vertex[1].parseFloat()))
const
    width = 1200
    height = 800
var surf = newScreenSurface(width, height)
surf.fillSurface(colWhite)
for p in data:
    surf.drawCircle((floor(p.x / 12).int, height - 1 - floor(p.y / 12).int), 3, colRed)
for i in 0..<pathVert.high:
    surf.drawLine((floor(pathVert[i].x / 12).int, height - 1 - floor(pathVert[i].y / 12).int),
                  (floor(pathVert[i+1].x / 12).int, height - 1 - floor(pathVert[i+1].y / 12).int),
                  colGray)
sdl.updateRect(surf.s, 0, 0, width, height)
surf.writeToBmp("found-points.bmp")
withEvents(surf, event):
    var eventp = addr(event)
    case event.kind:
        of sdl.QUITEV:
            break
        else: discard
    sdl.updateRect(surf.s, 0, 0, width, height)

