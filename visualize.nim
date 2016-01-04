import times, strutils, algorithm, tables, math, colors
import graphics, nimPNG
from sdl import Surface

import dataprocessing, approximation

let data = readData("train.txt")
let routes = data.splitToRoutes()
let path = approxPath(data.splitToBusRoutes())

const
    width = 1200
    height = 800
var surf = newScreenSurface(width, height)
surf.fillSurface(colWhite)
var imgData = newString(12000*10000*3)
for i in imgData.low .. imgData.high:
    imgData[i] = char(255)

for route in routes:
    for i in route.data.low .. route.data.high:
        let
            x = route.data[i].coords.x
            y = route.data[i].coords.y
        imgData[((10000 - y.int)*12000 + x.int)*3 + 0] = char(route.color.extractRGB().r)
        imgData[((10000 - y.int)*12000 + x.int)*3 + 1] = char(route.color.extractRGB().g)
        imgData[((10000 - y.int)*12000 + x.int)*3 + 2] = char(route.color.extractRGB().b)
        if i < route.data.high:
            let 
                speed = sqrt(pow(route.data[i+1].coords.x - route.data[i].coords.x, 2) +
                             pow(route.data[i+1].coords.y - route.data[i].coords.y, 2)) /
                        (route.data[i+1].time - route.data[i].time).float
                distance = sqrt(pow(x, 2) + pow(y, 2))
            if floor(x / 12).int >= 0 and floor(x / 12).int < width and
               floor(y / 12).int >= 0 and floor(y / 12).int < height:
                surf[floor(x / 12).int, height - 1 - floor(y / 12).int] = route.color
            #if floor(distance / 10).int >= 0 and floor(distance / 10).int < width and
            #   floor(speed * 20).int >= 0 and floor(speed * 20).int < height:
            #    surf[floor(distance / 10).int, height - 1 - floor(speed * 20).int] = route.color
            #else:
            #    echo x, " ", y
            #    echo distance, " ", speed
echo "resulting path:"
var i = 0
while i < path.high:
    echo path[i]
    surf.drawLine(((path[i].x / 12).int, (height - 1 - path[i].y / 12).int),
                  ((path[i+1].x / 12).int, (height - 1 - path[i+1].y / 12).int),
                  colBlack)
    i += 1
sdl.updateRect(surf.s, 0, 0, width, height)
#surf.writeToBmp("frequency-map.bmp")
#echo savePNG24("map.png", imgData, 12000, 10000)
withEvents(surf, event):
    var eventp = addr(event)
    case event.kind:
        of sdl.QUITEV:
            break
        else: discard
    sdl.updateRect(surf.s, 0, 0, width, height)

