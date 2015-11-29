import times, strutils, algorithm, tables, math, colors
import graphics, nimPNG
from sdl import Surface

import dataprocessing

let routes = readData("train.txt").splitToRoutes()

const
    width = 1450
    height = 650
var surf = newScreenSurface(width, height)
surf.fillSurface(colWhite)
#var i = 0
var data = newString(12000*10000*3)
for i in data.low .. data.high:
    data[i] = char(255)
for route in routes.values:
    #if route.data[0].id == "80c48":
        for i in route.data.low ..< route.data.high:
            let
                x = route.data[i].coords.x
                y = route.data[i].coords.y
            data[(y.int*12000 + x.int)*3 + 0] = char(route.color.extractRGB().r)
            data[(y.int*12000 + x.int)*3 + 1] = char(route.color.extractRGB().g)
            data[(y.int*12000 + x.int)*3 + 2] = char(route.color.extractRGB().b)
            let 
                speed = sqrt(pow(route.data[i+1].coords.x - route.data[i].coords.x, 2) +
                             pow(route.data[i+1].coords.y - route.data[i].coords.y, 2)) /
                        (route.data[i+1].time - route.data[i].time).float
                distance = sqrt(pow(x, 2) + pow(y, 2))
            if floor(distance / 10).int >= 0 and floor(distance / 10).int < width and
               floor(speed * 20).int >= 0 and floor(speed * 20).int < height:
                surf[floor(distance / 10).int, height - 1 - floor(speed * 20).int] = route.color
            else:
                echo x, " ", y
                echo distance, " ", speed
        #if i > 12:
        #    break
        #else:
        #    i.inc
sdl.updateRect(surf.s, 0, 0, width, height)
surf.writeToBmp("speed-distance.bmp")
echo savePNG24("map.png", data, 12000, 10000)
withEvents(surf, event):
    var eventp = addr(event)
    case event.kind:
        of sdl.QUITEV:
            break
        else: discard
    sdl.updateRect(surf.s, 0, 0, width, height)

