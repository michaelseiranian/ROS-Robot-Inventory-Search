#!/usr/bin/env python3
import rospy
from second_coursework.srv import GetRoomCoordRequest, GetRoomCoordResponse, GetRoomCoord

aCoords = [[1, 10], [3, 9], [1.5, 7], [2.8, 7]]
bCoords = [[4.5, 10], [7.5, 9.4], [4.6, 7], [6.8, 7.7]]
cCoords = [[9.3, 9.7], [11.8, 9.7], [9.5, 7.1], [11.7, 7.1]]
dCoords = [[0.7, 5], [3, 5], [1, 1.8], [2.9, 2.5]]
eCoords = [[4.6, 4.5], [7.2, 4.6], [4.7, 1.6], [7.1, 2.5]]
fCoords = [[9, 4], [12, 3.7], [9.3, 1.1], [11.7, 1.5]]
a, b, c, d, e, f = 0, 0, 0, 0, 0, 0


def return_coord(req: GetRoomCoordRequest):
    global a, b, c, d, e, f
    coord = GetRoomCoordResponse()
    if req.room.lower() == "a":
        coord.coordinate.x = aCoords[a][0]
        coord.coordinate.y = aCoords[a][1]
        if a < 3:
            a += 1
        else:
            a = 0
    elif req.room.lower() == "b":
        coord.coordinate.x = bCoords[b][0]
        coord.coordinate.y = bCoords[b][1]
        if b < 3:
            b += 1
        else:
            b = 0
    elif req.room.lower() == "c":
        coord.coordinate.x = cCoords[c][0]
        coord.coordinate.y = cCoords[c][1]
        if c < 3:
            c += 1
        else:
            c = 0
    elif req.room.lower() == "d":
        coord.coordinate.x = dCoords[d][0]
        coord.coordinate.y = dCoords[d][1]
        if d < 3:
            d += 1
        else:
            d = 0
    elif req.room.lower() == "e":
        coord.coordinate.x = eCoords[e][0]
        coord.coordinate.y = eCoords[e][1]
        if e < 3:
            e += 1
        else:
            e = 0
    elif req.room.lower() == "f":
        coord.coordinate.x = fCoords[f][0]
        coord.coordinate.y = fCoords[f][1]
        if f < 3:
            f += 1
        else:
            f = 0
    else:
        return GetRoomCoordResponse()
    return coord


rospy.init_node('roomservice')
get_coord_srv = rospy.Service('return_coord', GetRoomCoord, return_coord)
rospy.spin()
