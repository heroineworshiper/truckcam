# isogrid library

# note the as keywords inside a library
import FreeCAD as App
import FreeCADGui as Gui
import Sketcher
import Part
import math



doc = App.activeDocument()
gui = Gui.activeDocument()

SKETCH_Z = 20.0
# flip the triangle direction
FLIP = False

def extruderDefaults(x):
    x.DirMode = "Normal"
    x.DirLink = None
    x.LengthFwd = 0.0
    x.LengthRev = 0.0
    x.Solid = True
    x.Reversed = False
    x.Symmetric = False
    x.TaperAngle = 0.000000000000000
    x.TaperAngleRev = 0.000000000000000


def toRad(angle):
    return angle * math.pi * 2.0 / 360.0

def toDeg(x):
    return x * 360.0 / 2 / math.pi

# add vector with length & angle to start 
def addLine(start, angle, length):
    result = App.Vector(start.x + length * math.cos(angle),
        start.y + length * math.sin(angle),
        start.z)
    return result

def angleChange(old_angle, new_angle):
    result = new_angle - old_angle
    if result > math.pi: 
        result -= math.pi * 2
    elif result < -math.pi:
        result += math.pi * 2
    return result

# make a closed polygon
def makePoly(sketch, coords, closeIt = True):
    prevLine = None
    for i in range(0, len(coords) - 1):
        sketch.addGeometry(Part.LineSegment(coords[i],
            coords[i + 1]),
            False)
# constrain 2 lines. Crashes
#        if i > 0:
#            sketch.addConstraint(Sketcher.Constraint('Coincident', 
#                i - 1,
#                2,
#                i, 
#                1))

    if closeIt:
        sketch.addGeometry(Part.LineSegment(coords[len(coords) - 1],
            coords[0]),
            False)
#        sketch.addConstraint(Sketcher.Constraint('Coincident', 
#            0,
#            1,
#            len(coords) - 1, 
#            2))



# make an iso triangle
def makeTri(sketch, 
    sketch2,
    corners, 
    innerCorners,
    x1, 
    x2, 
    y1, 
    y2, 
    y3,
    flipX):

    innerCorners2 = []
    corners2 = []
# defeat call by reference
    if len(innerCorners) > 0:
        for i in range(0, 3):
            innerCorners2.append(
                App.Vector(innerCorners[i].x, innerCorners[i].y, 0.0))
    
    for i in range(0, 6):
        corners2.append(
            App.Vector(corners[i].x, corners[i].y, 0.0))


    if flipX:
        x3 = x2
        x2 = x1
        x1 = x3
        if len(innerCorners) > 0:
            for i in range(0, 3):
                innerCorners2[i].x *= -1

        for i in range(0, 6):
            corners2[i].x *= -1

# the triangle
    coords = [
        App.Vector(x1, y1, 0.0) + corners2[1],
        App.Vector(x1, y3, 0.0) + corners2[5],
        App.Vector(x2, y2, 0.0) + corners2[3]]
    makePoly(sketch, coords)

    if len(innerCorners) > 0:
# inner subdivisions have to be combined with triangles in a single sketch 
# to make the slicer join them with a continuous filament
        center_x = x1 + (x2 - x1) / 2 / 1.732
        x_offset = (corners2[5].x - corners2[0].x) / 2 + innerCorners2[2].x / 2
        y_offset = (corners2[5].y - corners2[0].y) / 2 + innerCorners2[2].y / 2
        coords = [
            # left
            App.Vector(x1 + corners2[1].x, y2 + innerCorners2[1].y, 0.0),
            # center
            App.Vector(center_x, y2, 0.0) + innerCorners2[1],
            # top right
            App.Vector((x1 + x2) / 2 + x_offset, (y2 + y3) / 2 + y_offset, 0.0) + innerCorners2[1],
            # top left
            App.Vector(x1, y3, 0.0) + corners2[5]
            ]
        makePoly(sketch2, coords)


        coords = [
            # top right
            App.Vector((x1 + x2) / 2 + x_offset, (y2 + y3) / 2 + y_offset, 0.0) + innerCorners2[0],
            # center
            App.Vector(center_x, y2, 0.0) + innerCorners2[0],
            # bottom right
            App.Vector((x1 + x2) / 2 + x_offset, (y1 + y2) / 2 - y_offset, 0.0) + innerCorners2[0],
            # right
            App.Vector(x2, y2, 0.0) + corners2[3]
            ]
        makePoly(sketch2, coords)


        coords = [
            # bottom right
            App.Vector((x1 + x2) / 2 + x_offset, (y1 + y2) / 2 - y_offset, 0.0) + innerCorners2[2],
            # center
            App.Vector(center_x, y2, 0.0) + innerCorners2[2],
            # left
            App.Vector(x1 + corners2[1].x, y2 - innerCorners2[1].y, 0.0),
            # bottom left
            App.Vector(x1, y1, 0.0) + corners2[1]
            ]
        makePoly(sketch2, coords)


# replace or keep existing parallel line
def storeYLine(yLines, start, end):
    total = int(len(yLines) / 2)
    for i in range(0, total):
        avgX = (yLines[i * 2].x + yLines[i * 2 + 1].x) / 2
# got a parallel line
        if (start.x < avgX and end.x > avgX) or \
            (start.x > avgX and end.x < avgX):
            avgY = (yLines[i * 2].y + yLines[i * 2 + 1].y) / 2
            avgY2 = (start.y + end.y) / 2
# replace existing one
            if avgY < avgY2:
                yLines[i * 2] = App.Vector(start.x, start.y, start.z)
                yLines[i * 2 + 1] = App.Vector(end.x, end.y, end.z)
                return
            else:
# discard new one
                return

# create new line
    yLines.append(App.Vector(start.x, start.y, start.z))
    yLines.append(App.Vector(end.x, end.y, end.z))


# replace or keep existing parallel line
def storeXLine(xLines, start, end):
    total = int(len(xLines) / 2)
    for i in range(0, total):
        avgY = (xLines[i * 2].y + xLines[i * 2 + 1].y) / 2
# got a parallel line
        if (start.y < avgY and end.y > avgY) or \
            (start.y > avgY and end.y < avgY):
            avgX = (xLines[i * 2].x + xLines[i * 2 + 1].x) / 2
            avgX2 = (start.x + end.x) / 2
# replace existing one
            if avgX < avgX2:
                xLines[i * 2] = App.Vector(start.x, start.y, start.z)
                xLines[i * 2 + 1] = App.Vector(end.x, end.y, end.z)
                return
            else:
# discard new one
                return

# create new line
    xLines.append(App.Vector(start.x, start.y, start.z))
    xLines.append(App.Vector(end.x, end.y, end.z))


def joinLines(lines):
    total2 = len(lines)
    total = int(len(lines) / 2)

    for i in range(0, total):
        start = lines[i * 2]
        end = lines[i * 2 + 1]
        len1 = math.hypot(start.x - end.x, start.y - end.y)

        #print('i=', i)
# search for existing points other than start & end
        for j in range(0, total2):
            if j != i * 2 and j != i * 2 + 1:
                point = lines[j]
                len2 = math.hypot(point.x - start.x, point.y - start.y)
# join point to start
                if len2 < len1 / 2 and abs(len2) > 0.001:
                    lines.append(point)
                    lines.append(start)
                    
                len2 = math.hypot(point.x - end.x, point.y - end.y)
# join point to end
                if len2 < len1 / 2 and abs(len2) > 0.001:
                    lines.append(point)
                    lines.append(end)



def closeLines(points, useX):
    endPoint = None
    result = []
# get starting point
    for i in range(0, len(points)):
        point = points[i]
        if (not useX and (endPoint is None or point.y < endPoint.y)) or \
            (useX and (endPoint is None or point.x < endPoint.x)):
            endPoint = point
    result.append(endPoint)

# get next point until no next point remanes
    while True:
        nextPoint = None
        for i in range(0, len(points)):
            point = points[i]
            
            if (not useX and \
                (point.y > endPoint.y + 0.001 and \
                (nextPoint is None or point.y < nextPoint.y))) or \
                (useX and \
                (point.x > endPoint.x + 0.001 and \
                (nextPoint is None or point.x < nextPoint.x))):
                nextPoint = point
        
        if nextPoint is not None:
            endPoint = nextPoint
            result.append(nextPoint)
        else:
            break
    return result


# make dividing lines in an isogrid for segmented printing
def makeDivision(extrude, \
    x, \
    y):
# base sketch
    src = extrude.Base
    xDst = doc.addObject('Sketcher::SketchObject','xdivision')
    xDst.Placement = App.Placement(
        App.Vector(0.0, 0.0, SKETCH_Z),
        App.Rotation(App.Vector(0, 0, 1), 0.0))
    yDst = doc.addObject('Sketcher::SketchObject','ydivision')
    yDst.Placement = App.Placement(
        App.Vector(0.0, 0.0, SKETCH_Z),
        App.Rotation(App.Vector(0, 0, 1), 0.0))

    xLines = []
    yLines = []

# search for lines in the base sketch
    total = src.GeometryCount
    for i in range(0, total):
        start = src.getPoint(i, 1)
        end = src.getPoint(i, 2)

# test for diagonals
        if abs(start.x - end.x) >= 1 and \
            abs(start.y - end.y) >= 1:

# test for crossing of y
            #print('makeDivision start.y=', start.y, ' end.y=', end.y, ' y=', y)
            if (start.y > y and end.y < y) or \
                (start.y < y and end.y > y):
                storeYLine(yLines, start, end)
                

# test for crossing of x
            if (start.x > x and end.x < x) or \
                (start.x < x and end.x > x):
                storeXLine(xLines, start, end)

# join lines
    joinLines(xLines)
    joinLines(yLines)
# convert to closed polygons
    xLines = closeLines(xLines, False)
    makePoly(xDst, xLines, False)
    yLines = closeLines(yLines, True)
    makePoly(yDst, yLines, False)

# parallel copy for PLA tape
    xDst2 = None
    if len(xLines) > 1:
        xLines2 = []
        for i in xLines:
            newPoint = App.Vector(i.x + 40.0, i.y, i.z)
            xLines2.append(newPoint)
        xDst2 = doc.addObject('Sketcher::SketchObject','xdivision')
        makePoly(xDst2, xLines2, False)

    yDst2 = None
    if len(yLines) > 1:
        yLines2 = []
        for i in yLines:
            newPoint = App.Vector(i.x, i.y + 30.0, i.z)
            yLines2.append(newPoint)
        yDst2 = doc.addObject('Sketcher::SketchObject','ydivision')
        makePoly(yDst2, yLines2, False)


    return xDst, yDst, xDst2, yDst2




def makeGrid(w, \
    h, \
    x_slices, \
    y_slices, \
    x_borders, \
    y_borders, \
    subdivision_thickness, \
    subdivision_w, \
    triangle_thickness, \
    triangle_w):

    extrudeTris = None
    extrudeSubs = None

# fudge the x borders
    x_borders[0] = x_borders[0] - .6
    x_borders[1] = x_borders[1] - .6

    slice_h = (h - y_borders[0] - y_borders[1]) / y_slices
    slice_w = (w - x_borders[0] - x_borders[1]) / x_slices


# top layer with just triangles
    sketch = doc.addObject('Sketcher::SketchObject','gridsketch')
    sketch.Placement = App.Placement(
        App.Vector(0.0, 0.0, SKETCH_Z),
        App.Rotation(App.Vector(0, 0, 1), 0.0))
    gui.getObject(sketch.Label).Visibility = False
# bottom layer with subdivisions
    sketch2 = doc.addObject('Sketcher::SketchObject','gridsketch')
    sketch2.Placement = App.Placement(
        App.Vector(0.0, 0.0, SKETCH_Z),
        App.Rotation(App.Vector(0, 0, 1), 0.0))
    gui.getObject(sketch2.Label).Visibility = False



# radius of the circle joining 6 ribs
    radius = triangle_w
# offsets of the 6 triangle corners around each center coordinate
# coordinates are counterclockwise
    corners = []
    for i in range(0, 6):
        corner = App.Vector( \
            radius * math.cos(toRad(i * 60)), \
            radius * math.sin(toRad(i * 60)), \
            0.0)
        corners.append(corner)

# offsets of the 3 inner triangle corners
    innerCorners = []
    innerRadius = subdivision_w / 1.732


#    print('makeGrid innerRadius=', innerRadius)
    for i in range(0, 3):
        corner = App.Vector( \
            innerRadius * math.cos(toRad(i * 120)), \
            innerRadius * math.sin(toRad(i * 120)), \
            0.0)
        innerCorners.append(corner)

# bottom & top row
    for x in range(0, x_slices):
        center_x1 = x_borders[0] + x * slice_w
        center_x2 = x_borders[0] + (x + 1) * slice_w
        if (not FLIP and (x % 2) == 0) or \
            (FLIP and (x % 2) == 1):
# bottom left
            center_y1 = y_borders[0]
            center_y2 = y_borders[0] + .5 * slice_h
            coords = [
                App.Vector(center_x1, center_y1, 0.0) + App.Vector(corners[0].x, 0, 0),
                App.Vector(center_x2, center_y1, 0.0) + App.Vector(corners[4].x, 0, 0),
                App.Vector(center_x2, center_y2, 0.0) + corners[4]]
            makePoly(sketch, coords)
            if subdivision_thickness > 0.001:
                makePoly(sketch2, coords)

# top left.  
            center_y2 = y_borders[0] + y_slices * slice_h
            center_y1 = y_borders[0] + (y_slices - .5) * slice_h
            if y_slices - int(y_slices) < 0.25:
# If a fractional row, flip it horizontally
                coords = [
                    App.Vector(center_x1, center_y2, 0.0) + App.Vector(corners[0].x, 0, 0),
                    App.Vector(center_x2, center_y2, 0.0) + App.Vector(corners[4].x, 0, 0),
                    App.Vector(center_x2, center_y1, 0.0) + corners[2]]
            else:
                coords = [
                    App.Vector(center_x2, center_y2, 0.0) + App.Vector(corners[3].x, 0, 0),
                    App.Vector(center_x1, center_y2, 0.0) + App.Vector(corners[1].x, 0, 0),
                    App.Vector(center_x1, center_y1, 0.0) + corners[1]]
            makePoly(sketch, coords)
            if subdivision_thickness > 0.001:
                makePoly(sketch2, coords)
        else:
# bottom right
            center_y1 = y_borders[0]
            center_y2 = y_borders[0] + .5 * slice_h
            coords = [
                App.Vector(center_x2, center_y1, 0.0) + App.Vector(corners[3].x, 0, 0),
                App.Vector(center_x1, center_y1, 0.0) + App.Vector(corners[1].x, 0, 0),
                App.Vector(center_x1, center_y2, 0.0) + corners[5]]
            makePoly(sketch, coords)
            if subdivision_thickness > 0.001:
                makePoly(sketch2, coords)
# top right
            center_y2 = y_borders[0] + y_slices * slice_h
            center_y1 = y_borders[0] + (y_slices - .5) * slice_h
            if y_slices - int(y_slices) < 0.25:
# If a fractional row, flip it horizontally
                coords = [
                    App.Vector(center_x2, center_y2, 0.0) + App.Vector(corners[3].x, 0, 0),
                    App.Vector(center_x1, center_y2, 0.0) + App.Vector(corners[1].x, 0, 0),
                    App.Vector(center_x1, center_y1, 0.0) + corners[1]]
            else:
                coords = [
                    App.Vector(center_x1, center_y2, 0.0) + App.Vector(corners[0].x, 0, 0),
                    App.Vector(center_x2, center_y2, 0.0) + App.Vector(corners[4].x, 0, 0),
                    App.Vector(center_x2, center_y1, 0.0) + corners[2]]
            makePoly(sketch, coords)
            if subdivision_thickness > 0.001:
                makePoly(sketch2, coords)



# middle rows
    for y in range(0, int(y_slices)):
        for x in range(0, int(x_slices)):
            center_y1 = y_borders[0] + y * slice_h
            center_y2 = y_borders[0] + (y + .5) * slice_h
            center_y3 = y_borders[0] + (y + 1) * slice_h
            center_x1 = x_borders[0] + x * slice_w
            center_x2 = x_borders[0] + (x + 1) * slice_w

# debug
#            coords = [
#                App.Vector(center_x1, center_y1, 0.0) + corners[0],
#                App.Vector(center_x1, center_y1, 0.0) + corners[1],
#                App.Vector(center_x1, center_y1, 0.0) + corners[2],
#                App.Vector(center_x1, center_y1, 0.0) + corners[3],
#                App.Vector(center_x1, center_y1, 0.0) + corners[4],
#                App.Vector(center_x1, center_y1, 0.0) + corners[5]
#            ]
#            makePoly(sketch, coords)
#
#            coords = [
#                App.Vector(center_x1, center_y1, 0.0) + innerCorners[0],
#                App.Vector(center_x1, center_y1, 0.0) + innerCorners[1],
#                App.Vector(center_x1, center_y1, 0.0) + innerCorners[2]
#            ]
#            makePoly(sketch, coords)

            if subdivision_thickness < 0.001:
                innerCorners = []

            direction = (not FLIP and (x % 2) == 1) or \
                (FLIP and (x % 2) == 0)
            makeTri(sketch, 
                sketch2,
                corners, 
                innerCorners,
                center_x1, 
                center_x2, 
                center_y1, 
                center_y2, 
                center_y3,
                direction)

            if y < y_slices - 1:
                center_y1 += slice_h / 2
                center_y2 += slice_h / 2
                center_y3 += slice_h / 2

                if subdivision_thickness < 0.001:
                    innerCorners = []
                makeTri(sketch, 
                    sketch2,
                    corners, 
                    innerCorners,
                    center_x1, 
                    center_x2, 
                    center_y1, 
                    center_y2, 
                    center_y3,
                    not direction)




# border
    coords = [
        App.Vector(0.0, 0.0, 0.0),
        App.Vector(0.0, h, 0.0),
        App.Vector(w, h, 0.0),
        App.Vector(w, 0.0, 0.0),
        ]
    makePoly(sketch, coords)
    if subdivision_thickness > 0.001:
        makePoly(sketch2, coords)

# isogrid extrusion
    if True:
        extrudeTris = doc.addObject('Part::Extrusion', 'isogrid')
        extruderDefaults(extrudeTris)
        extrudeTris.Placement = App.Placement(
            App.Vector(0.0, 0.0, 0.0),
            App.Rotation(App.Vector(0,0,1), 0.0))
        extrudeTris.Base = sketch
        extrudeTris.LengthFwd = triangle_thickness

# subdivisions
        if subdivision_thickness > 0.001:
            extrudeSubs = doc.addObject('Part::Extrusion', 'isogrid')
            extruderDefaults(extrudeSubs)
            extrudeSubs.Placement = App.Placement(
                App.Vector(0.0, 0.0, 0.0),
                App.Rotation(App.Vector(0,0,1), 0.0))
            extrudeSubs.Base = sketch2
            extrudeSubs.LengthFwd = subdivision_thickness
        else:
            doc.removeObject(sketch2.Label)

    doc.recompute()
    return extrudeTris, extrudeSubs






def getObj(name):
    for i in doc.Objects:
        if i.Label == name:
            return i
    return None


def vectorsEqual(a, b):
    return a.sub(b).Length < 0.0001

def vectorsNotEqual(a, b):
    return a.sub(b).Length >= 0.0001

def getSegments(ref, closed):
    # get the line segments
    global starts
    global ends
    global total
    global sorted

    starts = []
    ends = []
    total = ref.GeometryCount
    for i in range(0, total):
        start = ref.getPoint(i, 1)
        end = ref.getPoint(i, 2)
    #    print('line ', i, '=', start, ' -> ', end)
        starts.append(start)
        ends.append(end)


    # sort the line segments
    sorted = []
    taken = []
    for i in range(0, total):
        taken.append(False)

    # find line with 1 dangling point
    for i in range(0, total):
        gotStart = False
        gotEnd = False
        for j in range(0, total):
            if j != i:
                if vectorsEqual(starts[i], starts[j]) or \
                    vectorsEqual(starts[i], ends[j]):
                    gotStart = True
                if vectorsEqual(ends[i], starts[j]) or \
                    vectorsEqual(ends[i], ends[j]):
                    gotEnd = True
        if not gotStart or not gotEnd:
            sorted.append(i)
            taken[i] = True
            break

    #print('sorted=', len(sorted))
    if len(sorted) == 0:
        if closed:
            # closed sketch has no starting line
            sorted.append(0)
            taken[0] = True
        else:
            print('*** Got no starting line')

    # get rest of lines
    prev = sorted[0]
    for i in range(1, total):
        for j in range(0, total):
            if not taken[j]:
                if vectorsEqual(starts[prev], starts[j]) or \
                    vectorsEqual(starts[prev], ends[j]) or \
                    vectorsEqual(ends[prev], starts[j]) or \
                    vectorsEqual(ends[prev], ends[j]):
                    #print('prev=', prev, ' j=', j)
                    taken[j] = True
                    sorted.append(j)
                    prev = j
                    break

    #for i in range(0, total):
    #    print('sorted[', i, '] = ', sorted[i])

    # swap line ends
    for i in range(1, total):
        index1 = sorted[i - 1]
        index2 = sorted[i]
        if i == 1:
            if vectorsNotEqual(ends[index1], starts[index2]) and \
                vectorsNotEqual(ends[index1], ends[index2]):
                temp = starts[index1]
                starts[index1] = ends[index1]
                ends[index1] = temp
        if vectorsNotEqual(ends[index1], starts[index2]):
            temp = starts[index2]
            starts[index2] = ends[index2]
            ends[index2] = temp

    for i in range(0, total):
        print('line ', i, '=', starts[sorted[i]], ' -> ', ends[sorted[i]])


# border for a closed polygon
def cropBorder(sketchTitle, \
    backTitle, \
    x, \
    y, \
    z, \
    borderW, \
    borderH, \
    insideDirection, \
    extrudeTris, \
    makeCrop,
    closed):
    ref = getObj(sketchTitle)
    if backTitle is not None:
        back = getObj(backTitle)
    getSegments(ref, closed)

    # create border lines
    borderLine1 = []
    borderLine2 = []

    for i in range(0, total):
        prevStart = starts[sorted[total - 1]]
        prevEnd = ends[sorted[total - 1]]
        start = starts[sorted[i]]
        end = ends[sorted[i]]

        if i == 0:
            prevAngle = math.atan2(prevEnd.y - prevStart.y, prevEnd.x - prevStart.x)
            nextAngle = math.atan2(end.y - start.y, end.x - start.x)
            angle2 = prevAngle + angleChange(prevAngle, nextAngle) / 2.0 + insideDirection
            mag = abs(1.0 / math.cos(angleChange(prevAngle, nextAngle) / 2.0))
            print('1st line prevAngle=', \
                toDeg(prevAngle), \
                'nextAngle=', \
                toDeg(nextAngle), \
                ' angle2=', \
                toDeg(angle2))

            borderLine1.append(start)
            borderLine2.append(addLine(start, angle2, mag * borderW))
        if i == total - 1:
            start2 = starts[sorted[0]]
            end2 = ends[sorted[0]]
            prevAngle = math.atan2(end.y - start.y, end.x - start.x)
            nextAngle = math.atan2(end2.y - start2.y, end2.x - start2.x)
            angle2 = prevAngle + angleChange(prevAngle, nextAngle) / 2.0 + insideDirection
            mag = abs(1.0 / math.cos(angleChange(prevAngle, nextAngle) / 2.0))
            print(' prevAngle=', \
                toDeg(prevAngle), \
                'nextAngle=', \
                toDeg(nextAngle), \
                ' angle2=', \
                toDeg(angle2))
            borderLine1.append(end)
            borderLine2.append(addLine(end, angle2, mag * borderW))
        else:
            start2 = starts[sorted[i + 1]]
            end2 = ends[sorted[i + 1]]
            prevAngle = math.atan2(end.y - start.y, end.x - start.x)
            nextAngle = math.atan2(end2.y - start2.y, end2.x - start2.x)
            angle2 = prevAngle + angleChange(prevAngle, nextAngle) / 2.0 + insideDirection
            mag = abs(1.0 / math.cos(angleChange(prevAngle, nextAngle) / 2.0))
            print(' prevAngle=', \
                toDeg(prevAngle), \
                'nextAngle=', \
                toDeg(nextAngle), \
                ' angle2=', \
                toDeg(angle2))
            borderLine1.append(end)
            borderLine2.append(addLine(end, angle2, mag * borderW))

    border = doc.addObject('Sketcher::SketchObject','border')
#    gui.getObject(border.Label).Visibility = False
    border.Placement = App.Placement(
        App.Vector(x, y, z),
        App.Rotation(App.Vector(0, 0, 1), 0.0))

    if makeCrop:
        crop = doc.addObject('Sketcher::SketchObject','crop')
    #    gui.getObject(crop.Label).Visibility = False
        crop.Placement = App.Placement(
            App.Vector(x, y, z),
            App.Rotation(App.Vector(0, 0, 1), 0.0))

    for i in range(1, total + 1):
        i1 = i - 1
        i2 = i

        border.addGeometry(Part.LineSegment(borderLine1[i1],
                    borderLine1[i2]),
                    False)
        border.addGeometry(Part.LineSegment(borderLine2[i1],
                    borderLine2[i2]),
                    False)
        if makeCrop:
            crop.addGeometry(Part.LineSegment(borderLine1[i1],
                        borderLine1[i2]),
                        False)

# extrude border
    extrudeBorder = doc.addObject('Part::Extrusion', 'border')
    extruderDefaults(extrudeBorder)
    extrudeBorder.Base = border
    extrudeBorder.LengthFwd = borderH

    if extrudeTris is not None:
# create a fusion with an isogrid from a previous operation
        isogridFusion = doc.addObject("Part::MultiFuse","isogrid")
        isogridFusion.Shapes = [extrudeTris, back, extrudeBorder]

# create a cropping tool for the isogrid
        if makeCrop:
            extrudeCrop = doc.addObject('Part::Extrusion', 'crop')
            extruderDefaults(extrudeCrop)
            extrudeCrop.Base = crop
            extrudeCrop.LengthFwd = borderH

# crop the isogrid
            union = doc.addObject("Part::MultiCommon","isogrid")
            union.Shapes = [isogridFusion, extrudeCrop]










