import maya.OpenMaya as om
import maya.cmds as cmds


# The Moeller-Trumbore trianle intersection algorithm translated into Python
def mtIntersect(p0, p1, v0, v1, v2):
    """
    :param p0: MVector(): The start of our raygiven as an MVector
    :param p1: MVector(): A point on our ray to give it direction
    :param v0: MVector(): Middle vertex world loc
    :param v1: MVector(): Second vertex world loc
    :param v2: MVector(): Third vertex world loc
    :return: MVector(): Hit point vector or False if none
    """
    # The Moeller Trumbore triangle intersection method
    # Translated from c++ from https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

    # epsilon is the threshold used to determine if a vector is parallel to the plane or not
    eps = .0000001
    rayVector = p1 - p0

    # the edges of our triangle from v0
    edge1 = v1 - v0
    edge2 = v2 - v0

    h = rayVector ^ edge2
    a = edge1 * h

    if -1 * eps < a < eps:
        return False

    f = 1.0 / a
    s = p0 - v0
    u = f * (s * h)

    if u < 0.0 or u > 1.0:
        return False

    q = s ^ edge1
    v = f * (rayVector * q)

    if v < 0.0 or v > 1.0:
        return False

    t = f * (edge2 * q)
    if t > eps:
        outPoint = p0 + rayVector * t
        return outPoint
    else:
        return False


# Main method to detect a mesh collision based on a given mesh, rayOrigin object and rayDirection object
def rayMeshIntersect(meshDag, rayOrigin, rayDirection):
    """
    :param meshDag: MDagPath() to the mesh we want to hit
    :param rayOrigin: MVector() represents the start of our ray
    :param rayDirection: MVector() gives our ray a direction
    :return: MPoint() if there is a hit or False if not.
    """

    try:
        fnMesh = om.MFnMesh(meshDag)
        rayVector = (rayDirection - rayOrigin)
    except RuntimeError:
        print "RuntimeError: (kInvalidParameter): Object does not exist"
        return

    # empty MIntArray to hold the triCounts (# of triangles per polygon index)
    triCountsArray = om.MIntArray()
    # empty MIntArray to hold the vert ids for each triangle
    triVertsArray = om.MIntArray()
    # get our mesh's triangles
    fnMesh.getTriangles(triCountsArray, triVertsArray)

    # iterate through every polygon in our mesh and store the triangle vert locations
    triVertLocs = []
    # the index in our triangleVetsArray that we'll use to iterate through it
    vertArrayIndex = 0

    # for every polygon in our mesh:
    for i in range(fnMesh.numPolygons()):
        # check if that polygon is facing away from us using its normal vector
        nVector = om.MVector()
        fnMesh.getPolygonNormal(i, nVector, om.MSpace.kWorld)
        # if the dot product of these two vectors is greater than zero then that face is facing away from us
        if (rayVector * nVector) > 0:
            # if that's the case then we're going to skip this face and it's iteration through the next for loop
            for j in range(triCountsArray[i]):
                # properly increase our vertArrayIndex according to the number of triangles we're skipping
                vertArrayIndex += 3
        else:
            for k in range(triCountsArray[i]):
                # declare this triangle's points
                v0 = om.MPoint()
                v1 = om.MPoint()
                v2 = om.MPoint()
                # get the location of each point in world space
                fnMesh.getPoint(triVertsArray[vertArrayIndex], v0, om.MSpace.kWorld)
                vertArrayIndex += 1
                fnMesh.getPoint(triVertsArray[vertArrayIndex], v1, om.MSpace.kWorld)
                vertArrayIndex += 1
                fnMesh.getPoint(triVertsArray[vertArrayIndex], v2, om.MSpace.kWorld)
                vertArrayIndex += 1
                # append an array of those points to our triVertLocs array
                triVertLocs.append([v0, v1, v2])

    hitList = []
    for tri in triVertLocs:
        # get the points for each triangle as an array and send them to our mtIntersect function to see if they hit
        v0 = om.MVector(tri[0][0], tri[0][1], tri[0][2])
        v1 = om.MVector(tri[1][0], tri[1][1], tri[1][2])
        v2 = om.MVector(tri[2][0], tri[2][1], tri[2][2])
        hit = mtIntersect(rayOrigin, rayDirection, v0, v1, v2)

        # if the function returned something other than False:
        if hit:
            # append that hit to our hitList array for later evaluation
            hitList.append(hit)
    # go through each point in our list and find the one with the least distance from our ray origin
    closestLen = float("inf")
    if hitList:
        closestHitPoint = hitList[0]
        for hitPoint in hitList:
            # using the ray origin as the basis for our distance will give us intersections when p1 is inside the mesh
            if (rayOrigin - hitPoint).length() < closestLen:
                # compare distances. We're trying to find the closest one (again, to avoid back faces)
                closestLen = (rayOrigin - hitPoint).length()
                closestHitPoint = hitPoint

        # make a sphere marker just for fun :]
        s = cmds.polySphere(subdivisionsAxis=10, subdivisionsHeight=10, radius=.1, ch=0)
        cmds.xform(s, t=[closestHitPoint.x, closestHitPoint.y, closestHitPoint.z], ws=1)
        cmds.select(cl=1)

"""
import ApiScripts.bm_rayMeshIntersect as rmi

reload(rmi)

sel = om.MSelectionList()
sel.add("pSphere1")
sel.add("joint1")
sel.add("joint2")

meshDag = om.MDagPath()
sel.getDagPath(0, meshDag)
p0Dag = om.MDagPath()
sel.getDagPath(1, p0Dag)
p1Dag = om.MDagPath()
sel.getDagPath(2, p1Dag)

p0Vec = om.MFnTransform(p0Dag).getTranslation(om.MSpace.kWorld)
p1Vec = om.MFnTransform(p1Dag).getTranslation(om.MSpace.kWorld)

rmi.rayMeshIntersect(meshDag, p0Vec, p1Vec)
"""