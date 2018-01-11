try:
    import maya.OpenMaya as om
    import maya.cmds as cmds
except ImportError:
    pass


# The Moeller-Trumbore trianle intersection algorithm translated into Python
def mtIntersect(p0, p1, v0, v1, v2):
    """The Moeller-Trumbore triangle intersection algorithm translated into Python

    :param p0: The start of our raygiven as an MVector
    :param p1: A point on our ray to give it direction
    :param v0: Middle vertex world loc
    :param v1: Second vertex world loc
    :param v2: Third vertex world loc
    :type p0: MVector
    :type p1: MVector
    :type v0: MVector
    :type v1: MVector
    :type v2: MVector

    :return: Hit point vector or False
    :rtype: MVector, False

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
    """Detect a mesh collision based on a given mesh, rayOrigin object and rayDirection object

    :param meshDag: The mesh we want to hit
    :param rayOrigin: The start of our ray
    :param rayDirection: Gives our ray a direction
    :type meshDag: MDagPath
    :type rayOrigin: MVector
    :type rayDirection: MVector
    :return: Returns a hit point or False if none
    :rtype: MPoint, False
    """

    try:
        itMesh = om.MItMeshPolygon(meshDag)
        rayVector = (rayDirection - rayOrigin)
    except RuntimeError:
        print "RuntimeError: (kInvalidParameter): Object does not exist"
        return

    # the smaller this value, the more back faces get ignored (0 ignores only back faces, -.25 would also ignore grazes)
    minDotProduct = -.25
    # an array that holds the vertex vectors for each non-culled triangle
    triVertLocs = []

    while not itMesh.isDone():
        # the normal vector for this face
        nVector = om.MVector()

        # check if that polygon is facing away from us using its normal vector
        itMesh.getNormal(nVector, om.MSpace.kWorld)
        if (rayVector * nVector) > minDotProduct:
            # we'll skip this face in our calculations to save time
            itMesh.next()
        else:
            # stores the vertex ids for the triangles that make up each face.
            thisFaceTris = om.MIntArray()
            # an array that holds the points for each triangle in this face (its length is multiples of 3)
            thisFaceTriVerts = om.MPointArray()
            # get the triangles at this face and store them in their proper variables
            itMesh.getTriangles(thisFaceTriVerts, thisFaceTris, om.MSpace.kWorld)
            numTris = len(thisFaceTris) / 3
            i = 0
            while i < numTris:
                # make an array of MVectors for each triangle at this face
                v0 = om.MVector(thisFaceTriVerts[(3 * i) + 0].x,
                                thisFaceTriVerts[(3 * i) + 0].y,
                                thisFaceTriVerts[(3 * i) + 0].z)
                v1 = om.MVector(thisFaceTriVerts[(3 * i) + 1].x,
                                thisFaceTriVerts[(3 * i) + 1].y,
                                thisFaceTriVerts[(3 * i) + 1].z)
                v2 = om.MVector(thisFaceTriVerts[(3 * i) + 2].x,
                                thisFaceTriVerts[(3 * i) + 2].y,
                                thisFaceTriVerts[(3 * i) + 2].z)
                triVertLocs.append([v0, v1, v2])
                i += 1
            itMesh.next()

    hitList = []
    for tri in triVertLocs:
        # get the points for each triangle as an array and send them to our mtIntersect function to see if they hit
        hit = mtIntersect(rayOrigin, rayDirection, tri[0], tri[1], tri[2])

        # if the function returned something other than False:
        if hit:
            # append that hit to our hitList array for later evaluation
            hitList.append(hit)
    # go through each point in our list and find the one with the least distance from our ray origin

    closestLen = float("inf")
    if hitList:
        closestHitPoint = hitList[0]
        for hitPoint in hitList:
            rayHitDist = (rayOrigin - hitPoint).length()
            # using the ray origin as the basis for our distance will give us intersections when p1 is inside the mesh
            if rayHitDist < closestLen:
                # compare distances. We're trying to find the closest one (again, to avoid back faces)
                closestHitPoint = hitPoint

        # make a sphere marker just for fun :]
        s = cmds.polySphere(subdivisionsAxis=10, subdivisionsHeight=10, radius=.1, ch=0)
        cmds.xform(s, t=closestHitPoint, ws=1)
        return closestHitPoint
    else:
        return False


"""
import ApiScripts.bm_rayMeshIntersect as rmi
import maya.OpenMaya as om
import time

t = time.time()
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
print time.time() - t
"""
