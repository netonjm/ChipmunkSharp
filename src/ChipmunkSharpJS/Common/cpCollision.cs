/* Copyright (c) 2007 Scott Lembcke ported by Jose Medrano (@netonjm)
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */
using ChipmunkSharp;
using ChipmunkSharp.Shapes;
using System.Linq;
// // typedef int (*collisionFunc)(cpShape , cpShape , cpContact );
using System;
using System.Collections.Generic;

namespace ChipmunkSharp
{



    //public delegate int CollisionFunc(cpShape a, cpShape b, int id, List<ContactPoint> arr);

    public class cpCollision
    {
        #region CONSTANTS

        public const int DRAW_ALL = 0;
        public const int ENABLE_CACHING = 1;

        public const int DRAW_GJK = (0 | DRAW_ALL);
        public const int DRAW_EPA = (0 | DRAW_ALL);
        public const int DRAW_CLOSEST = (0 | DRAW_ALL);
        public const int DRAW_CLIP = (0 | DRAW_ALL);

        public const int MAX_GJK_ITERATIONS = 30;
        public const int MAX_EPA_ITERATIONS = 30;
        public const int WARN_GJK_ITERATIONS = 20;
        public const int WARN_EPA_ITERATIONS = 20;

        #endregion

        #region PROPERTIES

        public int a;

        int numContacts = 0;

        public cpVect r2 { get; set; }

        public cpVect r1 { get; set; }

        public float bias { get; set; }

        public float bounce { get; set; }

        public float tMass { get; set; }

        public float nMass { get; set; }

        public object hash { get; set; }

        public float jBias { get; set; }

        public float jtAcc { get; set; }

        public float jnAcc { get; set; }

        #endregion



        //MARK: Collision Functions


        #region  Collision Functions

        public static List<ContactPoint> circle2circle(cpCircleShape circ1, cpCircleShape circ2)
        {
            return circle2circleQuery(circ1.tc, circ2.tc, circ1.r, circ2.r);
            //return contact != null ? contact : null;
        }

        public static List<ContactPoint> circle2circleQuery(cpVect p1, cpVect p2, float r1, float r2)
        {
            var mindist = r1 + r2;
            var delta = cpVect.cpvsub(p2, p1);
            var distsq = cpVect.cpvlengthsq(delta);

            if (distsq >= mindist * mindist)
                return new List<ContactPoint>();

            var dist = cp.cpfsqrt(distsq);

            // Allocate and initialize the contact.
            return new List<ContactPoint>() {  new ContactPoint(
                 cpVect.cpvadd(p1, cpVect.cpvmult(delta, 0.5f + (r1 - 0.5f * mindist) / (dist > 0 ? dist : cp.Infinity))),
                (dist > 0 ? cpVect.cpvmult(delta, 1 / dist) : new cpVect(1, 0)),
                dist - mindist,
                "0"
            )};

        }

        public static List<ContactPoint> circle2segment(cpCircleShape circleShape, cpSegmentShape segmentShape)
        {

            var seg_a = segmentShape.ta;
            var seg_b = segmentShape.tb;
            var center = circleShape.tc;

            var seg_delta = cpVect.cpvsub(seg_b, seg_a);
            var closest_t = cp.cpfclamp01(cpVect.cpvdot(seg_delta, cpVect.cpvsub(center, seg_a)) / cpVect.cpvlengthsq(seg_delta));
            var closest = cpVect.cpvadd(seg_a, cpVect.cpvmult(seg_delta, closest_t));

            var contact = circle2circleQuery(center, closest, circleShape.r, segmentShape.r);
            if (contact != null)
            {
                List<ContactPoint> dev = new List<ContactPoint>();

                foreach (var item in contact)
                {
                    var n = item.n;

                    // Reject endcap collisions if tangents are provided.
                    if ((closest_t == 0 && cpVect.cpvdot(n, segmentShape.a_tangent) < 0) ||
                        (closest_t == 1 && cpVect.cpvdot(n, segmentShape.b_tangent) < 0))
                    { }
                    else
                        dev.Add(item);
                }

                if (dev.Count > 0)
                    return dev;

            }

            return new List<ContactPoint>();

        }

        public static List<ContactPoint> circle2poly(cpCircleShape circ, cpPolyShape poly)
        {
            var planes = poly.tPlanes;

            var mini = 0;
            var min = cpVect.cpvdot(planes[0].n, circ.tc) - planes[0].d - circ.r;
            for (var i = 0; i < planes.Length; i++)
            {
                var dist = cpVect.cpvdot(planes[i].n, circ.tc) - planes[i].d - circ.r;
                if (dist > 0)
                {
                    return new List<ContactPoint>();
                }
                else if (dist > min)
                {
                    min = dist;
                    mini = i;
                }
            }

            var n = planes[mini].n;

            var verts = poly.tVerts;
            var len = verts.Length;
            var mini2 = mini << 1;

            //var a = poly.tVerts[mini];
            //var b = poly.tVerts[(mini + 1)%poly.tVerts.length];
            var ax = verts[mini2];
            var ay = verts[mini2 + 1];
            var bx = verts[(mini2 + 2) % len];
            var by = verts[(mini2 + 3) % len];

            var dta = cpVect.cpvcross2(n.x, n.y, ax, ay);
            var dtb = cpVect.cpvcross2(n.x, n.y, bx, by);
            var dt = cpVect.cpvcross(n, circ.tc);

            if (dt < dtb)
            {
                return circle2circleQuery(circ.tc, new cpVect(bx, by), circ.r, 0);
            }
            else if (dt < dta)
            {
                return new List<ContactPoint>() {  new ContactPoint(
			cpVect.cpvsub(circ.tc, cpVect.cpvmult(n, circ.r + min/2)),
			cpVect.cpvneg(n),
			min,
			"0"
		)};
            }
            else
            {
                return circle2circleQuery(circ.tc, new cpVect(ax, ay), circ.r, 0);
            }
        }

        public static List<ContactPoint> seg2poly(cpSegmentShape seg, cpPolyShape poly)
        {

            var arr = new List<ContactPoint>();

            var planes = poly.tPlanes;
            var numVerts = planes.Length;

            var segD = cpVect.cpvdot(seg.tn, seg.ta);
            var minNorm = poly.valueOnAxis(seg.tn, segD) - seg.r;
            var minNeg = poly.valueOnAxis(cpVect.cpvneg(seg.tn), -segD) - seg.r;
            if (minNeg > 0 || minNorm > 0)
                return new List<ContactPoint>();

            var mini = 0;
            var poly_min = cp.segValueOnAxis(seg, planes[0].n, planes[0].d);
            if (poly_min > 0)
                return new List<ContactPoint>();

            for (var i = 0; i < numVerts; i++)
            {
                var dist = cp.segValueOnAxis(seg, planes[i].n, planes[i].d);
                if (dist > 0)
                {
                    return new List<ContactPoint>();
                }
                else if (dist > poly_min)
                {
                    poly_min = dist;
                    mini = i;
                }
            }

            var poly_n = cpVect.cpvneg(planes[mini].n);

            var va = cpVect.cpvadd(seg.ta, cpVect.cpvmult(poly_n, seg.r));
            var vb = cpVect.cpvadd(seg.tb, cpVect.cpvmult(poly_n, seg.r));
            if (poly.containsVert(va.x, va.y))
                arr.Add(new ContactPoint(va, poly_n, poly_min, cp.hashPair(seg.hashid, "0")));
            if (poly.containsVert(vb.x, vb.y))
                arr.Add(new ContactPoint(vb, poly_n, poly_min, cp.hashPair(seg.hashid, "1")));

            // Floating point precision problems here.
            // This will have to do for now.
            //	poly_min -= cp_collision_slop; // TODO is this needed anymore?

            if (minNorm >= poly_min || minNeg >= poly_min)
            {
                if (minNorm > minNeg)
                    cp.findPointsBehindSeg(arr, seg, poly, minNorm, 1);
                else
                    cp.findPointsBehindSeg(arr, seg, poly, minNeg, -1);
            }

            // If no other collision points are found, try colliding endpoints.
            if (arr.Count == 0)
            {
                var mini2 = mini * 2;
                var verts = poly.tVerts;

                var poly_a = new cpVect(verts[mini2], verts[mini2 + 1]);

                List<ContactPoint> con;
                if ((con = circle2circleQuery(seg.ta, poly_a, seg.r, 0f)) != null) return con;
                if ((con = circle2circleQuery(seg.tb, poly_a, seg.r, 0f)) != null) return con;

                var len = numVerts * 2;
                var poly_b = new cpVect(verts[(mini2 + 2) % len], verts[(mini2 + 3) % len]);
                if ((con = circle2circleQuery(seg.ta, poly_b, seg.r, 0f)) != null) return con;
                if ((con = circle2circleQuery(seg.tb, poly_b, seg.r, 0f)) != null) return con;
            }

            //	console.log(poly.tVerts, poly.tPlanes);
            //	console.log('seg2poly', arr);
            return arr;
        }



        public static List<ContactPoint> Poly2Poly(cpPolyShape poly1, cpPolyShape poly2)
        {
            float mini1 = cp.findMSA(poly2, poly1.tPlanes);
            if (mini1 == -1) return new List<ContactPoint>();
            float min1 = cp.last_MSA_min;

            float mini2 = cp.findMSA(poly1, poly2.tPlanes);
            if (mini2 == -1) return new List<ContactPoint>();
            float min2 = cp.last_MSA_min;

            // There is overlap, find the penetrating verts
            if (min1 > min2)
                return cp.findVerts(poly1, poly2, poly1.tPlanes[(int)mini1].n, min1);
            else
                return cp.findVerts(poly1, poly2, cpVect.cpvneg(poly2.tPlanes[(int)mini2].n), min2);

            // return new List<ContactPoint>();
        }




        #endregion

    }



}
//#region NOT USED

// Add contact points for circle to circle collisions.
// Used by several collision tests.
// TODO should accept hash parameter
//public static int circle2circleQuery(cpVect p1, cpVect p2, float r1, float r2, int hash, cpContact con)
//{
//    float mindist = r1 + r2;
//    cpVect delta = cpVect.Sub(p2, p1);
//    float distsq = delta.LengthSQ;
//    if (distsq >= mindist * mindist)
//        return 0;

//    float dist = cpEnvironment.cpfsqrt(distsq);
//    cpVect n = (dist == 0 ? cpVect.cpvmult(delta, 1.0f / dist) : new cpVect(1.0f, 0.0f));

//    con.Init(cpVect.cpvlerp(p1, p2, r1 / (r1 + r2)), n, dist - mindist, hash);
//    return 1;
//}


//// Collide circle shapes.
//public static int circle2circle(cpShape shape1, cpShape shape2, cpContact arr)
//{
//    throw new NotImplementedException("Not implemented");
//    //cpCircleShape circ1 = (cpCircleShape)shape1; //TODO
//    //cpCircleShape circ2 = (cpCircleShape)shape2;

//    //return circle2circleQuery(circ1.tc, circ2.tc, circ1.r, circ2.r, arr);
//}

//public static int circle2segment(cpCircleShape circleShape, cpSegmentShape segmentShape, List<cpContact> con)
//{
//    cpVect seg_a = segmentShape.ta;
//    cpVect seg_b = segmentShape.tb;
//    cpVect center = circleShape.tc;

//    cpVect seg_delta = cpVect.Sub(seg_b, seg_a);
//    float closest_t = cpEnvironment.cpfclamp01(cpVect.Dot(seg_delta, cpVect.Sub(center, seg_a)) / seg_delta.LengthSQ);
//    cpVect closest = cpVect.Add(seg_a, cpVect.Multiply(seg_delta, closest_t));

//    if (circle2circleQuery(center, closest, circleShape.r, segmentShape.r, con) != null)
//    {
//        cpVect n = con[0].normal;

//        // Reject endcap collisions if tangents are provided.
//        if (
//            (closest_t == 0.0f && cpVect.Dot(n, segmentShape.a_tangent) < 0.0) ||
//            (closest_t == 1.0f && cpVect.Dot(n, segmentShape.b_tangent) < 0.0)
//        ) return 0;

//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}

//// Helper function for working with contact buffers
//// This used to malloc/realloc memory on the fly but was repurposed.
//public static cpContact nextContactPoint(List<cpContact> arr, int numPtr)
//{


//    int index = numPtr;

//    if (index < cpArbiter.CP_MAX_CONTACTS_PER_ARBITER)
//    {
//        numPtr = index + 1;
//        return arr[index];
//    }
//    else
//    {
//        return arr[cpArbiter.CP_MAX_CONTACTS_PER_ARBITER - 1];
//    }
//}


//// Add contacts for probably penetrating vertexes.
//// This handles the degenerate case where an overlap was detected, but no vertexes fall inside
//// the opposing polygon. (like a star of david)
//public static int findVertsFallback(List<cpContact> arr, cpPolyShape poly1, cpPolyShape poly2, cpVect n, double dist)
//{

//    throw new NotImplementedException("Not implemented");
//    //int num = 0;

//    //for (int i = 0; i < poly1.numVerts; i++)
//    //{
//    //    cpVect v = poly1.tVerts[i];
//    //    if (poly2.ContainsVertPartial(v, n.Neg()))
//    //        cpContactInit(nextContactPoint(arr, num), v, n, dist, cpEnvironment.CP_HASH_PAIR(poly1.shape.hashid, i));
//    //}

//    //for (int i = 0; i < poly2.numVerts; i++)
//    //{
//    //    cpVect v = poly2.tVerts[i];
//    //    if (poly1.ContainsVertPartial(v, n))
//    //        cpContactInit(nextContactPoint(arr, num), v, n, dist, cpEnvironment.CP_HASH_PAIR(poly2.shape.hashid, i));
//    //}

//    //return num;
//}

//// Add contacts for penetrating vertexes.
//public static int findVerts(List<cpContact> arr, cpPolyShape poly1, cpPolyShape poly2, cpVect n, double dist)
//{
//    throw new NotImplementedException("Not implemented");

//    //int num = 0;

//    //for (int i = 0; i < poly1.numVerts; i++)
//    //{
//    //    cpVect v = poly1.tVerts[i];
//    //    if (poly2.ContainsVert(v))
//    //        cpContactInit(nextContactPoint(arr, num), v, n, dist, cpEnvironment.CP_HASH_PAIR(poly1.shape.hashid, i));
//    //}

//    //for (int i = 0; i < poly2.numVerts; i++)
//    //{
//    //    cpVect v = poly2.tVerts[i];
//    //    if (cpPolyShapeContainsVert(poly1, v))
//    //        cpContactInit(nextContactPoint(arr, num), v, n, dist, cpEnvironment.CP_HASH_PAIR(poly2.shape.hashid, i));
//    //}

//    //return (num != null) ? num : findVertsFallback(arr, poly1, poly2, n, dist);
//}

//// Collide poly shapes together.
//public static int poly2poly(cpShape shape1, cpShape shape2, cpContact arr)
//{
//    throw new NotImplementedException("Not implemented");
//    //cpPolyShape poly1 = (cpPolyShape)shape1;
//    //cpPolyShape poly2 = (cpPolyShape)shape2;

//    //double min1;
//    //int mini1 = poly2.FindMSA(poly1.tPlanes, poly1.numVerts, min1);
//    //if (mini1 == -1) return 0;

//    //double min2;
//    //int mini2 = poly1.FindMSA(poly2.tPlanes, poly2.numVerts, min2);
//    //if (mini2 == -1) return 0;

//    //// There is overlap, find the penetrating verts
//    //if (min1 > min2)
//    //    return findVerts(arr, poly1, poly2, poly1.tPlanes[mini1].n, min1);
//    //else
//    //    return findVerts(arr, poly1, poly2, poly2.tPlanes[mini2].n.Neg(), min2);
//}

//// Like cpPolyValueOnAxis(), but for segments.
//public static double segValueOnAxis(cpSegmentShape seg, cpVect n, double d)
//{
//    double a = cpVect.Dot(n, seg.ta) - seg.r;
//    double b = cpVect.Dot(n, seg.tb) - seg.r;
//    return System.Math.Min(a, b) - d;
//}

//// Identify vertexes that have penetrated the segment.
//public static void findPointsBehindSeg(cpContact arr, int num, cpSegmentShape seg, cpPolyShape poly, double pDist, double coef)
//{
//    throw new NotImplementedException("Not implemented");

//    //double dta = cpVect.CrossProduct(seg.tn, seg.ta);
//    //double dtb = cpVect.CrossProduct(seg.tn, seg.tb);
//    //cpVect n = cpVect.Multiply(seg.tn, coef);

//    //for (int i = 0; i < poly.numVerts; i++)
//    //{
//    //    cpVect v = poly.tVerts[i];
//    //    if (cpVect.Dot(v, n) < cpVect.Dot(seg.tn, seg.ta) * coef + seg.r)
//    //    {
//    //        double dt = cpVect.CrossProduct(seg.tn, v);
//    //        if (dta >= dt && dt >= dtb)
//    //        {
//    //            cpContactInit(nextContactPoint(arr, num), v, n, pDist, cpEnvironment.CP_HASH_PAIR(poly.shape.hashid, i));
//    //        }
//    //    }
//    //}
//}

//// This one is complicated and gross. Just don't go there...
//// TODO: Comment me!
//public static int seg2poly(cpShape shape1, cpShape shape2, List<cpContact> arr)
//{
//    throw new NotImplementedException("Not implemented");

//    //cpSegmentShape seg = (cpSegmentShape)shape1;
//    //cpPolyShape poly = (cpPolyShape)shape2;
//    //List<cpSplittingPlane> planes = poly.tPlanes;

//    //double segD = cpVect.Dot(seg.tn, seg.ta);
//    //double minNorm = poly.ValueOnAxis(seg.tn, segD) - seg.r;
//    //double minNeg = poly.ValueOnAxis(seg.tn.Neg(), -segD) - seg.r;
//    //if (minNeg > 0.0f || minNorm > 0.0f) return 0;

//    //int mini = 0;
//    //double poly_min = segValueOnAxis(seg, planes[0].n, planes[0].d);
//    //if (poly_min > 0.0f) return 0;
//    //for (int i = 0; i < poly.numVerts; i++)
//    //{
//    //    double dist = segValueOnAxis(seg, planes[i].n, planes[i].d);
//    //    if (dist > 0.0f)
//    //    {
//    //        return 0;
//    //    }
//    //    else if (dist > poly_min)
//    //    {
//    //        poly_min = dist;
//    //        mini = i;
//    //    }
//    //}

//    //int num = 0;

//    //cpVect poly_n = planes[mini].n.Neg(); // cpvneg();

//    //cpVect va = cpVect.Add(seg.ta, cpVect.Multiply(poly_n, seg.r));
//    //cpVect vb = cpVect.Add(seg.tb, cpVect.Multiply(poly_n, seg.r));

//    //if (poly.ContainsVert(va))
//    //    cpContactInit(nextContactPoint(arr, num), va, poly_n, poly_min, cpEnvironment.CP_HASH_PAIR(seg.shape.hashid, 0));
//    //if (poly.ContainsVert(vb))
//    //    cpContactInit(nextContactPoint(arr, num), vb, poly_n, poly_min, cpEnvironment.CP_HASH_PAIR(seg.shape.hashid, 1));

//    //// doubleing point precision problems here.
//    //// This will have to do for now.
//    ////	poly_min -= cp_collision_slop; // TODO is this needed anymore?

//    //if (minNorm >= poly_min || minNeg >= poly_min)
//    //{
//    //    if (minNorm > minNeg)
//    //        findPointsBehindSeg(arr, num, seg, poly, minNorm, 1.0f);
//    //    else
//    //        findPointsBehindSeg(arr, num, seg, poly, minNeg, -1.0f);
//    //}

//    //// If no other collision points are found, try colliding endpoints.
//    //if (num == 0)
//    //{
//    //    cpVect poly_a = poly.tVerts[mini];
//    //    cpVect poly_b = poly.tVerts[(mini + 1) % poly.numVerts];

//    //    //TODO: Revise
//    //    if (circle2circleQuery(seg.ta, poly_a, seg.r, 0.0f) == 0) return 1;
//    //    if (circle2circleQuery(seg.tb, poly_a, seg.r, 0.0f) == 0) return 1;
//    //    if (circle2circleQuery(seg.ta, poly_b, seg.r, 0.0f) == 0) return 1;
//    //    if (circle2circleQuery(seg.tb, poly_b, seg.r, 0.0f) == 0) return 1;
//    //}

//    //return num;
//}

//// This one is less gross, but still gross.
//// TODO: Comment me!
//public static int circle2poly(cpShape shape1, cpShape shape2, cpContact con)
//{

//    throw new NotImplementedException("Not implemented");
//    //cpCircleShape circ = (cpCircleShape)shape1;
//    //cpPolyShape poly = (cpPolyShape)shape2;
//    //List<cpSplittingPlane> planes = poly.tPlanes;

//    //int mini = 0;
//    //float min = planes[0].Compare(circ.tc) - circ.r;
//    //for (int i = 0; i < poly.numVerts; i++)
//    //{
//    //    double dist = planes[i].Compare(circ.tc) - circ.r;
//    //    if (dist > 0.0f)
//    //    {
//    //        return 0;
//    //    }
//    //    else if (dist > min)
//    //    {
//    //        min = dist;
//    //        mini = i;
//    //    }
//    //}

//    //cpVect n = planes[mini].n;
//    //cpVect a = poly.tVerts[mini];
//    //cpVect b = poly.tVerts[(mini + 1) % poly.numVerts];
//    //float dta = cpVect.CrossProduct(n, a);
//    //float dtb = cpVect.CrossProduct(n, b);
//    //float dt = cpVect.CrossProduct(n, circ.tc);

//    //if (dt < dtb)
//    //{
//    //    return circle2circleQuery(circ.tc, b, circ.r, 0.0f, con);
//    //}
//    //else if (dt < dta)
//    //{
//    //    new cpContact(
//    //          con,
//    //          circ.tc.Sub(n.Multiply(circ.r + min / 2.0f)),
//    //         n.Neg(),
//    //          min,
//    //          0
//    //      );

//    //    return 1;
//    //}
//    //else
//    //{
//    //    return circle2circleQuery(circ.tc, a, circ.r, 0.0f, con);
//    //}
//}

//// Submitted by LegoCyclon
//public static int seg2seg(cpShape shape1, cpShape shape2, cpContact con)
//{

//    throw new NotImplementedException("Not implemented");
//    //cpSegmentShape seg1 = (cpSegmentShape)shape1;
//    //cpSegmentShape seg2 = (cpSegmentShape)shape2;

//    //cpVect v1 = cpVect.Sub(seg1.tb, seg1.ta);
//    //cpVect v2 = cpVect.Sub(seg2.tb, seg2.ta);

//    //double v1lsq = v1.LengthSQ;// cpVect.LengthSQ(v1);
//    //double v2lsq = v2.LengthSQ;// cpVect.LengthSQ(v2);
//    //// project seg2 onto seg1
//    //cpVect p1a = seg2.ta.Sub(seg1.ta).Project(v1);  // cpVect.cpvproject(   cpVect.Sub(seg2.ta, seg1.ta), v1);
//    //cpVect p1b = seg2.tb.Sub(seg1.ta).Project(v1); // cpVect.cpvproject(cpVect.Sub(seg2.tb, seg1.ta), v1);
//    //// project seg1 onto seg2
//    //cpVect p2a = seg2.ta.Sub(seg1.ta).Project(v1); //cpVect.cpvproject(cpVect.Sub(seg1.ta, seg2.ta), v2);
//    //cpVect p2b = seg2.ta.Sub(seg1.ta).Project(v1); //cpVect.cpvproject(cpVect.Sub(seg1.tb, seg2.ta), v2);

//    //// clamp projections to segment endcaps
//    //if (cpVect.Dot(p1a, v1) < 0.0f)
//    //    p1a = cpVect.ZERO;
//    //else if (cpVect.Dot(p1a, v1) > 0.0f && p1a.LengthSQ > v1lsq)
//    //    p1a = v1;
//    //if (cpVect.Dot(p1b, v1) < 0.0f)
//    //    p1b = cpVect.ZERO;
//    //else if (cpVect.Dot(p1b, v1) > 0.0f && p1b.LengthSQ > v1lsq)
//    //    p1b = v1;
//    //if (cpVect.Dot(p2a, v2) < 0.0f)
//    //    p2a = cpVect.ZERO;
//    //else if (cpVect.Dot(p2a, v2) > 0.0f && p2a.LengthSQ > v2lsq)
//    //    p2a = v2;
//    //if (cpVect.Dot(p2b, v2) < 0.0f)
//    //    p2b = cpVect.ZERO;
//    //else if (cpVect.Dot(p2b, v2) > 0.0f && p2b.LengthSQ > v2lsq)
//    //    p2b = v2;

//    //p1a = cpVect.Add(p1a, seg1.ta);
//    //p1b = cpVect.Add(p1b, seg1.ta);
//    //p2a = cpVect.Add(p2a, seg2.ta);
//    //p2b = cpVect.Add(p2b, seg2.ta);

//    //int num = 0;

//    //if (circle2circleQuery(p1a, p2a, seg1.r, seg2.r, nextContactPoint(con, num)) != 0)
//    //    --num;

//    //if (circle2circleQuery(p1b, p2b, seg1.r, seg2.r, nextContactPoint(con, num)) != 0)
//    //    --num;

//    //if (circle2circleQuery(p1a, p2b, seg1.r, seg2.r, nextContactPoint(con, num)) != 0)
//    //    --num;

//    //if (circle2circleQuery(p1b, p2a, seg1.r, seg2.r, nextContactPoint(con, num)) != 0)
//    //    --num;

//    //return num;
//}

//public static int cpCollideShapes(cpShape a, cpShape b, cpContact arr)
//{
//    // Their shape types must be in order.
//    // cpAssertSoft(a.klass.type <= b.klass.type, "Collision shapes passed to cpCollideShapes() are not sorted.");
//    throw new NotImplementedException("Not implemented");
//    //collisionFunc cfunc = colfuncs[a.type + b.type * CP_NUM_SHAPES];
//    //return (cfunc) ? cfunc(a, b, arr) : 0;
//}

//public static object cpCollisionBeginFunc { get; set; }
//#endregion

////Add contact points for circle to circle collisions.
////Used by several collision tests.
////TODO should accept hash parameter
//public static int CircleToCircleQuery(cpVect p1, cpVect p2, float r1, float r2, string hash, List<ContactPoint> con)
//{
//    float mindist = r1 + r2;
//    cpVect delta = cpVect.cpvsub(p2, p1);
//    float distsq = cpVect.cpvlengthsq(delta);

//    if (distsq < mindist * mindist)
//    {

//        float dist = cpEnvironment.cpfsqrt(distsq);
//        cpVect n = (dist == 0 ? cpVect.cpvmult(delta, 1.0f / dist) : new cpVect(1.0f, 0.0f));
//        con.Add(new ContactPoint(cpVect.cpvlerp(p1, p2, r1 / (r1 + r2)), n, dist - mindist, hash));


//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}

// Collide circle shapes.
//        public static int CircleToCircle(cpCircleShape c1, cpCircleShape c2, string id, List<ContactPoint> arr)
//        {
//            return CircleToCircleQuery(c1.tc, c2.tc, c1.r, c2.r, "0", arr);
//        }

//        public static int CircleToSegment(cpCircleShape circleShape, cpSegmentShape segmentShape, string id, List<ContactPoint> con)
//        {

//            cpVect seg_a = segmentShape.ta;
//            cpVect seg_b = segmentShape.tb;
//            cpVect center = circleShape.tc;

//            cpVect seg_delta = cpVect.cpvsub(seg_b, seg_a);
//            float closest_t = cpEnvironment.cpfclamp01(cpVect.cpvdot(seg_delta, cpVect.cpvsub(center, seg_a)) / cpVect.cpvlengthsq(seg_delta));
//            cpVect closest = cpVect.cpvadd(seg_a, cpVect.cpvmult(seg_delta, closest_t));

//            if (CircleToCircleQuery(center, closest, circleShape.r, segmentShape.r, 0, con) == 0)
//            {
//                cpVect n = con[0].n;

//                // Reject endcap collisions if tangents are provided.
//                if (
//                    (closest_t != 0.0f || cpVect.cpvdot(n, cpVect.cpvrotate(segmentShape.a_tangent, segmentShape.shape.body.Rotation)) >= 0.0f) &&
//                    (closest_t != 1.0f || cpVect.cpvdot(n, cpVect.cpvrotate(segmentShape.b_tangent, segmentShape.shape.body.Rotation)) >= 0.0f)
//                )
//                {
//                    return 1;
//                }
//            }

//            return 0;
//        }

//        public static int SegmentToSegment(cpSegmentShape seg1, cpSegmentShape seg2, int id, List<ContactPoint> arr)
//        {

//            SupportContext context = new SupportContext((cpShape)seg1, (cpShape)seg2, (a, b) =>
//            {
//                return SegmentSupportPoint((cpSegmentShape)a, b);
//            }, (a, b) =>
//            {
//                return SegmentSupportPoint((cpSegmentShape)a, b);
//            });


//            ClosestPoints points = GJK(context, id);

//#if DRAW_CLOSEST
//#if PRINT_LOG
////	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
//#endif

//    ChipmunkDebugDrawDot(6.0, points.a, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawDot(6.0, points.b, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
//#endif

//            cpVect n = points.n;
//            cpVect rot1 = seg1.shape.body.Rotation;
//            cpVect rot2 = seg2.shape.body.Rotation;
//            if (
//                points.d <= (seg1.r + seg2.r) &&
//                (
//                    (!cpVect.cpveql(points.a, seg1.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg1.a_tangent, rot1)) <= 0.0) &&
//                    (!cpVect.cpveql(points.a, seg1.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg1.b_tangent, rot1)) <= 0.0) &&
//                    (!cpVect.cpveql(points.b, seg2.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg2.a_tangent, rot2)) >= 0.0) &&
//                    (!cpVect.cpveql(points.b, seg2.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg2.b_tangent, rot2)) >= 0.0)
//                )
//            )
//            {
//                return ContactPoints(Edge.SupportEdgeForSegment(seg1, n), Edge.SupportEdgeForSegment(seg2, cpVect.cpvneg(n)), points, arr);
//            }
//            else
//            {
//                return 0;
//            }
//        }

//        public static int PolyToPoly(cpPolyShape poly1, cpPolyShape poly2, int id, List<ContactPoint> arr)
//        {

//            SupportContext context = new SupportContext(
//                 (cpShape)poly1, (cpShape)poly2,
//                  (a, b) => { return PolySupportPoint((cpPolyShape)a, b); },

//                  (a, b) => { return PolySupportPoint((cpPolyShape)a, b); }
//        );


//            ClosestPoints points = GJK(context, id);

//#if DRAW_CLOSEST
//#if PRINT_LOG
////	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
//#endif

//    ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
//#endif

//            if (points.d - poly1.r - poly2.r <= 0.0)
//            {
//                return ContactPoints(Edge.SupportEdgeForPoly(poly1, points.n), Edge.SupportEdgeForPoly(poly2, cpVect.cpvneg(points.n)), points, arr);
//            }
//            else
//            {
//                return 0;
//            }
//        }

//        public static int SegmentToPoly(cpSegmentShape seg, cpPolyShape poly, int id, List<ContactPoint> arr)
//        {
//            SupportContext context = new SupportContext(
//                   (cpShape)seg, (cpShape)poly,

//                   (a, b) =>
//                   {
//                       return SegmentSupportPoint((cpSegmentShape)a, b);
//                   }, (a, b) =>
//                   {
//                       return PolySupportPoint((cpPolyShape)a, b);
//                   });

//            ClosestPoints points = GJK(context, id);

//#if DRAW_CLOSEST
//#if PRINT_LOG
////	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
//#endif

//    ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
//#endif

//            // Reject endcap collisions if tangents are provided.
//            cpVect n = points.n;
//            cpVect rot = seg.shape.body.Rotation;
//            if (
//                points.d - seg.r - poly.r <= 0.0 &&
//                (
//                    (!cpVect.cpveql(points.a, seg.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg.a_tangent, rot)) <= 0.0) &&
//                    (!cpVect.cpveql(points.a, seg.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg.b_tangent, rot)) <= 0.0)
//                )
//            )
//            {
//                return ContactPoints(Edge.SupportEdgeForSegment(seg, n), Edge.SupportEdgeForPoly(poly, cpVect.cpvneg(n)), points, arr);
//            }
//            else
//            {
//                return 0;
//            }
//        }

//        // This one is less gross, but still gross.
//        // TODO: Comment me!
//        static int CircleToPoly(cpCircleShape circle, cpPolyShape poly, int id, ContactPoint con)
//        {

//            SupportContext context = new SupportContext(circle, poly,
//                (s, n) =>
//                {
//                    return CircleSupportPoint((cpCircleShape)s, n);
//                },
//                 (s, n) =>
//                 {
//                     return PolySupportPoint((cpPolyShape)s, n);
//                 }
//                     );

//            //{ (cpShape)circle, (cpShape)poly, (SupportPointFunc)CircleSupportPoint, (SupportPointFunc)PolySupportPoint };

//            ClosestPoints points = GJK(context, id);

//#if DRAW_CLOSEST
//    ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
//    ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
//#endif

//            float mindist = circle.r + poly.r;
//            if (points.d - mindist <= 0.0f)
//            {
//                cpVect p = cpVect.cpvlerp(points.a, points.b, circle.r / (mindist));
//                con.Init(p, points.n, points.d - mindist, "0");
//                return 1;
//            }
//            else
//            {
//                return 0;
//            }
//        }

//        static CollisionFunc[] builtinCollisionFuncs = new CollisionFunc[]  {

//            //(CollisionFunc)CircleToCircle,
//            //null,
//            //null,
//            //(CollisionFunc)CircleToSegment,
//            //null,
//            //null,
//            //(CollisionFunc)CircleToPoly,
//            //(CollisionFunc)SegmentToPoly,
//            //(CollisionFunc)PolyToPoly,
//        };

//static CollisionFunc[] colfuncs = builtinCollisionFuncs;

//static CollisionFunc[] segmentCollisions = new CollisionFunc[] {

//    (a,b,c,d) => { return CircleToCircle( (cpCircleShape) a, (cpCircleShape)b,c,d);  },
//    null,
//    null,
//      (a,b,c,d) => { return CircleToCircle( (cpCircleShape) a, (cpCircleShape)b,c,d); },
//    (a,b,c,d) => { return SegmentToSegment( (cpSegmentShape) a, (cpSegmentShape)b,c,d); },
//    null,
//      (a,b,c,d) => { return CircleToCircle( (cpCircleShape) a, (cpCircleShape)b,c,d); },
//    (a,b,c,d) => { return SegmentToSegment( (cpSegmentShape) a, (cpSegmentShape)b,c,d); },
//      (a,b,c,d) => { return PolyToPoly( (cpPolyShape) a, (cpPolyShape)b,c,d); }
//};

//public void cpEnableSegmentToSegmentCollisions()
//{
//    colfuncs = segmentCollisions;
//    //TODO: 
//}

//public static int cpCollideShapes(cpShape a, cpShape b, int id, List<ContactPoint> arr)
//{
//    // Their shape types must be in order.
//    cpEnvironment.assertSoft(a.klass.type <= b.klass.type, "Internal Error: Collision shapes passed to cpCollideShapes() are not sorted.");

//    CollisionFunc cfunc = colfuncs[((int)a.klass.type) + ((int)b.klass.type) * ((int)cpShapeType.CP_NUM_SHAPES)];

//    int numContacts = (cfunc != null ? cfunc(a, b, id, arr) : 0);
//    cpEnvironment.assertSoft(numContacts <= cpArbiter.CP_MAX_CONTACTS_PER_ARBITER, "Internal error: Too many contact points returned.");

//    return numContacts;
//}
