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

using ChipmunkSharp.Shapes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{



    public class cpSplittingPlane
    {
        public cpVect n;
        public float d;

        public cpSplittingPlane(cpVect n, float d)
        {
            // TODO: Complete member initialization
            this.n = n;
            this.d = d;
        }


        public static cpSplittingPlane cpSplittingPlaneNew(cpVect a, cpVect b)
        {
            cpVect n = cpVect.cpvnormalize(cpVect.cpvperp(cpVect.cpvsub(b, a)));
            cpSplittingPlane plane = new cpSplittingPlane(n, cpVect.cpvdot(n, a));
            return plane;
        }

        public static float cpSplittingPlaneCompare(cpSplittingPlane plane, cpVect v)
        {
            return cpVect.cpvdot(plane.n, v) - plane.d;
        }

    } ;

    /// @private
    public class cpPolyShape : cpShape, ICollisionShape
    {

        public int numVerts { get { return verts.Count; } }

        //int numVerts;
        public List<cpVect> verts, tVerts;
        public List<cpSplittingPlane> planes, tPlanes;

        public float r;

        public static cpShapeClass polyClass = new cpShapeClass(
            cpShapeType.CP_POLY_SHAPE,
           (poly, p, rot) => cpPolyShapeCacheData((cpPolyShape)poly, p, rot),
            (poly) => cpPolyShapeDestroy((cpPolyShape)poly),
            (poly, p, info) => cpPolyShapeNearestPointQuery((cpPolyShape)poly, p, info),
            (poly, a, b, info) => cpPolyShapeSegmentQuery((cpPolyShape)poly, a, b, info));



        public static cpBB cpPolyShapeTransformVerts(cpPolyShape poly, cpVect p, cpVect rot)
        {
            List<cpVect> src = poly.verts;
            List<cpVect> dst = poly.tVerts;

            float l = (float)cpEnvironment.INFINITY_FLOAT, r = -(float)cpEnvironment.INFINITY_FLOAT;
            float b = (float)cpEnvironment.INFINITY_FLOAT, t = -(float)cpEnvironment.INFINITY_FLOAT;

            for (int i = 0; i < poly.numVerts; i++)
            {
                cpVect v = cpVect.cpvadd(p, cpVect.cpvrotate(src[i], rot));
                var vx = p.x + src[i].x * rot.x - src[i].y * rot.y;
                var vy = p.y + src[i].x * rot.y + src[i].y * rot.x;
                dst[i] = v;
                l = cpEnvironment.cpfmin(l, v.x);
                r = cpEnvironment.cpfmax(r, v.x);
                b = cpEnvironment.cpfmin(b, v.y);
                t = cpEnvironment.cpfmax(t, v.y);
            }

            float radius = poly.r;
            return cpBB.cpBBNew(l - radius, b - radius, r + radius, t + radius);
        }




        public override void Draw(cpDraw m_debugDraw)
        {

            // var verts = this.tVerts;
            //var len = tVerts.Count;
            //var lastPoint = new cpVect(tVerts[len - 2].x, tVerts[len - 1].y);
            ////ctx.moveTo(lastPoint.x, lastPoint.y);

            //for (var i = 0; i < len; i += 2)
            //{
            //    var p = new cpVect(verts[i].x, verts[i + 1].y);
            //    m_debugDraw.DrawSegment(lastPoint, p, cpColor.Red);
            //    lastPoint = p;
            //}

            m_debugDraw.DrawPolygon(tVerts, numVerts, cpColor.Red);


            //// convert chipmunk points to coco points
            //Point *pointArray = new Point[poly->numVerts];
            //for (int i=0; i < poly->numVerts; i++) {
            //    pointArray[i] = Point(poly->tVerts[i].x, poly->tVerts[i].y);
            //}

            //DrawPrimitives::drawPoly(pointArray, poly->numVerts, true);

        }


        public static void cpPolyShapeTransformAxes(cpPolyShape poly, cpVect p, cpVect rot)
        {
            List<cpSplittingPlane> src = poly.planes;
            List<cpSplittingPlane> dst = poly.tPlanes;

            for (int i = 0; i < poly.numVerts; i++)
            {
                cpVect n = cpVect.cpvrotate(src[i].n, rot);
                dst[i].n = n;
                dst[i].d = cpVect.cpvdot(p, n) + src[i].d;
            }
        }

        public static cpBB cpPolyShapeCacheData(cpPolyShape poly, cpVect p, cpVect rot)
        {
            cpPolyShapeTransformAxes(poly, p, rot);
            cpBB bb = poly.bb = cpPolyShapeTransformVerts(poly, p, rot);

            return bb;
        }


        public static void cpPolyShapeDestroy(cpPolyShape poly)
        {
            //cpfree(poly.verts);
            //cpfree(poly.planes);
        }


        public static void cpPolyShapeNearestPointQuery(cpPolyShape poly, cpVect p, cpNearestPointQueryInfo info)
        {
            int count = poly.numVerts;
            List<cpSplittingPlane> planes = poly.tPlanes;
            List<cpVect> verts = poly.tVerts;
            float r = poly.r;

            cpVect v0 = verts[count - 1];
            float minDist = cpEnvironment.INFINITY_FLOAT;
            cpVect closestPoint = cpVect.ZERO;
            cpVect closestNormal = cpVect.ZERO;
            bool outside = false;

            for (int i = 0; i < count; i++)
            {
                if (cpSplittingPlane.cpSplittingPlaneCompare(planes[i], p) > 0.0f) outside = true;

                cpVect v1 = verts[i];
                cpVect closest = cpVect.closestPointOnSegment(p, v0, v1);

                float dist = cpVect.cpvdist(p, closest);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestPoint = closest;
                    closestNormal = planes[i].n;
                }

                v0 = v1;
            }

            float dist2 = (outside ? minDist : -minDist);
            cpVect g = cpVect.cpvmult(cpVect.cpvsub(p, closestPoint), 1.0f / dist2);

            info.shape = (cpShape)poly;
            info.p = cpVect.cpvadd(closestPoint, cpVect.cpvmult(g, r));
            info.d = dist2 - r;

            // Use the normal of the closest segment if the distance is small.
            info.g = (minDist > cpEnvironment.MAGIC_EPSILON ? g : closestNormal);
        }


        public static void cpPolyShapeSegmentQuery(cpPolyShape poly, cpVect a, cpVect b, cpSegmentQueryInfo info)
        {
            List<cpSplittingPlane> axes = poly.tPlanes;
            List<cpVect> verts = poly.tVerts;
            int numVerts = poly.numVerts;
            float r = poly.r;

            for (int i = 0; i < numVerts; i++)
            {
                cpVect n = axes[i].n;
                float an = cpVect.cpvdot(a, n);
                float d = axes[i].d + r - an;
                if (d > 0.0f) continue;

                float bn = cpVect.cpvdot(b, n);
                float t = d / (bn - an);
                if (t < 0.0f || 1.0f < t) continue;

                cpVect point = cpVect.cpvlerp(a, b, t);
                float dt = -cpVect.cpvcross(n, point);
                float dtMin = -cpVect.cpvcross(n, verts[(i - 1 + numVerts) % numVerts]);
                float dtMax = -cpVect.cpvcross(n, verts[i]);

                if (dtMin <= dt && dt <= dtMax)
                {
                    info.shape = (cpShape)poly;
                    info.t = t;
                    info.n = n;
                }
            }

            // Also check against the beveled vertexes.
            if (r > 0.0f)
            {
                for (int i = 0; i < numVerts; i++)
                {
                    cpSegmentQueryInfo circle_info = cpSegmentQueryInfo.CreateBlanck(); //  { null, 1.0f, cpVect.ZERO };
                    cpCircleShape.SegmentQuery(poly, verts[i], r, a, b, circle_info);
                    if (circle_info.t < info.t) info.Set(circle_info);
                }
            }
        }


        public static void setUpVerts(cpPolyShape poly, List<cpVect> verts, cpVect offset)
        {
            int numVerts = verts.Count;

            // Fail if the user attempts to pass a concave poly, or a bad winding.
            cpEnvironment.AssertHard(cpPolyShape.cpPolyValidate(verts, numVerts), "Polygon is concave or has a reversed winding. Consider using cpConvexHull() or CP_CONVEX_HULL().");
            //poly.numVerts = numVerts;
            poly.verts = verts; // (cpVect)cpcalloc(2 * numVerts, sizeof(cpVect));
            poly.planes = new List<cpSplittingPlane>();  // (cpSplittingPlane)cpcalloc(2 * numVerts, sizeof(cpSplittingPlane));
            //poly.tVerts = poly.verts; // +numVerts;
            poly.tVerts = new List<cpVect>();
            poly.tPlanes = new List<cpSplittingPlane>(); //+ numVerts;

            for (int i = 0; i < numVerts; i++)
            {
                cpVect a = cpVect.cpvadd(offset, verts[i]);
                cpVect b = cpVect.cpvadd(offset, verts[(i + 1) % numVerts]);
                cpVect n = cpVect.cpvnormalize(cpVect.cpvperp(cpVect.cpvsub(b, a)));

                poly.verts[i] = a;
                poly.planes.Add(new cpSplittingPlane(n, cpVect.cpvdot(n, a)));
                poly.tPlanes.Add(new cpSplittingPlane(cpVect.ZERO, 0));
                //poly.planes[i].n = n;
                //poly.planes[i].d = cpVect.cpvdot(n, a);
                poly.tVerts.Add(cpVect.ZERO);
            }

            // TODO: Why did I add this? It duplicates work from above.
            //            for (int i = 0; i < numVerts; i++)
            //            {
            //				var a = poly.verts[(i - 1 + numVerts) % numVerts];
            //				var b = poly.verts[i];
            //				poly.planes.Add(cpSplittingPlane.cpSplittingPlaneNew(a,b));
            //            }
        }

        /// Allocate a polygon shape.

        /// Initialize a polygon shape.
        /// A convex hull will be created from the vertexes.
        public static cpPolyShape cpPolyShapeInit(cpPolyShape poly, cpBody body, List<cpVect> verts, cpVect offset)
        {
            return cpPolyShapeInit2(poly, body, verts, offset, 0.0f);
        }
        /// Initialize a polygon shape.
        /// A convex hull will be created from the vertexes.
        public static cpPolyShape cpPolyShapeInit2(cpPolyShape poly, cpBody body, List<cpVect> verts, cpVect offset, float radius)
        {
            setUpVerts(poly, verts, offset);
            poly.Init(polyClass, body);
            //cpShapeInit((cpShape)poly,);
            poly.r = radius;

            return poly;
        }
        /// Allocate and initialize a polygon shape.
        /// A convex hull will be created from the vertexes.
        public static cpShape cpPolyShapeNew(cpBody body, List<cpVect> verts, cpVect offset)
        {
            return cpPolyShapeNew2(body, verts, offset, 0.0f);
        }
        /// Allocate and initialize a polygon shape.
        /// A convex hull will be created from the vertexes.
        public static cpShape cpPolyShapeNew2(cpBody body, List<cpVect> verts, cpVect offset, float radius)
        {

            return (cpShape)cpPolyShapeInit2(new cpPolyShape(), body, verts, offset, radius);
        }

        /// Initialize a box shaped polygon shape.
        public static cpPolyShape cpBoxShapeInit(cpPolyShape poly, cpBody body, float width, float height)
        {
            float hw = width / 2.0f;
            float hh = height / 2.0f;

            return cpBoxShapeInit2(poly, body, cpBB.cpBBNew(-hw, -hh, hw, hh));
        }
        /// Initialize an offset box shaped polygon shape.
        public static cpPolyShape cpBoxShapeInit2(cpPolyShape poly, cpBody body, cpBB box)
        {
            return cpBoxShapeInit3(poly, body, box, 0.0f);
        }
        /// Initialize an offset box shaped polygon shape.
        public static cpPolyShape cpBoxShapeInit3(cpPolyShape poly, cpBody body, cpBB box, float radius)
        {
            List<cpVect> verts = new List<cpVect>() {
	new cpVect(box.l, box.b),
	new cpVect(box.l, box.t),
	new cpVect(box.r, box.t),
	new cpVect(box.r, box.b),
	};

            return cpPolyShapeInit2(poly, body, verts, cpVect.ZERO, radius);
        }
        /// Allocate and initialize a box shaped polygon shape.
        public static cpShape cpBoxShapeNew(cpBody body, float width, float height)
        {

            return (cpShape)cpBoxShapeInit(new cpPolyShape(), body, width, height);
        }
        /// Allocate and initialize an offset box shaped polygon shape.
        public static cpShape cpBoxShapeNew2(cpBody body, cpBB box)
        {
            return (cpShape)cpBoxShapeInit2(new cpPolyShape(), body, box);
        }
        /// Allocate and initialize an offset box shaped polygon shape.
        public static cpShape cpBoxShapeNew3(cpBody body, cpBB box, float radius)
        {
            return (cpShape)cpBoxShapeInit3(new cpPolyShape(), body, box, radius);
        }

        /// Check that a set of vertexes is convex and has a clockwise winding.
        /// NOTE: Due to floating point precision issues, hulls created with cpQuickHull() are not guaranteed to validate!
        public static bool cpPolyValidate(List<cpVect> verts, int numVerts)
        {
            for (int i = 0; i < numVerts; i++)
            {
                cpVect a = verts[i];
                cpVect b = verts[(i + 1) % numVerts];
                cpVect c = verts[(i + 2) % numVerts];

                if (cpVect.cpvcross(cpVect.cpvsub(b, a), cpVect.cpvsub(c, a)) > 0.0f)
                {
                    return false;
                }
            }

            return true;
        }

        /// Get the number of verts in a polygon shape.
        public static int cpPolyShapeGetNumVerts(cpShape shape)
        {
            cpEnvironment.AssertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            return ((cpPolyShape)shape).numVerts;
        }
        /// Get the @c ith vertex of a polygon shape.
        public static cpVect cpPolyShapeGetVert(cpShape shape, int idx)
        {
            cpEnvironment.AssertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            cpEnvironment.AssertHard(0 <= idx && idx < cpPolyShapeGetNumVerts(shape), "Index out of range.");

            return ((cpPolyShape)shape).verts[idx];
        }
        /// Get the radius of a polygon shape.
        public static float cpPolyShapeGetRadius(cpShape shape)
        {
            cpEnvironment.AssertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            return ((cpPolyShape)shape).r;
        }

        /// @}
        // Unsafe API (chipmunk_unsafe.h)

        public static void cpPolyShapeSetVerts(cpShape shape, List<cpVect> verts, cpVect offset)
        {
            cpEnvironment.AssertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            cpPolyShapeDestroy((cpPolyShape)shape);
            setUpVerts((cpPolyShape)shape, verts, offset);
        }

        public static void cpPolyShapeSetRadius(cpShape shape, float radius)
        {
            cpEnvironment.AssertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            ((cpPolyShape)shape).r = radius;
        }


        public cpVect GetVert(int idx)
        {

            //cpEnvironment.cpAssertHard(klass == &polyClass, "Shape is not a poly shape.");
            cpEnvironment.AssertHard(0 <= idx && idx < cpPolyShapeGetNumVerts(this), "Index out of range.");

            return verts[idx];
        }

        public Func<object, object, List<cpContact>>[] collisionTable
        {
            get
            {
                return new Func<object, object, List<cpContact>>[] {
                    null,
                    null,
                    (o1,o2) => cpCollision.Poly2Poly(o1 as cpPolyShape ,o2 as cpPolyShape)
                };
            }
        }
        //(o1,o2) => cpCollision.Circle2Circle(o1 as cpCircleShape ,o2 as cpCircleShape),
        public int collisionCode
        {
            get { return 2; }
        }



        //internal float valueOnAxis(cpVect n, float d)
        //{
        //    var verts = this.tVerts;
        //    var m = cpVect.cpvdot2(n.x, n.y, verts[0], verts[1]);

        //    for (var i = 2; i < verts.Count; i += 2)
        //    {
        //        m = Math.Min(m, cpVect.cpvdot2(n.x, n.y, verts[i], verts[i + 1]));
        //    }

        //    return m - d;
        //}
    }

}
