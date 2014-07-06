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


        /// Initialize a polygon shape.
        /// The vertexes must be convex and have a clockwise winding.

        public cpSplittingPlane(cpVect n, float d)
        {
            // TODO: Complete member initialization
            this.n = n;
            this.d = d;
        }

        public float compare(cpVect v)
        {
            return cpVect.cpvdot(n, v) - d;
        }

    }

    /// @private
    public class cpPolyShape : cpShape, ICollisionShape
    {


        public int numVerts { get { return verts.Length; } }

        //int numVerts;
        public float[] verts, tVerts;
        public cpSplittingPlane[] planes, tPlanes;

        public float r;

        public cpPolyShape(cpBody body, float[] verts, cpVect offset)
            : base(body)
        {
            this.setVerts(verts, offset);
            this.shapeType = cpShapeType.POLY_SHAPE;
            //Shape.call(this, body);
        }

        public void setVerts(float[] verts, cpVect offset)
        {
            cp.assert(verts.Length >= 4, "Polygons require some verts");
            //cpEnvironment.assert(typeof(verts[0]) == 'number',
            //        'Polygon verticies should be specified in a flattened list (eg [x1,y1,x2,y2,x3,y3,...])');

            // Fail if the user attempts to pass a concave poly, or a bad winding.
            cp.assert(cp.polyValidate(verts), "Polygon is concave or has a reversed winding. Consider using cpConvexHull()");

            var len = verts.Length;
            var numVerts = len >> 1;

            // This a pretty bad way to do this in javascript. As a first pass, I want to keep
            // the code similar to the C.
            this.verts = new float[len];
            this.tVerts = new float[len];
            this.planes = new cpSplittingPlane[numVerts];
            this.tPlanes = new cpSplittingPlane[numVerts];

            for (var i = 0; i < len; i += 2)
            {
                //var a = vadd(offset, verts[i]);
                //var b = vadd(offset, verts[(i+1)%numVerts]);
                var ax = verts[i] + offset.x;
                var ay = verts[i + 1] + offset.y;
                var bx = verts[(i + 2) % len] + offset.x;
                var by = verts[(i + 3) % len] + offset.y;

                // Inefficient, but only called during object initialization.
                var n = cpVect.cpvnormalize(cpVect.cpvperp(new cpVect(bx - ax, by - ay)));

                this.verts[i] = ax;
                this.verts[i + 1] = ay;
                this.planes[i >> 1] = new cpSplittingPlane(n, cpVect.cpvdot2(n.x, n.y, ax, ay));
                this.tPlanes[i >> 1] = new cpSplittingPlane(new cpVect(0, 0), 0);
            }
        }


        public void transformVerts(cpVect p, cpVect rot)
        {
            var src = this.verts;
            var dst = this.tVerts;

            float l = cp.Infinity, r = -cp.Infinity;
            float b = cp.Infinity, t = -cp.Infinity;

            for (var i = 0; i < src.Length; i += 2)
            {
                //var v = vadd(p, vrotate(src[i], rot));
                var x = src[i];
                var y = src[i + 1];

                var vx = p.x + x * rot.x - y * rot.y;
                var vy = p.y + x * rot.y + y * rot.x;

                //console.log('(' + x + ',' + y + ') -> (' + vx + ',' + vy + ')');

                dst[i] = vx;
                dst[i + 1] = vy;

                l = Math.Min(l, vx);
                r = Math.Max(r, vx);
                b = Math.Min(b, vy);
                t = Math.Max(t, vy);
            }

            this.bb_l = l;
            this.bb_b = b;
            this.bb_r = r;
            this.bb_t = t;
        }


        public void transformAxes(cpVect p, cpVect rot)
        {
            var src = this.planes;
            var dst = this.tPlanes;

            for (var i = 0; i < src.Length; i++)
            {
                var n = cpVect.cpvrotate(src[i].n, rot);
                dst[i].n = n;
                dst[i].d = cpVect.cpvdot(p, n) + src[i].d;
            }
        }


        public override void cacheData(cpVect p, cpVect rot)
        {
            this.transformAxes(p, rot);
            this.transformVerts(p, rot);
        }

        public override cpNearestPointQueryInfo nearestPointQuery(cpVect p)
        {
            var planes = this.tPlanes;
            var verts = this.tVerts;

            var v0x = verts[verts.Length - 2];
            var v0y = verts[verts.Length - 1];
            var minDist = cp.Infinity;
            var closestPoint = cpVect.ZERO;
            var outside = false;

            for (var i = 0; i < planes.Length; i++)
            {
                if (planes[i].compare(p) > 0) outside = true;

                var v1x = verts[i * 2];
                var v1y = verts[i * 2 + 1];
                var closest = cp.closestPointOnSegment2(p.x, p.y, v0x, v0y, v1x, v1y);

                var dist = cpVect.cpvdist(p, closest);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestPoint = closest;
                }

                v0x = v1x;
                v0y = v1y;
            }

            return new cpNearestPointQueryInfo(this, closestPoint, (outside ? minDist : -minDist));
        }

        public override cpSegmentQueryInfo segmentQuery(cpVect a, cpVect b)
        {
            var axes = this.tPlanes;
            var verts = this.tVerts;
            var numVerts = axes.Length;
            var len = numVerts * 2;

            for (var i = 0; i < numVerts; i++)
            {
                var n = axes[i].n;
                var an = cpVect.cpvdot(a, n);
                if (axes[i].d > an) continue;

                var bn = cpVect.cpvdot(b, n);
                var t = (axes[i].d - an) / (bn - an);
                if (t < 0 || 1 < t) continue;

                var point = cpVect.cpvlerp(a, b, t);
                var dt = -cpVect.cpvcross(n, point);
                var dtMin = -cpVect.cpvcross2(n.x, n.y, verts[i * 2], verts[i * 2 + 1]);
                var dtMax = -cpVect.cpvcross2(n.x, n.y, verts[(i * 2 + 2) % len], verts[(i * 2 + 3) % len]);

                if (dtMin <= dt && dt <= dtMax)
                {
                    // josephg: In the original C code, this function keeps
                    // looping through axes after finding a match. I *think*
                    // this code is equivalent...
                    return new cpSegmentQueryInfo(this, t, n);
                }
            }
            return cpSegmentQueryInfo.CreateBlanck();
        }

        public float valueOnAxis(cpVect n, float d)
        {
            var verts = this.tVerts;
            var m = cpVect.cpvdot2(n.x, n.y, verts[0], verts[1]);

            for (var i = 2; i < verts.Length; i += 2)
            {
                m = Math.Min(m, cpVect.cpvdot2(n.x, n.y, verts[i], verts[i + 1]));
            }

            return m - d;
        }

        public bool containsVert(float vx, float vy)
        {
            var planes = this.tPlanes;

            for (var i = 0; i < planes.Length; i++)
            {
                var n = planes[i].n;
                var dist = cpVect.cpvdot2(n.x, n.y, vx, vy) - planes[i].d;
                if (dist > 0) return false;
            }

            return true;
        }

        public bool containsVertPartial(float vx, float vy, cpVect n)
        {
            var planes = this.tPlanes;

            for (var i = 0; i < planes.Length; i++)
            {
                var n2 = planes[i].n;
                if (cpVect.cpvdot(n2, n) < 0) continue;
                var dist = cpVect.cpvdot2(n2.x, n2.y, vx, vy) - planes[i].d;
                if (dist > 0) return false;
            }

            return true;
        }

        public int getNumVerts()
        {
            return this.verts.Length / 2;
        }

        // These methods are provided for API compatibility with Chipmunk. I recommend against using
        // them - just access the poly.verts list directly.
        public cpVect getVert(int i)
        {
            return new cpVect(this.verts[i * 2], this.verts[i * 2 + 1]);
        }

        public List<cpVect> getVers()
        {
            List<cpVect> dev = new List<cpVect>();
            for (int i = 0; i < getNumVerts(); i++)
                dev.Add(getVert(i));
            return dev;
        }

        public override void Draw(cpDraw m_debugDraw)
        {

            // var verts = this.tVerts;
            var len = tVerts.Count();

            List<cpVect> vertices = new List<cpVect>();

            var lastPoint = new cpVect(tVerts[len - 2], tVerts[len - 1]);
            ////ctx.moveTo(lastPoint.x, lastPoint.y);

            cpColor color = cp.styles[60];  //cp.GetShapeColor(this);
            //cpColor color = new cpColor(255, 0, 100);

            for (var i = 0; i < len; i += 2)
            {
                var p = new cpVect(tVerts[i], tVerts[i + 1]);
                m_debugDraw.DrawSegment(lastPoint, p, color);
                lastPoint = p;
            }
            //m_debugDraw.DrawPolygon(getVers(), getNumVerts(), cpColor.Red);


            //// convert chipmunk points to coco points
            //Point *pointArray = new Point[poly->numVerts];
            //for (int i=0; i < poly->numVerts; i++) {
            //    pointArray[i] = Point(poly->tVerts[i].x, poly->tVerts[i].y);
            //}

            //DrawPrimitives::drawPoly(pointArray, poly->numVerts, true);

        }


        public Func<object, object, List<ContactPoint>>[] collisionTable
        {
            get
            {
                return new Func<object, object, List<ContactPoint>>[] {
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


    }

}


/*

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
            cpEnvironment.assertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            return ((cpPolyShape)shape).numVerts;
        }
        /// Get the @c ith vertex of a polygon shape.
        public static cpVect cpPolyShapeGetVert(cpShape shape, int idx)
        {
            cpEnvironment.assertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            cpEnvironment.assertHard(0 <= idx && idx < cpPolyShapeGetNumVerts(shape), "Index out of range.");

            return ((cpPolyShape)shape).verts[idx];
        }
        /// Get the radius of a polygon shape.
        public static float cpPolyShapeGetRadius(cpShape shape)
        {
            cpEnvironment.assertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            return ((cpPolyShape)shape).r;
        }

        /// @}
        // Unsafe API (chipmunk_unsafe.h)

        public static void cpPolyShapeSetVerts(cpShape shape, List<cpVect> verts, cpVect offset)
        {
            cpEnvironment.assertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            cpPolyShapeDestroy((cpPolyShape)shape);
            setUpVerts((cpPolyShape)shape, verts, offset);
        }

        public static void cpPolyShapeSetRadius(cpShape shape, float radius)
        {
            cpEnvironment.assertHard(shape.klass.Equals(polyClass), "Shape is not a poly shape.");
            ((cpPolyShape)shape).r = radius;
        }


        public cpVect GetVert(int idx)
        {

            //cpEnvironment.cpAssertHard(klass == &polyClass, "Shape is not a poly shape.");
            cpEnvironment.assertHard(0 <= idx && idx < cpPolyShapeGetNumVerts(this), "Index out of range.");

            return verts[idx];
        }*/