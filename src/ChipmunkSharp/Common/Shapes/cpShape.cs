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

using ChipmunkSharp.Constraints;
using ChipmunkSharp.Shapes;
using System.Collections.Generic;

namespace ChipmunkSharp
{

    public class cpCircleShape : cpShape
    {

        public static cpShapeClass cpCircleShapeClass = new cpShapeClass(cpShapeType.CP_CIRCLE_SHAPE,
         (shape, p, rot) => ShapeCacheData((cpCircleShape)shape, p, rot),
        null,
        (shape, p, info) => NearestPointQuery((cpCircleShape)shape, p, info),
         (shape, a, b, info) => SegmentQuery((cpCircleShape)shape, a, b, info));


        public cpVect c, tc;
        public float r;

        /// @}
        /// @defgroup cpCircleShape cpCircleShape

        /// @privatepu  


        /// Allocate a circle shape.
        //cpCircleShape cpCircleShapeAlloc() { }
        /// Initialize a circle shape.
        public void Init(cpBody body, float radius, cpVect offset)
        {
            c = offset;
            r = radius;
            base.Init(cpCircleShapeClass, body);
        }
        /// Allocate and initialize a circle shape.
        public static cpCircleShape cpCircleShapeNew(cpBody body, float radius, cpVect offset)
        {
            cpCircleShape dev = new cpCircleShape();
            dev.Init(body, radius, offset);
            return dev;
        }

        public cpVect Offset { get { return c; } }

        public float Radius { get { return r; } }

        //CP_DeclareShapeGetter(cpCircleShape, cpVect, Offset);
        //CP_DeclareShapeGetter(cpCircleShape, cpFloat, Radius);

        public static cpBB ShapeCacheData(cpCircleShape circle, cpVect p, cpVect rot)
        {
            cpVect c = circle.tc = cpVect.cpvadd(p, cpVect.cpvrotate(circle.c, rot));
            return cpBB.cpBBNewForCircle(c, circle.r);
        }

        public override void Draw(ChipmunkDraw m_debugDraw)
        {

            m_debugDraw.DrawSolidCircle(new cpVect(c.x, c.y), r, cpVect.ZERO, cpColor.Red);


        }

        public static void NearestPointQuery(cpCircleShape circle, cpVect p, cpNearestPointQueryInfo info)
        {
            cpVect delta = cpVect.cpvsub(p, circle.tc);
            float d = cpVect.cpvlength(delta);

            info.shape = circle;
            info.p = cpVect.cpvadd(circle.tc, cpVect.cpvmult(delta, circle.r / d)); // TODO div/0
            info.d = d - circle.r;

            // Use up for the gradient if the distance is very small.
            info.g = (d > cpEnvironment.MAGIC_EPSILON ? cpVect.cpvmult(delta, 1.0f / d) : new cpVect(0.0f, 1.0f));
        }

        public static void SegmentQuery(cpCircleShape circle, cpVect a, cpVect b, cpSegmentQueryInfo info)
        {
            SegmentQuery((cpShape)circle, circle.tc, circle.r, a, b, info);
        }

        public static void ShapeSegmentQuery(cpCircleShape circle, cpVect a, cpVect b, cpSegmentQueryInfo info)
        {
            SegmentQuery((cpShape)circle, circle.tc, circle.r, a, b, info);
        }

        public void SetRadius(float radius)
        {
            // cpAssertHard(shape->klass == &cpCircleShapeClass, "Shape is not a circle shape.");
            r = radius;
        }


        public void SetOffset(cpVect offset)
        {
            c = offset;
        }

    }

    public class cpSegmentShape : cpShape
    {

        public static cpShapeClass cpSegmentShapeClass = new cpShapeClass(cpShapeType.CP_CIRCLE_SHAPE,
       (shape, p, rot) => ShapeCacheData((cpSegmentShape)shape, p, rot),
      null,
      (shape, p, info) => NearestPointQuery((cpSegmentShape)shape, p, info),
       (shape, a, b, info) => SegmentQuery((cpSegmentShape)shape, a, b, info));

        public static void SegmentQuery(cpSegmentShape seg, cpVect a, cpVect b, cpSegmentQueryInfo info)
        {
            cpVect n = seg.tn;
            float d = cpVect.cpvdot(cpVect.cpvsub(seg.ta, a), n);
            float r = seg.r;

            cpVect flipped_n = (d > 0.0f ? cpVect.cpvneg(n) : n);
            cpVect seg_offset = cpVect.cpvsub(cpVect.cpvmult(flipped_n, r), a);

            // Make the endpoints relative to 'a' and move them by the thickness of the segment.
            cpVect seg_a = cpVect.cpvadd(seg.ta, seg_offset);
            cpVect seg_b = cpVect.cpvadd(seg.tb, seg_offset);
            cpVect delta = cpVect.cpvsub(b, a);

            if (cpVect.cpvcross(delta, seg_a) * cpVect.cpvcross(delta, seg_b) <= 0.0f)
            {
                float d_offset = d + (d > 0.0f ? -r : r);
                float ad = -d_offset;
                float bd = cpVect.cpvdot(delta, n) - d_offset;

                if (ad * bd < 0.0f)
                {
                    info.shape = (cpShape)seg;
                    info.t = ad / (ad - bd);
                    info.n = flipped_n;
                }
            }
            else if (r != 0.0f)
            {
                cpSegmentQueryInfo info1 = cpSegmentQueryInfo.CreateBlanck(); // { null, 1.0f, cpVect.ZERO };
                cpSegmentQueryInfo info2 = cpSegmentQueryInfo.CreateBlanck();// { null, 1.0f, cpVect.ZERO };
                SegmentQuery(seg, seg.ta, seg.r, a, b, info1);
                SegmentQuery(seg, seg.tb, seg.r, a, b, info2);

                if (info1.t < info2.t)
                {
                    info.Set(info1); // (*info) = info1;
                }
                else
                {
                    info.Set(info2);
                }
            }
        }

        public static void NearestPointQuery(cpSegmentShape seg, cpVect p, cpNearestPointQueryInfo info)
        {
            cpVect closest = cpVect.closestPointOnSegment(p, seg.ta, seg.tb);

            cpVect delta = cpVect.cpvsub(p, closest);
            float d = cpVect.cpvlength(delta);
            float r = seg.r;
            cpVect g = cpVect.cpvmult(delta, 1.0f / d);

            info.shape = (cpShape)seg;
            info.p = (d != 0 ? cpVect.cpvadd(closest, cpVect.cpvmult(g, r)) : closest);
            info.d = d - r;

            // Use the segment's normal if the distance is very small.
            info.g = (d > cpEnvironment.MAGIC_EPSILON ? g : seg.n);
        }

        public static cpBB ShapeCacheData(cpSegmentShape seg, cpVect p, cpVect rot)
        {
            seg.ta = cpVect.cpvadd(p, cpVect.cpvrotate(seg.a, rot));
            seg.tb = cpVect.cpvadd(p, cpVect.cpvrotate(seg.b, rot));
            seg.tn = cpVect.cpvrotate(seg.n, rot);

            float l, r, b, t;

            if (seg.ta.x < seg.tb.x)
            {
                l = seg.ta.x;
                r = seg.tb.x;
            }
            else
            {
                l = seg.tb.x;
                r = seg.ta.x;
            }

            if (seg.ta.y < seg.tb.y)
            {
                b = seg.ta.y;
                t = seg.tb.y;
            }
            else
            {
                b = seg.tb.y;
                t = seg.ta.y;
            }

            float rad = seg.r;
            return cpBB.cpBBNew(l - rad, b - rad, r + rad, t + rad);
        }


        public cpShape shape;

        public cpVect a, b, n;
        public cpVect ta, tb, tn;
        public float r;

        public cpVect a_tangent, b_tangent;


        public cpVect A { get { return a; } }
        public cpVect B { get { return n; } }
        public cpVect Normal { get { return n; } }
        public float Radius { get { return r; } }


        public cpSegmentShape Init(cpBody body, cpVect a, cpVect b, float r)
        {
            this.a = a;
            this.b = b;
            this.n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(b, a)));
            this.r = r;
            this.a_tangent = cpVect.ZERO;// cpvzero;
            this.b_tangent = cpVect.ZERO;// cpvzero;

            //cpShapeInit((cpShape)seg, &cpSegmentShapeClass, body);
            base.Init(cpSegmentShapeClass, body);

            return this;
        }


        public void SetNeighbors(cpVect prev, cpVect next)
        {
            cpEnvironment.cpAssertHard(klass.Equals(cpSegmentShapeClass), "Shape is not a segment shape.");
            cpSegmentShape seg = (cpSegmentShape)shape;

            seg.a_tangent = cpVect.cpvsub(prev, seg.a);
            seg.b_tangent = cpVect.cpvsub(next, seg.b);
        }


        public static cpSegmentShape cpSegmentShapeNew(cpBody body, cpVect a, cpVect b, float r)
        {

            var tmp = new cpSegmentShape();
            tmp.Init(body, a, b, r);
            return tmp;
            //return (cpShape)cpSegmentShapeInit(cpSegmentShapeAlloc(), body, a, b, r);
        }


        public void cpSegmentShapeSetEndpoints(cpVect a, cpVect b)
        {
            this.a = a;
            this.b = b;
            this.n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(b, a)));
        }

        public void cpSegmentShapeSetRadius(float radius)
        {
            this.r = radius;
        }


    }
    /// @private
    public struct cpShapeClass
    {

        public cpShapeType type;
        public cpShapeCacheDataImpl cacheData;
        public cpShapeDestroyImpl destroy;
        public cpShapeNearestPointQueryImpl nearestPointQuery;
        public cpShapeSegmentQueryImpl segmentQuery;

        public cpShapeClass(cpShapeType type, cpShapeCacheDataImpl cacheData, cpShapeDestroyImpl destroy,
            cpShapeNearestPointQueryImpl nearestPointQuery, cpShapeSegmentQueryImpl segmentQuery)
        {
            this.type = type;
            this.cacheData = cacheData;
            this.destroy = destroy;
            this.nearestPointQuery = nearestPointQuery;
            this.segmentQuery = segmentQuery;

        }





    };

    /// @private
    public enum cpShapeType
    {
        CP_CIRCLE_SHAPE = 1,
        CP_SEGMENT_SHAPE = 2,
        CP_POLY_SHAPE = 3,
        CP_NUM_SHAPES = 4
    };


    #region DELEGATES

    public delegate cpBB cpShapeCacheDataImpl(cpShape shape, cpVect p, cpVect rot);
    public delegate void cpShapeDestroyImpl(cpShape shape);
    public delegate void cpShapeNearestPointQueryImpl(cpShape shape, cpVect p, cpNearestPointQueryInfo info);
    public delegate void cpShapeSegmentQueryImpl(cpShape shape, cpVect a, cpVect b, cpSegmentQueryInfo info);

    #endregion

    /// Opaque collision shape struct.
    public class cpShape
    {

        public int Collides(cpShape b, int id, List<cpContact> arr)
        {
            return cpCollision.cpCollideShapes(this, b, id, arr);
        }


        public static void SegmentQuery(cpShape shape, cpVect center, float r, cpVect a, cpVect b, cpSegmentQueryInfo info)
        {
            cpVect da = cpVect.cpvsub(a, center);
            cpVect db = cpVect.cpvsub(b, center);

            float qa = cpVect.cpvdot(da, da) - 2.0f * cpVect.cpvdot(da, db) + cpVect.cpvdot(db, db);
            float qb = -2.0f * cpVect.cpvdot(da, da) + 2.0f * cpVect.cpvdot(da, db);
            float qc = cpVect.cpvdot(da, da) - r * r;

            float det = qb * qb - 4.0f * qa * qc;

            if (det >= 0.0f)
            {
                float t = (-qb - cpEnvironment.cpfsqrt(det)) / (2.0f * qa);
                if (0.0f <= t && t <= 1.0f)
                {
                    info.shape = shape;
                    info.t = t;
                    info.n = cpVect.cpvnormalize(cpVect.cpvlerp(da, db, t));
                }
            }
        }


        public static int cpShapeIDCounter = 0;

        #region PROPS
        public cpShapeClass klass;

        /// The rigid body this collision shape is attached to.
        public cpBody body;

        /// The current bounding box of the shape.
        public cpBB bb;

        /// Sensor flag.
        /// Sensor shapes call collision callbacks but don't produce collisions.
        public bool sensor;

        /// Coefficient of restitution. (elasticity)
        public float e;
        /// Coefficient of friction.
        public float u;
        /// Surface velocity used when solving for friction.
        public cpVect surface_v;

        /// User definable data pointer.
        /// Generally this points to your the game object class so you can access it
        /// when given a cpShape reference in a callback.
        public object data;

        /// Collision type of this shape used when picking collision handlers.
        public int collision_type;
        /// Group of this shape. Shapes in the same group don't collide.
        public int group;
        // Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
        public int layers;

        public cpSpace space;

        public cpShape next;

        public cpShape prev;

        public int hashid;

        #endregion

        public bool Active()
        {
            return prev != null || (body != null && body.shapeList == this);
        }

        public static void cpResetShapeIdCounter()
        {
            cpShapeIDCounter = 0;
        }

        public void PointQueryFirst(ref cpShape outShape)
        {
            if (!sensor) outShape = this;
        }

        public void Init(cpShapeClass klass, cpBody body)
        {
            this.klass = klass;

            this.hashid = cpShapeIDCounter;
            cpShapeIDCounter++;

            this.body = body;
            this.sensor = false;

            this.e = 0.0f;
            this.u = 0.0f;
            this.surface_v = cpVect.ZERO;

            this.collision_type = 0;
            this.group = cpEnvironment.CP_NO_GROUP;
            this.layers = cpEnvironment.CP_ALL_LAYERS;

            this.data = null;

            this.space = null;

            this.next = null;
            this.prev = null;

        }

        /// Destroy a shape.
        public void Destroy()
        {
            if (klass.destroy != null) klass.destroy(this);
        }
        /// Destroy and Free a shape.
        public void Free()
        {

            Destroy();
            // cpfree(shape);

        }




        /// Update, cache and return the bounding box of a shape based on the body it's attached to.
        public cpBB CacheBB()
        {
            return Update(body.Position, body.Rotation);
        }

        /// Update, cache and return the bounding box of a shape with an explicit transformation.
        public cpBB Update(cpVect pos, cpVect rot)
        {
            return Update(this, pos, rot);
        }

        public static cpBB Update(cpShape shape, cpVect pos, cpVect rot)
        {
            return (shape.bb = shape.klass.cacheData(shape, pos, rot));
        }


        /// Test if a point lies within a shape.
        public bool PointQuery(cpVect p)
        {
            cpNearestPointQueryInfo info = cpNearestPointQueryInfo.CreateEmpty(); // new cpNearestPointQueryInfo(null, cpVect.ZERO, cpEnvironment.INFINITY_FLOAT, cpVect.ZERO);
            NearestPointQuery(p, info);
            return (info.d < 0.0f);
        }

        /// Perform a nearest point query. It finds the closest point on the surface of shape to a specific point.
        /// The value returned is the distance between the points. A negative distance means the point is inside the shape.
        public float NearestPointQuery(cpVect p, cpNearestPointQueryInfo info)
        {
            klass.nearestPointQuery(this, p, info);
            return info.d;
        }

        /// Perform a segment query against a shape. @c info must be a pointer to a valid cpSegmentQueryInfo structure.
        public bool cpShapeSegmentQuery(cpVect a, cpVect b, cpSegmentQueryInfo info)
        {

            cpNearestPointQueryInfo nearest = cpNearestPointQueryInfo.CreateEmpty();
            klass.nearestPointQuery(this, a, nearest);

            if (nearest.d <= 0.0)
            {
                info.shape = this;
                info.t = 0.0f;
                info.n = cpVect.cpvnormalize(cpVect.cpvsub(a, nearest.p));
            }
            else
            {
                klass.segmentQuery(this, a, b, info);
            }

            return (info.shape != null);
        }

        /// Get the hit point for a segment query.
        public static cpVect cpSegmentQueryHitPoint(cpVect start, cpVect end, cpSegmentQueryInfo info)
        {
            return cpVect.cpvlerp(start, end, info.t);
        }

        /// Get the hit distance for a segment query.
        public static float cpSegmentQueryHitDist(cpVect start, cpVect end, cpSegmentQueryInfo info)
        {
            return cpVect.cpvdist(start, end) * info.t;
        }

        public void SetBody(cpBody body)
        {
            cpEnvironment.cpAssertHard(!Active(), "You cannot change the body on an active shape. You must remove the shape from the space before changing the body.");
            this.body = body;
        }


        /// When initializing a shape, it's hash value comes from a counter.
        /// Because the hash value may affect iteration order, you can reset the shape ID counter
        /// when recreating a space. This will make the simulation be deterministic.
        //#define CP_DeclareShapeGetter(struct, type, name) type struct##Get##name(const cpShape *shape)



        public void SetGroup(int id)
        {
            group = id;
        }

        public void SetElasticity(float value)
        {
            // throw new NotImplementedException();
            this.e = value;
        }

        public void SetFriction(float value)
        {
            u = value;
        }

        public virtual void Draw(ChipmunkDraw m_debugDraw)
        {

        }
    };

}

