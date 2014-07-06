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
using System;

namespace ChipmunkSharp
{

    public interface ICollisionShape
    {
        Func<object, object, List<ContactPoint>>[] collisionTable { get; }
        int collisionCode { get; }
    }

    public class cpCircleShape : cpShape, ICollisionShape
    {

        public cpVect c, tc;
        public float r;

        public float Radius { get { return r; } }

        public cpVect Offset { get { return c; } }

        public cpCircleShape(cpBody body, float radius, cpVect offset)
            : base(body)
        {
            c = offset;
            r = radius;
            this.shapeType = cpShapeType.CIRCLE_SHAPE;
        }


        public override cpSegmentQueryInfo segmentQuery(cpVect a, cpVect b)
        {
            return cp.circleSegmentQuery(this, this.tc, this.r, a, b);
        }

        public override void cacheData(cpVect p, cpVect rot)
        {
            //var c = this.tc = vadd(p, vrotate(this.c, rot));
            var c = this.tc = cpVect.cpvrotate(this.c, rot) + p;
            //this.bb = bbNewForCircle(c, this.r);
            var r = this.r;
            this.bb_l = c.x - r;
            this.bb_b = c.y - r;
            this.bb_r = c.x + r;
            this.bb_t = c.y + r;
        }


        public override cpNearestPointQueryInfo nearestPointQuery(cpVect p)
        {
            var deltax = p.x - this.tc.x;
            var deltay = p.y - this.tc.y;
            var d = cpVect.cplength2(deltax, deltay);
            var r = this.r;

            var nearestp = new cpVect(this.tc.x + deltax * r / d, this.tc.y + deltay * r / d);
            return new cpNearestPointQueryInfo(this, nearestp, d - r);
        }



        public override void Draw(cpDraw m_debugDraw)
        {

            cpColor color = cp.GetShapeColor(this);

            m_debugDraw.DrawSolidCircle(new cpVect(tc.x, tc.y), r, cpVect.ZERO, color);

        }

        public Func<object, object, List<ContactPoint>>[] collisionTable
        {
            get
            {
                return new Func<object, object, List<ContactPoint>>[] {
                    (o1,o2) => cpCollision.circle2circle(o1 as cpCircleShape ,o2 as cpCircleShape),
                    (o1,o2) => cpCollision.circle2segment(o1 as cpCircleShape ,o2 as cpSegmentShape),
                    (o1,o2) => cpCollision.circle2poly(o1 as cpCircleShape ,o2 as cpPolyShape)
                };
            }
        }
        //(
        public int collisionCode
        {
            get { return 0; }
        }


    }

    public class cpSegmentShape : cpShape, ICollisionShape
    {

        public cpShape shape;

        public cpVect a, b, n;
        public cpVect ta, tb, tn;
        public float r;

        public cpVect a_tangent, b_tangent;

        public cpVect A { get { return a; } }
        public cpVect B { get { return n; } }
        public cpVect Normal { get { return n; } }
        public float Radius { get { return r; } }

        public cpSegmentShape(cpBody body, cpVect a, cpVect b, float r)
            : base(body)
        {

            this.a = a;
            this.b = b;

            this.n = cpVect.cpvperp(cpVect.vnormalize(cpVect.cpvsub(b, a)));

            this.ta = this.tb = this.tn = null;

            this.r = r;

            this.a_tangent = cpVect.ZERO;
            this.b_tangent = cpVect.ZERO;

            this.shapeType = cpShapeType.SEGMENT_SHAPE;
            // Shape.call(this, body);
        }

        public override void cacheData(cpVect p, cpVect rot)
        {
            this.ta = cpVect.cpvadd(p, cpVect.cpvrotate(this.a, rot));
            this.tb = cpVect.cpvadd(p, cpVect.cpvrotate(this.b, rot));
            this.tn = cpVect.cpvrotate(this.n, rot);

            float l, r, b, t;

            if (this.ta.x < this.tb.x)
            {
                l = this.ta.x;
                r = this.tb.x;
            }
            else
            {
                l = this.tb.x;
                r = this.ta.x;
            }

            if (this.ta.y < this.tb.y)
            {
                b = this.ta.y;
                t = this.tb.y;
            }
            else
            {
                b = this.tb.y;
                t = this.ta.y;
            }

            var rad = this.r;

            this.bb_l = l - rad;
            this.bb_b = b - rad;
            this.bb_r = r + rad;
            this.bb_t = t + rad;
        }

        public override cpNearestPointQueryInfo nearestPointQuery(cpVect p)
        {
            cpVect closest = cp.closestPointOnSegment(p, this.ta, this.tb);

            var deltax = p.x - closest.x;
            var deltay = p.y - closest.y;
            var d = cpVect.cplength2(deltax, deltay);
            var r = this.r;

            var nearestp = (d > 0 ? cpVect.cpvadd(closest, cpVect.cpvmult(new cpVect(deltax, deltay), r / d)) : closest);
            return new cpNearestPointQueryInfo(this, nearestp, d - r);

        }

        public override cpSegmentQueryInfo segmentQuery(cpVect a, cpVect b)
        {
            var n = this.tn;
            var d = cpVect.cpvdot(cpVect.cpvsub(this.ta, a), n);
            var r = this.r;

            var flipped_n = (d > 0 ? cpVect.cpvneg(n) : n);
            var n_offset = cpVect.cpvsub(cpVect.cpvmult(flipped_n, r), a);

            var seg_a = cpVect.cpvadd(this.ta, n_offset);
            var seg_b = cpVect.cpvadd(this.tb, n_offset);
            var delta = cpVect.cpvsub(b, a);

            if (cpVect.cpvcross(delta, seg_a) * cpVect.cpvcross(delta, seg_b) <= 0)
            {
                var d_offset = d + (d > 0 ? -r : r);
                var ad = -d_offset;
                var bd = cpVect.cpvdot(delta, n) - d_offset;

                if (ad * bd < 0)
                {
                    return new cpSegmentQueryInfo(this, ad / (ad - bd), flipped_n);
                }
            }
            else if (r != 0)
            {
                cpSegmentQueryInfo info1 = cp.circleSegmentQuery(this, this.ta, this.r, a, b);
                cpSegmentQueryInfo info2 = cp.circleSegmentQuery(this, this.tb, this.r, a, b);

                if (info1 != null)
                {
                    return info2 != null && info2.t < info1.t ? info2 : info1;
                }
                else
                {
                    return info2;
                }
            }

            return null;

        }

        public void setNeighbors(cpVect prev, cpVect next)
        {
            this.a_tangent = cpVect.cpvsub(prev, this.a);
            this.b_tangent = cpVect.cpvsub(next, this.b);
        }

        public void setEndpoints(cpVect a, cpVect b)
        {
            this.a = a;
            this.b = b;
            this.n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(b, a)));
        }

        public override void Draw(cpDraw m_debugDraw)
        {

            //var oldLineWidth = ctx.lineWidth;
            //var lineWidth = Math.Max(1, this.r  * 2);
            //drawLine(ctx, point2canvas, this.ta, this.tb);
            //ctx.lineWidth = oldLineWidth;

            cpColor color = cp.GetShapeColor(this);



            var lineWidth = Math.Max(1, this.r);  // take a look if we need to apply scale for radius
            m_debugDraw.DrawSegment(ta, tb, lineWidth, color);
        }

        public Func<object, object, List<ContactPoint>>[] collisionTable
        {
            get
            {
                return new Func<object, object, List<ContactPoint>>[] {
                    null,
                    (segA, segB) => {  return null; },
                    (o1,o2) => cpCollision.seg2poly(o1 as cpSegmentShape ,o2 as cpPolyShape)
                };
            }
        }
        //(o1,o2) => cpCollision.Circle2Circle(o1 as cpCircleShape ,o2 as cpCircleShape),
        public int collisionCode
        {
            get { return 1; }
        }


        //public cpSegmentShape Init(cpBody body, cpVect a, cpVect b, float r)
        //{

        //    this.bb = new cpBB(0, 0, 0, 0);
        //    this.a = a;
        //    this.b = b;
        //    this.n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(b, a)));
        //    this.r = r;
        //    this.a_tangent = cpVect.ZERO;// cpvzero;
        //    this.b_tangent = cpVect.ZERO;// cpvzero;

        //    //cpShapeInit((cpShape)seg, &cpSegmentShapeClass, body);
        //    base.Init(cpSegmentShapeClass, body);

        //    return this;
        //}

        //public void SetNeighbors(cpVect prev, cpVect next)
        //{
        //    cpEnvironment.assertHard(klass.Equals(cpSegmentShapeClass), "Shape is not a segment shape.");
        //    cpSegmentShape seg = (cpSegmentShape)shape;

        //    seg.a_tangent = cpVect.cpvsub(prev, seg.a);
        //    seg.b_tangent = cpVect.cpvsub(next, seg.b);
        //}


        //public static cpSegmentShape cpSegmentShapeNew(cpBody body, cpVect a, cpVect b, float r)
        //{

        //    var tmp = new cpSegmentShape();
        //    tmp.Init(body, a, b, r);
        //    return tmp;
        //    //return (cpShape)cpSegmentShapeInit(cpSegmentShapeAlloc(), body, a, b, r);
        //}

        //public void cpSegmentShapeSetEndpoints(cpVect a, cpVect b)
        //{
        //    this.a = a;
        //    this.b = b;
        //    this.n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(b, a)));
        //}

        //public void cpSegmentShapeSetRadius(float radius)
        //{
        //    this.r = radius;
        //}


        //public static void SegmentQuery(cpSegmentShape seg, cpVect a, cpVect b, cpSegmentQueryInfo info)
        //{
        //    cpVect n = seg.tn;
        //    float d = cpVect.cpvdot(cpVect.cpvsub(seg.ta, a), n);
        //    float r = seg.r;

        //    cpVect flipped_n = (d > 0.0f ? cpVect.cpvneg(n) : n);
        //    cpVect seg_offset = cpVect.cpvsub(cpVect.cpvmult(flipped_n, r), a);

        //    // Make the endpoints relative to 'a' and move them by the thickness of the segment.
        //    cpVect seg_a = cpVect.cpvadd(seg.ta, seg_offset);
        //    cpVect seg_b = cpVect.cpvadd(seg.tb, seg_offset);
        //    cpVect delta = cpVect.cpvsub(b, a);

        //    if (cpVect.cpvcross(delta, seg_a) * cpVect.cpvcross(delta, seg_b) <= 0.0f)
        //    {
        //        float d_offset = d + (d > 0.0f ? -r : r);
        //        float ad = -d_offset;
        //        float bd = cpVect.cpvdot(delta, n) - d_offset;

        //        if (ad * bd < 0.0f)
        //        {
        //            info.shape = (cpShape)seg;
        //            info.t = ad / (ad - bd);
        //            info.n = flipped_n;
        //        }
        //    }
        //    else if (r != 0.0f)
        //    {
        //        cpSegmentQueryInfo info1 = cpSegmentQueryInfo.CreateBlanck(); // { null, 1.0f, cpVect.ZERO };
        //        cpSegmentQueryInfo info2 = cpSegmentQueryInfo.CreateBlanck();// { null, 1.0f, cpVect.ZERO };
        //        SegmentQuery(seg, seg.ta, seg.r, a, b, info1);
        //        SegmentQuery(seg, seg.tb, seg.r, a, b, info2);

        //        if (info1.t < info2.t)
        //        {
        //            info.Set(info1); // (*info) = info1;
        //        }
        //        else
        //        {
        //            info.Set(info2);
        //        }
        //    }
        //}

        //public static void NearestPointQuery(cpSegmentShape seg, cpVect p, cpNearestPointQueryInfo info)
        //{
        //    cpVect closest = cpVect.closestPointOnSegment(p, seg.ta, seg.tb);

        //    cpVect delta = cpVect.cpvsub(p, closest);
        //    float d = cpVect.cpvlength(delta);
        //    float r = seg.r;
        //    cpVect g = cpVect.cpvmult(delta, 1.0f / d);

        //    info.shape = (cpShape)seg;
        //    info.p = (d != 0 ? cpVect.cpvadd(closest, cpVect.cpvmult(g, r)) : closest);
        //    info.d = d - r;

        //    // Use the segment's normal if the distance is very small.
        //    info.g = (d > cpEnvironment.MAGIC_EPSILON ? g : seg.n);
        //}

        //public static cpBB ShapeCacheData(cpSegmentShape seg, cpVect p, cpVect rot)
        //{
        //    seg.ta = cpVect.cpvadd(p, cpVect.cpvrotate(seg.a, rot));
        //    seg.tb = cpVect.cpvadd(p, cpVect.cpvrotate(seg.b, rot));
        //    seg.tn = cpVect.cpvrotate(seg.n, rot);

        //    float l, r, b, t;

        //    if (seg.ta.x < seg.tb.x)
        //    {
        //        l = seg.ta.x;
        //        r = seg.tb.x;
        //    }
        //    else
        //    {
        //        l = seg.tb.x;
        //        r = seg.ta.x;
        //    }

        //    if (seg.ta.y < seg.tb.y)
        //    {
        //        b = seg.ta.y;
        //        t = seg.tb.y;
        //    }
        //    else
        //    {
        //        b = seg.tb.y;
        //        t = seg.ta.y;
        //    }

        //    float rad = seg.r;
        //    return cpBB.cpBBNew(l - rad, b - rad, r + rad, t + rad);
        //}


    }


    /// @private
    public enum cpShapeType
    {
        CIRCLE_SHAPE = 1,
        SEGMENT_SHAPE = 2,
        POLY_SHAPE = 3,
        NUM_SHAPES = 4
    };


    //#region DELEGATES

    //public delegate cpBB cpShapeCacheDataImpl(cpShape shape, cpVect p, cpVect rot);
    //public delegate void cpShapeDestroyImpl(cpShape shape);
    //public delegate void cpShapeNearestPointQueryImpl(cpShape shape, cpVect p, cpNearestPointQueryInfo info);
    //public delegate void cpShapeSegmentQueryImpl(cpShape shape, cpVect a, cpVect b, cpSegmentQueryInfo info);

    //#endregion

    /// Opaque collision shape struct.
    public class cpShape : IObjectBox
    {

        public static int IDCounter = 0;

        #region PROPS

        //public virtual int GetCollisionCode() { throw new NotImplementedException(); }

        //public virtual ContactPoint GetCollisionTable(object a, object b) { throw new NotImplementedException(); }


        public cpShapeType shapeType;

        /// The rigid body this collision shape is attached to.
        public cpBody body;

        /// The current bounding box of the shape.

        public float bb_l { get; set; }
        public float bb_b { get; set; }
        public float bb_r { get; set; }
        public float bb_t { get; set; }

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
        public string collision_type;
        /// Group of this shape. Shapes in the same group don't collide.
        public int group;
        // Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
        public int layers;

        public cpSpace space;

        public cpShape next;

        public cpShape prev;

        public string hashid;

        #endregion

        public cpShape(cpBody body)
        {
            /// The rigid body this collision shape is attached to.
            this.body = body;

            /// The current bounding box of the shape.
            /// The current bounding box of the shape.
            this.bb_l = this.bb_b = this.bb_r = this.bb_t = 0;

            this.hashid = (cp.shapeIDCounter++).ToString();

            /// Sensor flag.
            /// Sensor shapes call collision callbacks but don't produce collisions.
            this.sensor = false;

            /// Coefficient of restitution. (elasticity)
            this.e = 0;
            /// Coefficient of friction.
            this.u = 0;
            /// Surface velocity used when solving for friction.
            this.surface_v = cpVect.ZERO;

            /// Collision type of this shape used when picking collision handlers.
            this.collision_type = "0";
            /// Group of this shape. Shapes in the same group don't collide.
            this.group = 0;
            // Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero.
            this.layers = cp.ALL_LAYERS;

            this.space = null;

            // Copy the collision code from the prototype into the actual object. This makes collision
            // function lookups slightly faster.
            //this.collisionCode = this.collisionCode;

        }

        #region SETTERS


        public void setElasticity(float value)
        {
            // throw new NotImplementedException();
            this.e = value;
        }
        public void setBody(cpBody body)
        {
            cp.assertHard(!active(), "You cannot change the body on an active shape. You must remove the shape from the space before changing the body.");
            this.body = body;
        }

        public void setGroup(int id)
        {
            group = id;
        }

        public void setLayers(int layers)
        {
            this.body.activate(); this.layers = layers;
        }

        public void setSensor(bool sensor)
        {
            this.body.activate(); this.sensor = sensor;
        }

        public void setCollisionType(string collision_type)
        {
            this.body.activate(); this.collision_type = collision_type;
        }

        public void setFriction(float value)
        {
            u = value;
        }


        #endregion

        public bool active()
        {
            return this.body != null && this.body.shapeList.IndexOf(this) != -1;
        }

        public virtual void cacheBB()
        {
            this.update(this.body.Position, this.body.Rotation);
        }


        public virtual cpSegmentQueryInfo segmentQuery(cpVect a, cpVect b)
        {
            throw new NotImplementedException();
        }

        public virtual cpNearestPointQueryInfo nearestPointQuery(cpVect p)
        {
            throw new NotImplementedException();
        }

        /// Update, cache and return the bounding box of a shape with an explicit transformation.
        public virtual void update(cpVect pos, cpVect rot)
        {
            cp.assert(!float.IsNaN(rot.x), "Rotation is NaN");
            cp.assert(!float.IsNaN(pos.x), "Position is NaN");
            this.cacheData(pos, rot);
        }

        public virtual void cacheData(cpVect pos, cpVect rot)
        {
            throw new NotImplementedException();
        }

        //public static cpBB update(cpShape shape, cpVect pos, cpVect rot)
        //{
        //    return (shape.bb = shape.klass.cacheData(shape, pos, rot));
        //}


        /// Test if a point lies within a shape.
        public virtual cpNearestPointQueryInfo pointQuery(cpVect p)
        {
            var info = this.nearestPointQuery(p);
            if (info.d < 0) return info;
            return null;
        }

        public cpBB getBB()
        {
            return new cpBB(this.bb_l, this.bb_b, this.bb_r, this.bb_t);
        }

        //public static void SegmentQuery(cpShape shape, cpVect center, float r, cpVect a, cpVect b, cpSegmentQueryInfo info)
        //{
        //    cpVect da = cpVect.cpvsub(a, center);
        //    cpVect db = cpVect.cpvsub(b, center);

        //    float qa = cpVect.cpvdot(da, da) - 2.0f * cpVect.cpvdot(da, db) + cpVect.cpvdot(db, db);
        //    float qb = -2.0f * cpVect.cpvdot(da, da) + 2.0f * cpVect.cpvdot(da, db);
        //    float qc = cpVect.cpvdot(da, da) - r * r;

        //    float det = qb * qb - 4.0f * qa * qc;

        //    if (det >= 0.0f)
        //    {
        //        float t = (-qb - cpEnvironment.cpfsqrt(det)) / (2.0f * qa);
        //        if (0.0f <= t && t <= 1.0f)
        //        {
        //            info.shape = shape;
        //            info.t = t;
        //            info.n = cpVect.cpvnormalize(cpVect.cpvlerp(da, db, t));
        //        }
        //    }
        //}

        //public static void cpResetShapeIdCounter()
        //{
        //    IDCounter = 0;
        //}

        //public void PointQueryFirst(ref cpShape outShape)
        //{
        //    if (!sensor) outShape = this;
        //}



        ///// Destroy a shape.
        //public void Destroy()
        //{
        //    if (klass.destroy != null) klass.destroy(this);
        //}
        ///// Destroy and Free a shape.
        //public void Free()
        //{

        //    Destroy();
        //    // cpfree(shape);

        //}


        ///// Update, cache and return the bounding box of a shape based on the body it's attached to.
        //public cpBB CacheBB()
        //{
        //    return update(body.Position, body.Rotation);
        //}





        ///// Perform a nearest point query. It finds the closest point on the surface of shape to a specific point.
        ///// The value returned is the distance between the points. A negative distance means the point is inside the shape.
        //public float NearestPointQuery(cpVect p, cpNearestPointQueryInfo info)
        //{
        //    klass.nearestPointQuery(this, p, info);
        //    return info.d;
        //}

        ///// Perform a segment query against a shape. @c info must be a pointer to a valid cpSegmentQueryInfo structure.
        //public bool cpShapeSegmentQuery(cpVect a, cpVect b, cpSegmentQueryInfo info)
        //{

        //    cpNearestPointQueryInfo nearest = cpNearestPointQueryInfo.CreateEmpty();
        //    klass.nearestPointQuery(this, a, nearest);

        //    if (nearest.d <= 0.0)
        //    {
        //        info.shape = this;
        //        info.t = 0.0f;
        //        info.n = cpVect.cpvnormalize(cpVect.cpvsub(a, nearest.p));
        //    }
        //    else
        //    {
        //        klass.segmentQuery(this, a, b, info);
        //    }

        //    return (info.shape != null);
        //}

        ///// Get the hit point for a segment query.
        //public static cpVect cpSegmentQueryHitPoint(cpVect start, cpVect end, cpSegmentQueryInfo info)
        //{
        //    return cpVect.cpvlerp(start, end, info.t);
        //}

        ///// Get the hit distance for a segment query.
        //public static float cpSegmentQueryHitDist(cpVect start, cpVect end, cpSegmentQueryInfo info)
        //{
        //    return cpVect.cpvdist(start, end) * info.t;
        //}



        /// When initializing a shape, it's hash value comes from a counter.
        /// Because the hash value may affect iteration order, you can reset the shape ID counter
        /// when recreating a space. This will make the simulation be deterministic.
        //#define CP_DeclareShapeGetter(struct, type, name) type struct##Get##name(const cpShape *shape)



        public virtual void Draw(cpDraw m_debugDraw)
        {
            throw new NotImplementedException();
        }



        internal void push(cpShape shape)
        {
            throw new NotImplementedException();
        }
    };

}

