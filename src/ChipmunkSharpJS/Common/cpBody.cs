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
using System;
using System.Collections.Generic;

namespace ChipmunkSharp
{


    ///// @defgroup cpBody cpBody
    ///// Chipmunk's rigid body type. Rigid bodies hold the physical properties of an object like
    ///// it's mass, and position and velocity of it's center of gravity. They don't have an shape on their own.
    ///// They are given a shape by creating collision shapes (cpShape) that point to the body.
    ///// @{

    ///// Rigid body velocity update function type.
    //public delegate void cpBodyVelocityFunc(cpVect gravity, float damping, float dt);
    ///// Rigid body position update function type.
    //public delegate void cpBodyPositionFunc(cpBody body, float dt);


    /// Used internally to track information on the collision graph.
    /// @private
    public struct cpComponentNode
    {
        public cpBody root;
        public cpBody next;
        public float idleTime;

        public cpComponentNode(cpBody root, cpBody next, float idleTime)
        {
            this.root = root;
            this.next = next;
            this.idleTime = idleTime;
        }

    } ;


    /// Chipmunk's rigid body struct.
    public class cpBody
    {

        #region STATIC VARS

        //public static void cpv_assert_nan(cpVect v, char* message) { cpAssertSoft(v.x == v.x && v.y == v.y, message); }
        //public static void cpv_assert_infinite(cpVect v, char* message) { cpAssertSoft(cpfabs(v.x) != INFINITY && cpfabs(v.y) != INFINITY, message); }
        //public static void cpv_assert_sane(cpVect v, string* message) { cpv_assert_nan(v, message); cpv_assert_infinite(v, message); }

        public cpVect Rotation { get { return rot; } }
        public float AngVel { get { return w; } set { w = value; } }
        public float Torque { get { return t; } set { t = value; } }
        public float VelLimit { get { return v_limit; } set { v_limit = value; } }
        public float AngVelLimit { get { return w_limit; } set { w_limit = value; } }
        public object UserData { get { return data; } set { data = value; } }

        public cpVect Vel { get { return v; } set { v = value; } }
        public cpVect Force { get { return f; } set { f = value; } }

        public float Angle { get { return a; } }

        public cpVect Position { get { return p; } set { setPos(value); } }

        public float Moment { get { return i; } set { setMoment(value); } }

        public float Mass { get { return m; } set { setMass(value); } }



        //public float Position { get { return p; } }

        #endregion

        #region PROPS

        /// Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity)
        // public cpBodyVelocityFunc velocity_func;

        /// Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition)
        // public cpBodyPositionFunc position_func;

        /// Mass of the body.
        /// Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason.
        private float m;
        /// Mass inverse.
        public float m_inv;


        /// Moment of inertia of the body.
        /// Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for this reason.
        public float i;
        /// Moment of inertia inverse.
        public float i_inv;

        /// Position of the rigid body's center of gravity.
        private cpVect p;
        /// Velocity of the rigid body's center of gravity.
        public cpVect v;
        /// Force acting on the rigid body's center of gravity.
        public cpVect f;

        /// Rotation of the body around it's center of gravity in radians.
        /// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
        public float a;
        /// Angular velocity of the body around it's center of gravity in radians/second.
        public float w;
        /// Torque applied to the body around it's center of gravity.
        public float t;

        /// Cached unit length vector representing the angle of the body.
        /// Used for fast rotations using cpvrotate().
        private cpVect rot;

        /// User definable data pointer.
        /// Generally this points to your the game object class so you can access it
        /// when given a cpBody reference in a callback.
        public object data;

        /// Maximum velocity allowed when updating the velocity.
        float v_limit;
        /// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
        float w_limit;

        public cpVect v_bias;
        public float w_bias;

        public cpSpace space;

        public List<cpShape> shapeList;
        public cpArbiter arbiterList;
        public cpConstraint constraintList;

        //public cpComponentNode node;

        #endregion

        #region CONSTRUCTORS

        public cpBody nodeRoot { get; set; }
        public cpBody nodeNext { get; set; }

        public float nodeIdleTime { get; set; }

        /// <summary>
        /// CREATES A STATIC BODY
        /// </summary>
        public cpBody()
            : this(cp.Infinity, cp.Infinity)
        {
            nodeIdleTime = cp.Infinity;
        }

        /// <summary>
        /// CREATES A BODY WITH MASS AND INERTIA
        /// </summary>
        /// <param name="m"></param>
        /// <param name="i"></param>
        public cpBody(float m, float i)
        {
            /// Mass of the body.
            /// Must agree with cpBody.m_inv! Use body.setMass() when changing the mass for this reason.
            //this.m;
            /// Mass inverse.
            //this.m_inv;

            /// Moment of inertia of the body.
            /// Must agree with cpBody.i_inv! Use body.setMoment() when changing the moment for this reason.
            //this.i;
            /// Moment of inertia inverse.
            //this.i_inv;

            /// Position of the rigid body's center of gravity.
            this.p = cpVect.ZERO;
            /// Velocity of the rigid body's center of gravity.
            this.v = cpVect.ZERO;
            /// Force acting on the rigid body's center of gravity.
            this.f = cpVect.ZERO;

            /// Rotation of the body around it's center of gravity in radians.
            /// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
            //this.a;
            /// Angular velocity of the body around it's center of gravity in radians/second.
            this.w = 0;
            /// Torque applied to the body around it's center of gravity.
            this.t = 0;

            /// Cached unit length vector representing the angle of the body.
            /// Used for fast rotations using cpvrotate().
            //cpVect rot;

            /// Maximum velocity allowed when updating the velocity.
            this.v_limit = cp.Infinity;
            /// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
            this.w_limit = cp.Infinity;

            // This stuff is all private.
            this.v_bias = cpVect.ZERO; //x = this.v_biasy = 0;
            this.w_bias = 0;

            this.space = null;

            this.shapeList = new List<cpShape>();

            this.arbiterList = null; // These are both wacky linked lists.
            this.constraintList = null;

            // This stuff is used to track information on the collision graph.
            this.nodeRoot = null;

            this.nodeNext = null;
            this.nodeIdleTime = 0;

            // Set this.m and this.m_inv
            this.setMass(m);

            // Set this.i and this.i_inv
            this.setMoment(i);

            // Set this.a and this.rot
            this.rot = cpVect.ZERO;
            this.setAngle(0);


        }

        #endregion

        #region PUBLIC METHODS

        public void sanityCheck()
        {
            //cpEnvironment.AssertHard(this.m == this.m && this.m_inv == this.m_inv, "Body's mass is invalid.");
            //cpEnvironment.assert(this.i == this.i && this.i_inv == this.i_inv, "Body's moment is invalid.");
            //cpEnvironment.v_assert_sane(this.p, "Body's position is invalid.");
            //cpEnvironment.v_assert_sane(this.f, "Body's force is invalid.");
            //cpEnvironment.assert(this.vx == this.vx && Math.abs(this.vx) != Infinity, "Body's velocity is invalid.");
            //cpEnvironment.assert(this.vy == this.vy && Math.abs(this.vy) != Infinity, "Body's velocity is invalid.");
            //cpEnvironment.assert(this.a == this.a && Math.abs(this.a) != Infinity, "Body's angle is invalid.");
            //cpEnvironment.assert(this.w == this.w && Math.abs(this.w) != Infinity, "Body's angular velocity is invalid.");
            //cpEnvironment.assert(this.t == this.t && Math.abs(this.t) != Infinity, "Body's torque is invalid.");
            //cpEnvironment.v_assert_sane(this.rot, "Body's rotation vector is invalid.");
            //cpEnvironment.assert(this.v_limit == this.v_limit, "Body's velocity limit is invalid.");
            //cpEnvironment.assert(this.w_limit == this.w_limit, "Body's angular velocity limit is invalid.");

        }

        public cpVect getPos() { return this.p; }
        public cpVect getVel() { return v; }
        public float getAngVel() { return this.w; }


        /// Returns true if the body is sleeping.
        public bool isSleeping()
        {
            return this.nodeRoot != null;
        }

        /// Returns true if the body is static.
        public bool isStatic()
        {
            return nodeIdleTime == cp.Infinity;
        }

        /// Returns true if the body has not been added to a space.
        /// Note: Static bodies are a subtype of rogue bodies.
        public bool isRogue()
        {
            return space == null;  //(cpSpace)0));
        }

        /// Set the mass of a body.
        public void setMass(float mass)
        {
            cp.assertHard(mass > 0.0f, "Mass must be positive and non-zero.");

            activate();
            m = mass;
            m_inv = 1.0f / mass;
        }

        //CP_DefineBodyStructGetter(float, i, Moment)
        /// Set the moment of a body.
        public void setMoment(float moment)
        {
            cp.assertHard(moment > 0.0f, "Moment of Inertia must be positive and non-zero.");

            activate();
            i = moment;
            i_inv = 1.0f / moment;
        }

        public void addShape(cpShape shape)
        {
            this.shapeList.Add(shape);
        }

        public void removeShape(cpShape shape)
        {
            // This implementation has a linear time complexity with the number of shapes.
            // The original implementation used linked lists instead, which might be faster if
            // you're constantly editing the shape of a body. I expect most bodies will never
            // have their shape edited, so I'm just going to use the simplest possible implemention.
            shapeList.Remove(shape);
        }


        public void removeConstraint(cpConstraint constraint)
        {
            constraintList = cp.filterConstraints(constraintList, this, constraint);
        }


        /// Set the position of a body.
        public void setPos(cpVect pos)
        {
            this.activate();
            this.sanityCheck();
            // If I allow the position to be set to vzero, vzero will get changed.
            //if (pos == cpVect.ZERO) {
            //    pos = cp.v(0,0);
            //}
            this.p = pos;
        }

        public void setAngle(float angle)
        {
            this.activate();
            this.sanityCheck();
            this.setAngleInternal(angle);
        }

        public void setVel(cpVect velocity)
        {
            this.activate();
            this.v.x = velocity.x;
            this.v.y = velocity.y;
        }


        public void setAngVel(float w)
        {
            this.activate();
            this.w = w;
        }


        public void setAngleInternal(float angle)
        {
            cp.assertHard(!float.IsNaN(angle), "Internal Error: Attempting to set body's angle to NaN");
            this.a = angle;//fmod(a, (cpFloat)M_PI*2.0f);

            //this.rot = vforangle(angle);
            this.rot.x = cp.cpfcos(angle);
            this.rot.y = cp.cpfsin(angle);
        }


        ///// Rigid body velocity update function type.
        //public delegate void cpBodyVelocityFunc(cpVect gravity, float damping, float dt);
        ///// Rigid body position update function type.
        //public delegate void cpBodyPositionFunc(cpBody body, float dt);


        public void velocity_func(cpVect gravity, float damping, float dt)
        {
            //this.v = vclamp(vadd(vmult(this.v, damping), vmult(vadd(gravity, vmult(this.f, this.m_inv)), dt)), this.v_limit);
            var vx = this.v.x * damping + (gravity.x + this.f.x * this.m_inv) * dt;
            var vy = this.v.y * damping + (gravity.y + this.f.y * this.m_inv) * dt;

            //var v = vclamp(new Vect(vx, vy), this.v_limit);
            //this.vx = v.x; this.vy = v.y;
            var v_limit = this.v_limit;
            var lensq = vx * vx + vy * vy;
            var scale = (lensq > v_limit * v_limit) ? v_limit / cp.cpfsqrt(lensq) : 1;
            this.v.x = vx * scale;
            this.v.y = vy * scale;

            var w_limit = this.w_limit;
            this.w = cp.cpfclamp(this.w * damping + this.t * this.i_inv * dt, -w_limit, w_limit);

            this.sanityCheck();
        }


        public void position_func(float dt)
        {
            //this.p = vadd(this.p, vmult(vadd(this.v, this.v_bias), dt));

            //this.p = this.p + (this.v + this.v_bias) * dt;
            this.p.x += (this.v.x + this.v_bias.x) * dt;
            this.p.y += (this.v.y + this.v_bias.y) * dt;

            this.setAngleInternal(this.a + (this.w + this.w_bias) * dt);

            this.v_bias.x = this.v_bias.y = 0;
            this.w_bias = 0;

            this.sanityCheck();
        }

        /// Set the forces and torque or a body to zero.
        public void resetForces()
        {
            this.activate();
            f = cpVect.ZERO;
            t = 0.0f;
        }

        /// Apply an force (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
        public void applyForce(cpVect force, cpVect r)
        {
            this.activate();
            f = cpVect.cpvadd(force, force);
            t += cpVect.cpvcross(r, force);
        }

        /// Apply an impulse (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
        public void applyImpulse(cpVect j, cpVect r)
        {
            this.activate();
            cp.apply_impulse(this, j, r);
        }

        public cpVect getVelAtPoint(cpVect r)
        {
            return cpVect.cpvadd(v, cpVect.cpvmult(cpVect.cpvperp(r), w));
        }

        /// Get the velocity on a body (in world units) at a point on the body in world coordinates.
        public cpVect GetVelAtWorldPoint(cpVect point)
        {
            return getVelAtPoint(cpVect.cpvsub(point, p));
        }


        /// Get the velocity on a body (in world units) at a point on the body in local coordinates.
        public cpVect GetVelAtLocalPoint(cpVect point)
        {
            return getVelAtPoint(cpVect.cpvrotate(point, rot));
        }


        //public delegate void cpBodyComponentIteratorFunc(cpBody body, cpBody component, object data);


        ///// Body/shape iterator callback function type. 
        //public delegate void cpBodyShapeIteratorFunc(cpBody body, cpShape shape, object data);


        public void eachShape(Action<cpShape> func)
        {
            for (int i = 0, len = this.shapeList.Count; i < len; i++)
            {
                func(this.shapeList[i]);
            }
        }

        ///// Body/raint iterator callback function type. 
        //public delegate void cpBodyConstraintIteratorFunc(cpBody body, cpConstraint raint, object data);


        public void eachConstraint(Action<cpConstraint> func)
        {
            var constraint = this.constraintList;
            while (constraint != null)
            {
                var next = constraint.next(this);
                func(constraint);
                constraint = next;
            }
        }


        /// Body/arbiter iterator callback function type. 
        //public delegate void cpBodyArbiterIteratorFunc(cpBody body, cpArbiter arbiter, object data);

        public void eachArbiter(Action<cpArbiter> func)
        {
            var arb = this.arbiterList;
            while (arb != null)
            {
                var next = arb.next(this);

                arb.swappedColl = (this == arb.body_b);
                func(arb);

                arb = next;
            }
        }


        /// Convert body relative/local coordinates to absolute/world coordinates.
        public cpVect local2World(cpVect v)
        {
            return cpVect.cpvadd(p, cpVect.cpvrotate(v, rot));
        }

        /// Convert body absolute/world coordinates to  relative/local coordinates.
        public cpVect world2Local(cpVect v)
        {
            return cpVect.cpvunrotate(cpVect.cpvsub(v, p), rot);
        }

        //        /// Get the kinetic energy of a body.
        public float kineticEnergy()
        {
            // Need to do some fudging to avoid NaNs
            float vsq = cpVect.cpvdot(v, v);
            float wsq = w * w;
            return (vsq == 0 ? vsq * m : 0.0f) + (wsq == 0 ? wsq * i : 0.0f);
        }


        //        // Defined in cpSpace.c
        //        /// Wake up a sleeping or idle body.
        public void activate()
        {
            if (!this.isRogue())
            {
                this.nodeIdleTime = 0;
                cp.componentActivate(cp.componentRoot(this));
            }

        }
        //        /// Wake up any sleeping or idle bodies touching a static body.
        public void activateStatic(cpShape filter)
        {
            cp.assertHard(this.isStatic(), "Body.activateStatic() called on a non-static body.");

            for (var arb = this.arbiterList; arb != null; arb = arb.next(this))
            {
                if (filter == null || filter == arb.a || filter == arb.b)
                {
                    (arb.body_a == this ? arb.body_b : arb.body_a).activate();
                }
            }

            // TODO should also activate joints!
        }




        public void pushArbiter(cpArbiter arb)
        {

            cp.assertSoft((arb.body_a == this ? arb.thread_a_next : arb.thread_b_next) == null,
            "Internal Error: Dangling contact graph pointers detected. (A)");
            cp.assertSoft((arb.body_a == this ? arb.thread_a_prev : arb.thread_b_prev) == null,
                "Internal Error: Dangling contact graph pointers detected. (B)");

            var next = this.arbiterList;
            cp.assertSoft(next == null || (next.body_a == this ? next.thread_a_prev : next.thread_b_prev) == null,
                "Internal Error: Dangling contact graph pointers detected. (C)");

            if (arb.body_a == this)
            {
                arb.thread_a_next = next;
            }
            else
            {
                arb.thread_b_next = next;
            }

            if (next != null)
            {
                if (next.body_a == this)
                {
                    next.thread_a_prev = arb;
                }
                else
                {
                    next.thread_b_prev = arb;
                }
            }
            this.arbiterList = arb;
        }



        //        /// Force a body to fall asleep immediately.
        public void Sleep()
        {
            this.sleepWithGroup(null);
        }


        //        /// Force a body to fall asleep immediately along with other bodies in a group.
        public void sleepWithGroup(cpBody group)
        {
            cp.assertSoft(!this.isStatic() && !this.isRogue(), "Rogue and static bodies cannot be put to sleep.");

            var space = this.space;
            cp.assertSoft(space != null, "Cannot put a rogue body to sleep.");
            cp.assertSoft(space.isLocked, "Bodies cannot be put to sleep during a query or a call to cpSpaceStep(). Put these calls into a post-step callback.");
            cp.assertSoft(group == null || group.isSleeping(), "Cannot use a non-sleeping body as a group identifier.");

            if (this.isSleeping())
            {
                cp.assertSoft(cp.componentRoot(this) == cp.componentRoot(group), "The body is already sleeping and it's group cannot be reassigned.");
                return;
            }

            for (var i = 0; i < this.shapeList.Count; i++)
            {
                this.shapeList[i].update(this.p, this.rot);
            }
            space.deactivateBody(this);

            if (group != null)
            {
                var root = cp.componentRoot(group);

                this.nodeRoot = root;
                this.nodeNext = root.nodeNext;
                this.nodeIdleTime = 0;

                root.nodeNext = this;
            }
            else
            {
                this.nodeRoot = this;
                this.nodeNext = null;
                this.nodeIdleTime = 0;

                space.sleepingComponents.Add(this);
            }
            space.bodies.Remove(this);

        }







        //        //CP_DefineBodyStructGetter(cpVect, p, Pos)

        //        /// Initialize a static cpBody.
        //        public void InitStatic()
        //        {
        //            Init(cpEnvironment.Infinity, cpEnvironment.Infinity);
        //            node.idleTime = cpEnvironment.Infinity;
        //        }

        //        /// Initialize a cpBody.
        //        private void Init(float m, float i)
        //        {
        //            space = null;
        //            shapeList = null;
        //            arbiterList = null;
        //            constraintList = null;

        //            velocity_func = UpdateVelocity;
        //            position_func = UpdatePosition;

        //            this.node = new cpComponentNode(null, null, 0.0f);

        //            p = cpVect.ZERO;
        //            v = cpVect.ZERO;
        //            f = cpVect.ZERO;

        //            w = 0.0f;
        //            t = 0.0f;

        //            v_bias = cpVect.ZERO;
        //            w_bias = 0.0f;

        //            v_limit = cpEnvironment.Infinity;
        //            w_limit = cpEnvironment.Infinity;

        //            data = null;

        //            // Setters must be called after full initialization so the sanity checks don't assert on garbage data.
        //            SetMass(m);
        //            SetMoment(i);
        //            SetAngle(0.0f);
        //        }






        //        /// Allocate and initialize a static cpBody.

        //        /// Destroy a cpBody.
        //        public void Destroy() { }

        //        /// Destroy and free a cpBody.
        //        public void Free() { }

        //        /// Check that the properties of a body is sane. (Only in debug mode)
        //        static void SanityCheck(cpBody body)
        //        {
        //#if DEBUG

        //            // Most of these do not make any sense except for the infinity checks.
        //            //cpEnvironment.cpAssertSoft(body.m == body.m && body.m_inv == body.m_inv, "Body's mass is invalid.");
        //            //cpEnvironment.cpAssertSoft(body.i == body.i && body.i_inv == body.i_inv, "Body's moment is invalid.");
        //            //cpEnvironment.cpAssertSoft((body.p != null) && (body.p.x == body.p.x && body.p.y == body.p.y) && (!double.IsInfinity(Math.Abs(body.p.x)) && !double.IsInfinity(Math.Abs(body.p.y))),
        //            //    "Body's position is invalid.");
        //            //cpEnvironment.cpAssertSoft((body.v != null) && (body.v.x == body.v.x && body.v.y == body.v.y) && (!double.IsInfinity(Math.Abs(body.v.x)) && !double.IsInfinity(Math.Abs(body.v.y))),
        //            //    "Body's velocity is invalid.");
        //            //cpEnvironment.cpAssertSoft((body.f != null) && (body.f.x == body.f.x && body.f.y == body.f.y) && (!double.IsInfinity(Math.Abs(body.f.x)) && !double.IsInfinity(Math.Abs(body.f.y))),
        //            //    "Body's force is invalid.");
        //            //cpEnvironment.cpAssertSoft(body.a == body.a && !double.IsInfinity(Math.Abs(body.a)), "Body's angle is invalid.");
        //            //cpEnvironment.cpAssertSoft(body.w == body.w && !double.IsInfinity(Math.Abs(body.w)), "Body's anglular velocity is invalid.");
        //            //cpEnvironment.cpAssertSoft(body.t == body.t && !double.IsInfinity(Math.Abs(body.t)), "Body's torque is invalid.");
        //            //cpEnvironment.cpAssertSoft((body.rot != null) && (body.rot.x == body.rot.x && body.rot.y == body.rot.y) && (!double.IsInfinity(Math.Abs(body.rot.x)) && !double.IsInfinity(Math.Abs(body.rot.y))),
        //            //    "Body's rotation vector is invalid.");
        //            //cpEnvironment.cpAssertSoft(body.v_limit == body.v_limit, "Body's velocity limit is invalid.");
        //            //cpEnvironment.cpAssertSoft(body.w_limit == body.w_limit, "Body's angular velocity limit is invalid.");
        //#endif
        //        }

        //        private void AssertSane()
        //        {
        //            //  throw new NotImplementedException();
        //            // for now just call SanityCheck
        //            SanityCheck(this);
        //        }



        //        public static void ComponentActivate(cpBody root)
        //        {
        //            if (root == null || !root.IsSleeping()) return;
        //            cpEnvironment.AssertHard(!root.IsRogue(), "Internal Error: ComponentActivate() called on a rogue body.");

        //            // cpSpace space = space;
        //            //cpBody body = this;
        //            cpSpace space = root.space;
        //            cpBody body = root;

        //            while (body != null)
        //            {
        //                cpBody next = body.node.next;

        //                body.node.idleTime = 0.0f;
        //                body.node.root = null;
        //                body.node.next = null;

        //                space.ActivateBody(body);
        //                //cpSpace.cpSpace

        //                body = next;
        //            }
        //            space.sleepingComponents.Remove(root);
        //            //cpArrayDeleteObj(, );
        //        }


        //        //public static cpBody ComponentRoot(cpBody body)
        //        //{
        //        //    return (body != null ? body.node.root : null);
        //        //}

        //        public cpBody ComponentRoot()
        //        {
        //            return node.root;
        //        }



        //        //#define CP_DefineBodyStructGetter(type, member, name) \

        //        //static  type cpBodyGet##name( cpBody *body){return body.member;}

        //        //#define CP_DefineBodyStructSetter(type, member, name) \

        //        //static  void cpBodySet##name(cpBody *body,  type value){ \
        //        //    cpBodyActivate(body); \
        //        //    body.member = value; \
        //        //    cpBodyAssertSane(body); \
        //        //}

        //        //#define CP_DefineBodyStructProperty(type, member, name) \
        //        //CP_DefineBodyStructGetter(type, member, name) \
        //        //CP_DefineBodyStructSetter(type, member, name)

        //        //// TODO add to docs
        //        //CP_DefineBodyStructGetter(cpSpace, CP_PRIVATE(space), Space)

        //        //CP_DefineBodyStructGetter(float, m, Mass)



        //        public static cpConstraint FilterConstraints(cpConstraint node, cpBody body, cpConstraint filter)
        //        {
        //            if (node == filter)
        //            {
        //                return node.Next(body);// cpConstraintNext(node, body);
        //            }
        //            else if (node.a == body)
        //            {
        //                node.next_a = FilterConstraints(node.next_a, body, filter);
        //            }
        //            else
        //            {
        //                node.next_b = FilterConstraints(node.next_b, body, filter);
        //            }

        //            return node;
        //        }





        //        public void UpdateVelocity(cpVect gravity, float damping, float dt)
        //        {
        //            UpdateVelocity(this, gravity, damping, dt);
        //        }

        //        public static void UpdateVelocity(cpBody body, cpVect gravity, float damping, float dt)
        //        {
        //            body.v = cpVect.cpvclamp(cpVect.cpvadd(cpVect.cpvmult(body.v, damping), cpVect.cpvmult(cpVect.cpvadd(gravity, cpVect.cpvmult(body.f, body.m_inv)), dt)), body.v_limit);

        //            // float w_limit = w_limit;
        //            body.w = cpEnvironment.cpfclamp(body.w * damping + body.t * body.i_inv * dt, -body.w_limit, body.w_limit);

        //            SanityCheck(body);
        //        }

        //        public void UpdatePosition(float dt)
        //        {
        //            UpdatePosition(this, dt);
        //        }

        //        public static void UpdatePosition(cpBody body, float dt)
        //        {
        //            body.p = cpVect.cpvadd(body.p, cpVect.cpvmult(cpVect.cpvadd(body.v, body.v_bias), dt));

        //            body.SetAngle(body.a + (body.w + body.w_bias) * dt);

        //            body.v_bias = cpVect.ZERO;
        //            body.w_bias = 0.0f;

        //            SanityCheck(body);
        //        }


        //        /// Default Integration functions.
        //        //public void UpdateVelocity(cpVect gravity, float damping, float dt)
        //        //{
        //        //    v = cpVect.cpvclamp(cpVect.cpvadd(cpVect.cpvmult(v, damping), cpVect.cpvmult(cpVect.cpvadd(gravity, cpVect.cpvmult(f, m_inv)), dt)), v_limit);

        //        //    w = cpEnvironment.cpfclamp(w * damping + t * i_inv * dt, -w_limit, w_limit);

        //        //    SanityCheck();
        //        //}

        //        //public void UpdatePosition(float dt)
        //        //{
        //        //    p = cpVect.cpvadd(p, cpVect.cpvmult(cpVect.cpvadd(v, v_bias), dt));
        //        //    setAngle(a + (w + w_bias) * dt);

        //        //    v_bias = cpVect.ZERO;
        //        //    w_bias = 0.0f;

        //        //    SanityCheck();
        //        //}









        //        /// Call @c func once for each shape attached to @c body and added to the space.
        //        public void EachShape(cpBodyShapeIteratorFunc func, object data)
        //        {
        //            cpShape shape = shapeList;
        //            while (shape != null)
        //            {
        //                cpShape next = shape.next;
        //                func(this, shape, data);
        //                shape = next;
        //            }
        //        }


        //        /// Call @c func once for each raint attached to @c body and added to the space.
        //        public void EachConstraint(cpBodyConstraintIteratorFunc func, object data)
        //        {
        //            cpConstraint constraint = constraintList;
        //            while (constraint != null)
        //            {
        //                cpConstraint next = constraint.Next(this); // cpConstraintNext(constraint, this);
        //                func(this, constraint, data);
        //                constraint = next;
        //            }
        //        }


        //        /// Call @c func once for each arbiter that is currently active on the body.
        //        public void EachArbiter(cpBodyArbiterIteratorFunc func, object data)
        //        {
        //            cpArbiter arb = arbiterList;
        //            while (arb != null)
        //            {
        //                cpArbiter next = arb.Next(this);

        //                arb.swappedColl = (this == arb.body_b);
        //                func(this, arb, data);

        //                arb = next;
        //            }
        //        }

        //        public void EachComponent(cpBodyComponentIteratorFunc func, object data)
        //        {
        //            for (cpBody component = node.root; component != null; component = component.node.next)
        //                func(this, component, data);
        //        }


        //        //public void EachComponent(Func<cpBody, cpBody, object> func, object data)
        //        //{
        //        //    for (cpBody component = node.root; component != null; component = component.node.next)
        //        //        func(this, component);
        //        //}

        //        //    #define CP_BODY_FOREACH_COMPONENT(root, var)\
        //        //for(cpBody *var = root; var; var = var->node.next)


        //        ///@}

        //        #region Space component


        //        #endregion


        //        public void SetTorque(float torque)
        //        {
        //            Torque = torque;
        //            //throw new NotImplementedException();
        //        }



    }
        #endregion


}
