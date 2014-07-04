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

using System;
using System.Collections.Generic;

namespace ChipmunkSharp
{


    /// @defgroup cpArbiter cpArbiter
    /// The cpArbiter struct controls pairs of colliding shapes.
    /// They are also used in conjuction with collision handler callbacks
    /// allowing you to retrieve information on the collision and control it.
    /// @{

    /// Collision begin event function callback type.
    /// Returning false from a begin callback causes the collision to be ignored until
    /// the the separate callback is called when the objects stop colliding.
    public delegate bool cpCollisionBeginFunc(cpArbiter arb, cpSpace space, object data);
    /// Collision pre-solve event function callback type.
    /// Returning false from a pre-step callback causes the collision to be ignored until the next step.
    public delegate bool cpCollisionPreSolveFunc(cpArbiter arb, cpSpace space, object data);
    /// Collision post-solve event function callback type.
    public delegate void cpCollisionPostSolveFunc(cpArbiter arb, cpSpace space, object data);
    /// Collision separate event function callback type.
    public delegate void cpCollisionSeparateFunc(cpArbiter arb, cpSpace space, object data);

    /// @private
    public class cpCollisionHandler : IObjectBox
    {


        public int a;
        public int b;
        public cpCollisionBeginFunc begin;
        public cpCollisionPreSolveFunc preSolve;
        public cpCollisionPostSolveFunc postSolve;
        public cpCollisionSeparateFunc separate;
        public object data;


        public cpCollisionHandler(int a, int b, cpCollisionBeginFunc begin, cpCollisionPreSolveFunc preSolve, cpCollisionPostSolveFunc postSolve, cpCollisionSeparateFunc separate, object data)
        {
            this.a = a;
            this.b = b;
            this.begin = begin;
            this.preSolve = preSolve;
            this.postSolve = postSolve;
            this.separate = separate;
            this.data = data;

            // bb = new cpBB(0f, 0f, 0f, 0f);
        }

        public cpCollisionHandler()
        {
            // TODO: Complete member initialization
        }


        public cpCollisionHandler Clone()
        {
            cpCollisionHandler copy = new cpCollisionHandler();
            copy.a = a;
            copy.b = b;
            copy.begin = begin;
            copy.preSolve = preSolve;
            copy.postSolve = postSolve;
            copy.separate = separate;
            copy.data = data;
            return copy;
        }

        public cpBB bb { get; set; }

    };

    /// @private
    public enum cpArbiterState
    {
        // Arbiter is active and its the first collision.
        cpArbiterStateFirstColl = 1,
        // Arbiter is active and its not the first collision.
        cpArbiterStateNormal = 2,
        // Collision has been explicitly ignored.
        // Either by returning false from a begin collision handler or calling cpArbiterIgnore().
        cpArbiterStateIgnore = 3,
        // Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
        cpArbiterStateCached = 4,
    } ;

    public struct arbiterFilterContext
    {
        public cpSpace space;
        public cpBody body;
        public cpShape shape;

        public arbiterFilterContext(cpSpace space, cpBody body, cpShape shape)
        {
            this.space = space;
            this.body = body;
            this.shape = shape;

        }


    };


    /// @private
    public class cpArbiterThread
    {
        // Links to next and previous arbiters in the contact graph.
        public cpArbiter next, prev;

        public void Clear()
        {
            prev = next = null;
        }
    };


    /// A colliding pair of shapes.
    public class cpArbiter : IObjectBox
    {



        public static int CP_MAX_CONTACTS_PER_ARBITER = 4;

        #region PUBLIC PROPS

        /// Calculated value to use for the elasticity coefficient.
        /// Override in a pre-solve collision handler for custom behavior.
        public float e;
        /// Calculated value to use for the friction coefficient.
        /// Override in a pre-solve collision handler for custom behavior.
        public float u;
        /// Calculated value to use for applying surface velocities.
        /// Override in a pre-solve collision handler for custom behavior.
        public cpVect surface_vr { get; set; }

        /// User definable data pointer.
        /// The value will persist for the pair of shapes until the separate() callback is called.
        /// NOTE: If you need to clean up this pointer, you should implement the separate() callback to do it.
        //public cpDataPointer data;

        public object data;

        #endregion

        #region PRIVATE PROPS

        public cpShape b { get; set; }
        public cpShape a { get; set; }

        public cpBody body_a { get; set; }
        public cpBody body_b { get; set; }

        private cpArbiterThread thread_a { get; set; }
        private cpArbiterThread thread_b { get; set; }

        //private struct cpArbiterThread thread_a;
        //private struct cpArbiterThread thread_b;

        public List<cpContact> contacts { get; set; }

        public cpCollisionHandler handler { get; set; }


        public int stamp;

        public bool swappedColl { get; set; }


        public cpArbiterState state;

        #endregion

        #region STUDY PROPS

        public cpShape A
        {
            get
            {
                if (swappedColl)
                    return b;
                else
                    return a;
            }
        }

        public cpShape B
        {
            get
            {
                if (swappedColl)
                    return a;
                else
                    return b;
            }
        }

        public float Elasticity { get { return e; } set { e = value; } }
        public float Friction { get { return u; } set { u = value; } }

        public object UserData { get { return data; } set { data = value; } }

        public int numContacts { get { return contacts.Count; } }


        #endregion

        // TODO make this generic so I can reuse it for constraints also.

        public cpArbiter(cpShape a, cpShape b)
        {
            Init(a, b);
        }

        private void Init(cpShape a, cpShape b)
        {

            thread_a = new cpArbiterThread();
            thread_b = new cpArbiterThread();

            this.handler = new cpCollisionHandler();
            this.swappedColl = false;

            this.e = 0.0f;
            this.u = 0.0f;
            this.surface_vr = cpVect.ZERO;

            this.contacts = new List<cpContact>();

            this.a = a; this.body_a = a.body;
            this.b = b; this.body_b = b.body;

            this.thread_a.next = null;
            this.thread_b.next = null;
            this.thread_a.prev = null;
            this.thread_b.prev = null;

            this.stamp = 0;

            this.state = cpArbiterState.cpArbiterStateFirstColl;

            this.data = null;
        }



        // Get the relative surface velocity of the two shapes in contact.
        public cpVect GetSurfaceVelocity()
        {
            return surface_vr.Multiply(swappedColl ? -1.0f : 1.0f); //  cpVect.cpvmult(surface_vr,);
        }

        // Override the relative surface velocity of the two shapes in contact.
        // By default this is calculated to be the difference of the two
        // surface velocities clamped to the tangent plane.
        public void SetSurfaceVelocity(cpVect vr)
        {
            surface_vr = vr.Multiply(swappedColl ? -1.0f : 1.0f); // cpvmult(vr,);
        }


        /// Calculate the total impulse that was applied by this arbiter.
        /// This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.

        /// Calculate the total impulse that was applied by this arbiter.
        /// This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
        public cpVect TotalImpulse()
        {
            var sum = new cpVect(0, 0);

            for (int i = 0, count = GetCount(); i < count; i++)
            {
                var con = contacts[i];
                // sum.Add(con.n.Multiply(con.jnAcc));
                sum = sum.Add(con.n.Multiply(con.jnAcc));
            }

            return this.swappedColl ? sum : sum.Neg();
        }

        /// Calculate the total impulse including the friction that was applied by this arbiter.
        /// This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
        public cpVect TotalImpulseWithFriction()
        {
            var sum = cpVect.ZERO;

            for (int i = 0, count = GetCount(); i < count; i++)
            {
                var con = contacts[i];
                sum = sum.Add(new cpVect(con.jnAcc, con.jtAcc).Rotate(con.n));
            }

            return this.swappedColl ? sum : sum.Neg();
        }


        /// Calculate the amount of energy lost in a collision including static, but not dynamic friction.
        /// This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
        public float TotalKE()
        {

            float eCoef = (1 - e) / (1 + e);
            float sum = 0.0f;

            //cpContact contacts = contacts;

            for (int i = 0, count = contacts.Count; i < count; i++)
            {

                cpContact con = contacts[i];
                float jnAcc = con.jnAcc;
                float jtAcc = con.jtAcc;

                sum += eCoef * jnAcc * jnAcc / con.nMass + jtAcc * jtAcc / con.tMass;
            }

            return sum;
            //return -1;
        }


        /// Causes a collision pair to be ignored as if you returned false from a begin callback.
        /// If called from a pre-step callback, you will still need to return false
        /// if you want it to be ignored in the current step.
        public void Ignore()
        {
            state = cpArbiterState.cpArbiterStateIgnore;
        }

        /// Return the colliding shapes involved for this arbiter.
        /// The order of their cpSpace.collision_type values will match
        /// the order set when the collision handler was registered.
        public void GetShapes(out cpShape shapeA, out cpShape shapeB)
        {
            if (swappedColl)
            {
                shapeA = b;
                shapeB = a;
            }
            else
            {
                shapeA = a;
                shapeB = b;
            }
        }


        /// Return the colliding bodies involved for this arbiter.
        /// The order of the cpSpace.collision_type the bodies are associated with values will match
        /// the order set when the collision handler was registered.
        public void GetBodies(out cpBody a, out cpBody b)
        {
            cpShape shape_a, shape_b;
            GetShapes(out shape_a, out shape_b);

            a = shape_a.body;
            b = shape_b.body;
        }

        /// Return a contact set from an arbiter.
        public List<cpContact> GetContactPointSet()
        {
            List<cpContact> set = new List<cpContact>();
            foreach (var item in contacts)
                set.Add(item.Clone());
            return set;
        }

        /// Replace the contact point set for an arbiter.
        /// This can be a very powerful feature, but use it with caution!
        public void SetContactPointSet(List<cpContact> set)
        {

            cpEnvironment.AssertHard(set.Count == numContacts, "The number of contact points cannot be changed.");
            cpContact tmp;
            for (int i = 0; i < set.Count; i++)
            {
                tmp = contacts[i];
                tmp.p = set[i].p;
                tmp.n = set[i].n;
                tmp.dist = set[i].dist;
            }

        }

        /// Returns true if this is the first step a pair of objects started colliding.
        public bool IsFirstContact()
        {
            return state == cpArbiterState.cpArbiterStateFirstColl;
        }

        /// Get the number of contact points for this arbiter.
        public int GetCount()
        {
            // Return 0 contacts if we are in a separate callback.
            return (state != cpArbiterState.cpArbiterStateCached ? numContacts : 0);
            //return contacts.Count;
        }


        /// Get the position of the @c ith contact point.
        public cpVect GetPoint(int i)
        {
            cpEnvironment.AssertHard(0 <= i && i < GetCount(), "Index error: The specified contact index is invalid for this arbiter");
            return contacts[i].p;
            // return contacts[i].point;
        }

        /// Get the normal of the @c ith contact point.
        public cpVect GetNormal(int i)
        {
            cpEnvironment.AssertHard(0 <= i && i < GetCount(), "Index error: The specified contact index is invalid for this arbiter");

            var n = this.contacts[i].n;
            return this.swappedColl ? n.Neg() : n;
        }


        /// Get the depth of the @c ith contact point.
        public float GetDepth(int i)
        {
            // return this.contacts[i].dist;
            cpEnvironment.AssertHard(0 <= i && i < GetCount(), "Index error: The specified contact index is invalid for this arbiter");
            return contacts[i].dist;

        }


        public void update(List<cpContact> contacts, cpCollisionHandler handler, cpShape a, cpShape b)
        {
            //throw new NotImplementedException();

            if (this.contacts != null)
            {

                // Iterate over the possible pairs to look for hash value matches.
                for (int i = 0; i < this.contacts.Count; i++)
                {
                    cpContact old = this.contacts[i];

                    for (int j = 0; j < this.contacts.Count; j++)
                    {
                        cpContact new_contact = this.contacts[j];

                        // This could trigger false positives, but is fairly unlikely nor serious if it does.
                        if (new_contact.hash == old.hash)
                        {
                            // Copy the persistant contact information.
                            new_contact.jnAcc = old.jnAcc;
                            new_contact.jtAcc = old.jtAcc;
                        }
                    }
                }

            }

            this.contacts = contacts;

            this.handler = handler;
            this.swappedColl = (a.collision_type != handler.a);

            this.e = a.e * b.e;
            this.u = a.u * b.u;

            this.surface_vr = cpVect.cpvsub(a.surface_v, b.surface_v);

            // For collisions between two similar primitive types, the order could have been swapped.
            this.a = a; this.body_a = a.body;
            this.b = b; this.body_b = b.body;

            // mark it as new if it's been cached
            if (this.state == cpArbiterState.cpArbiterStateCached) this.state = cpArbiterState.cpArbiterStateFirstColl;

        }

        public void preStep(float dt, float slop, float bias)
        {
            var a = this.body_a;
            var b = this.body_b;

            for (var i = 0; i < this.contacts.Count; i++)
            {
                var con = this.contacts[i];

                // Calculate the offsets.
                con.r1 = cpVect.cpvsub(con.p, a.Position);
                con.r2 = cpVect.cpvsub(con.p, b.Position);

                // Calculate the mass normal and mass tangent.
                con.nMass = 1 / cpEnvironment.k_scalar(a, b, con.r1, con.r2, con.n);
                con.tMass = 1 / cpEnvironment.k_scalar(a, b, con.r1, con.r2, cpVect.cpvperp(con.n));

                // Calculate the target bias velocity.
                con.bias = -bias * Math.Min(0, con.dist + slop) / dt;
                con.jBias = 0;

                // Calculate the target bounce velocity.
                con.bounce = cpEnvironment.normal_relative_velocity(a, b, con.r1, con.r2, con.n) * this.e;
            }
        }

        // TODO is it worth splitting velocity/position correction?
        public void ApplyCachedImpulse(float dt_coef)
        {
            if (this.IsFirstContact()) return;

            var a = this.body_a;
            var b = this.body_b;

            for (var i = 0; i < this.contacts.Count; i++)
            {
                var con = this.contacts[i];

                //var j = vrotate(con.n, new Vect(con.jnAcc, con.jtAcc));

                cpVect j = con.n.Rotate(new cpVect(con.jnAcc, con.jtAcc));  // cpvrotate(con.normal, cpv(con.jnAcc, con.jtAcc));

                cpEnvironment.apply_impulses(a, b, con.r1, con.r2, j.Multiply(dt_coef));//  cpEnvironment.cpvmult(j, dt_coef));
                //var nx = con.normal.x;
                //var ny = con.normal.y;
                //var jx = nx * con.jnAcc - ny * con.jtAcc;
                //var jy = nx * con.jtAcc + ny * con.jnAcc;
                //apply_impulses(a, b, con.r1, con.r2, vmult(j, dt_coef));
                //cpEnvironment.apply_impulses(a, b, con.r1, con.r2, jx * dt_coef, jy * dt_coef);
            }
        }

        public void ApplyImpulse(float dt)
        {

            cpEnvironment.numApplyImpulse++;
            //if (!this.contacts) { throw new Error('contacts is undefined'); }
            var a = this.body_a;
            var b = this.body_b;
            var surface_vr = this.surface_vr;
            var friction = this.u;

            for (var i = 0; i < this.contacts.Count; i++)
            {
                cpEnvironment.numApplyContact++;
                var con = this.contacts[i];
                var nMass = con.nMass;
                var n = con.n;
                var r1 = con.r1;
                var r2 = con.r2;

                //var vr = relative_velocity(a, b, r1, r2);
                var vrx = b.Vel.x - r2.y * b.w - (a.Vel.x - r1.y * a.w);
                var vry = b.Vel.y + r2.x * b.w - (a.Vel.y + r1.x * a.w);

                //var vb1 = vadd(vmult(vperp(r1), a.w_bias), a.v_bias);
                //var vb2 = vadd(vmult(vperp(r2), b.w_bias), b.v_bias);
                //var vbn = vdot(vsub(vb2, vb1), n);

                var vbn = n.x * (b.v_bias.x - r2.y * b.w_bias - a.v_bias.x + r1.y * a.w_bias) +
                        n.y * (r2.x * b.w_bias + b.v_bias.y - r1.x * a.w_bias - a.v_bias.y);

                var vrn = cpVect.cpvdot2(vrx, vry, n.x, n.y);
                //var vrt = vdot(vadd(vr, surface_vr), vperp(n));
                var vrt = cpVect.cpvdot2(vrx + surface_vr.x, vry + surface_vr.y, -n.y, n.x);

                var jbn = (con.bias - vbn) * nMass;
                var jbnOld = con.jBias;
                con.jBias = Math.Max(jbnOld + jbn, 0);

                var jn = -(con.bounce + vrn) * nMass;
                var jnOld = con.jnAcc;
                con.jnAcc = Math.Max(jnOld + jn, 0);

                var jtMax = friction * con.jnAcc;
                var jt = -vrt * con.tMass;
                var jtOld = con.jtAcc;
                con.jtAcc = cpEnvironment.cpclamp(jtOld + jt, -jtMax, jtMax);

                //apply_bias_impulses(a, b, r1, r2, vmult(n, con.jBias - jbnOld));
                var bias_x = n.x * (con.jBias - jbnOld);
                var bias_y = n.y * (con.jBias - jbnOld);
                cpEnvironment.apply_bias_impulse(a, -bias_x, -bias_y, r1);
                cpEnvironment.apply_bias_impulse(b, bias_x, bias_y, r2);

                //apply_impulses(a, b, r1, r2, vrotate(n, new Vect(con.jnAcc - jnOld, con.jtAcc - jtOld)));
                var rot_x = con.jnAcc - jnOld;
                var rot_y = con.jtAcc - jtOld;

                // Inlining apply_impulses decreases speed for some reason :/
                cpEnvironment.apply_impulses(a, b, r1, r2, n.x * rot_x - n.y * rot_y, n.x * rot_y + n.y * rot_x);
            }
        }

        public void Unthread()
        {
            Unthread(body_a);
            Unthread(body_b);
        }

        public cpArbiterThread ThreadForBody(cpBody body)
        {
            return (body_a == body ? thread_a : thread_b);
        }


        // TODO make this generic so I can reuse it for constraints also.
        public void Unthread(cpBody body)
        {
            cpArbiterThread thread = ThreadForBody(body);
            cpArbiter prev = thread.prev;
            cpArbiter next = thread.next;

            if (prev != null)
            {
                prev.ThreadForBody(body).next = next;
            }
            else if (body.arbiterList == this)
            {
                // IFF prev is NULL and body->arbiterList == arb, is arb at the head of the list.
                // This function may be called for an arbiter that was never in a list.
                // In that case, we need to protect it from wiping out the body->arbiterList pointer.
                body.arbiterList = next;
            }

            if (next != null)
                next.ThreadForBody(body).prev = prev;

            thread.prev = null;
            thread.next = null;
        }

        public cpArbiter Next(cpBody body)
        {
            return (body_a == body ? thread_a.next : thread_b.next);
        }

        /// Return the colliding shapes involved for this arbiter.
        /// The order of their cpSpace.collision_type values will match
        /// the order set when the collision handler was registered.
        public bool cachedArbitersFilter(arbiterFilterContext context)
        {

            cpShape shape = context.shape;
            cpBody body = context.body;

            // Match on the filter shape, or if it's null the filter body
            if (
                (body == body_a && (shape == a || shape == null)) ||
                (body == body_b && (shape == b || shape == null))
            )
            {
                // Call separate when removing shapes.
                if (shape != null && state != cpArbiterState.cpArbiterStateCached)
                    CallSeparate(context.space);

                Unthread();

                return false;
            }

            return true;
        }

        public void CallSeparate(cpSpace space)
        {
            // The handler needs to be looked up again as the handler cached on the arbiter may have been deleted since the last step.
            var handler = space.LookupHandler(this.a.collision_type, this.b.collision_type);
            handler.separate(this, space, null);

        }


        public cpBB bb
        {
            get
            {
                return null;
            }
            set
            {

            }
        }
    };



}


