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
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{

    public class cpContactBufferHeader
    {
        public int stamp;
        public cpContactBufferHeader next;
        public int numContacts;
    };


    public partial class cpSpace
    {

        //MARK: Contact Buffer Functions

        public static void PostStepDoNothing(object obj, object data) { }

        //public cpPostStepCallback? GetPostStepCallback(int key)
        //{
        //    return postStepCallbacks.Find(k => k.key == key);
        //}

        public bool AddPostStepCallback(cpPostStepFunc func, int key, object data)
        {
            cpEnvironment.AssertWarn(locked,
               "Adding a post-step callback when the space is not locked is unnecessary. Post-step callbacks will not called until the end of the next call to cpSpaceStep() or the next query.");

            if (!GetPostStepCallback(key).HasValue)
            {
                cpPostStepCallback callback = new cpPostStepCallback(); // (cpPostStepCallback *)cpcalloc(1, sizeof(cpPostStepCallback));
                callback.func = (func != null ? func : PostStepDoNothing);
                callback.key = key;
                callback.data = data;

                PostStepCallbacks.Add(callback);
                //cpArrayPush(, callback);
                return true;
            }
            else
            {
                return false;
            }
        }

        //MARK: Locking Functions

        public void Lock()
        {
            locked = true;
        }

        public void Unlock(bool runPostStep)
        {

            locked = false;
            cpEnvironment.AssertHard(locked, "Internal Error: Space lock underflow.");

            if (!locked)
            {
                // = space.rousedBodies;

                List<object> deleteCache = new List<object>();

                foreach (var item in rousedBodies)
                {
                    ActivateBody(item);
                    deleteCache.Add(item);

                }

                foreach (var item in deleteCache)
                    rousedBodies.Remove((cpBody)item);

                //space.rousedBodies.Remove( space.rousedBodies.)
                if (!locked && runPostStep && !skipPostStep)
                {
                    skipPostStep = true;

                    deleteCache.Clear();

                    foreach (var callback in PostStepCallbacks)
                    {
                        if (callback.func != null)
                        {
                            callback.func(this, callback.key, callback.data);
                        }
                        deleteCache.Add(callback);
                    }

                    foreach (var item in deleteCache)
                        PostStepCallbacks.Remove((cpPostStepCallback)item);

                    skipPostStep = false;


                }
            }

        }

        static bool QueryReject(cpShape a, cpShape b)
        {
            return (
                // BBoxes must overlap
                !a.bb.Intersects(b.bb)
                // Don't collide shapes attached to the same body.
                || a.body == b.body
                // Don't collide objects in the same non-zero group
                || (a.group != 0 && a.group == b.group)
                // Don't collide objects that don't share at least on layer.
                || !(a.layers != 0 & b.layers != 0)
                // Don't collide infinite mass objects
                || (a.body.Mass == cpEnvironment.INFINITY_FLOAT && b.body.Mass == cpEnvironment.INFINITY_FLOAT)
            );
        }

        // Callback from the spatial hash.
        public List<cpContact> CollideShapes(cpShape a, cpShape b)
        {
            cpEnvironment.AssertWarn((a as ICollisionShape).collisionCode <= (b as ICollisionShape).collisionCode, "Collided shapes must be sorted by type");
            return (a as ICollisionShape).collisionTable[(b as ICollisionShape).collisionCode](a, b);
            //return null;

        }

        // Hashset filter func to throw away old arbiters.
        public bool ArbiterSetFilter(cpArbiter arb)
        {
            int ticks = stamp - arb.stamp;

            cpBody a = arb.body_a, b = arb.body_b;

            // TODO should make an arbiter state for this so it doesn't require filtering arbiters for dangling body pointers on body removal.
            // Preserve arbiters on sensors and rejected arbiters for sleeping objects.
            // This prevents errant separate callbacks from happenening.
            if (
                (a.IsStatic() || a.IsSleeping()) && (b.IsStatic() || b.IsSleeping())
            )
            {
                return true;
            }

            // Arbiter was used last frame, but not this one
            if (ticks >= 1 && arb.state != cpArbiterState.cpArbiterStateCached)
            {

                arb.CallSeparate(this);
                arb.state = cpArbiterState.cpArbiterStateCached;
            }

            if (ticks >= collisionPersistence)
            {
                arb.contacts = null;
                //arb.numContacts = 0;

                //pooledArbiters.Add(arb);
                return false;
            }

            return true;
        }

        //MARK: All Important cpSpaceStep() Function

        public void ShapeUpdateFunc(cpShape shape, object unused)
        {
            shape.Update(shape.body.Position, shape.body.Rotation);
        }

        public void Step(float dt)
        {
            // don't step if the timestep is 0!
            if (dt == 0.0f) return;

            cpEnvironment.step++;
            Console.WriteLine(cpEnvironment.step);
            stamp++;

            float prev_dt = curr_dt;
            curr_dt = dt;

            // Reset and empty the arbiter lists.
            for (int i = 0; i < arbiters.Count; i++)
            {
                cpArbiter arb = (cpArbiter)arbiters[i];
                arb.state = cpArbiterState.cpArbiterStateNormal;

                // If both bodies are awake, unthread the arbiter from the contact graph.
                if (!arb.body_a.IsSleeping() && !arb.body_b.IsSleeping())
                {
                    arb.Unthread();
                }
            }

            Lock();
            {
                // Integrate positions
                foreach (var body in bodies)
                    body.position_func(body, dt);

                this.activeShapes.Each((o) => UpdateFunc(o as cpShape));
                this.activeShapes.ReindexQuery((a, b) => CollideShapes(a as cpShape, b as cpShape));

            } Unlock(false);

            // Rebuild the contact graph (and detect sleeping components if sleeping is enabled)

            ProcessComponents(dt);

            Lock();
            {


                List<int> safeDeleteArray = new List<int>();

                // Clear out old cached arbiters and call separate callbacks
                foreach (var item in cachedArbiters.leaves)
                    if (!ArbiterSetFilter( item.Value.obj as cpArbiter ))
                        safeDeleteArray.Add(item.Key);

                foreach (var item in safeDeleteArray)
                    cachedArbiters.Remove(item);

                // cpHashSetFilter(cachedArbiters, ArbiterSetFilter, space);

                // Prestep the arbiters and constraints.
                float slop = collisionSlop;
                float biasCoef = 1.0f - cpEnvironment.cpfpow(collisionBias, dt);
                for (int i = 0; i < arbiters.Count; i++)
                {
                    arbiters[i].preStep(dt, slop, biasCoef);
                }

                foreach (var constraint in constraints)
                {
                    constraint.PreSolve(this);
                    constraint.PreStep(dt);
                }

                // Integrate velocities.
                float damping = cpEnvironment.cpfpow(this.damping, dt);
                for (int i = 0; i < bodies.Count; i++)
                {
                    // cpBody* body = (cpBody);
                    bodies[i].velocity_func(gravity, damping, dt);
                }

                // Apply cached impulses
                float dt_coef = (prev_dt == 0.0f ? 0.0f : dt / prev_dt);

                foreach (var arbiter in arbiters)
                    arbiter.ApplyCachedImpulse(dt_coef);

                foreach (var constraint in constraints)
                {
                    constraint.ApplyCachedImpulse(dt_coef);
                }


                // Run the impulse solver.
                for (int i = 0; i < iterations; i++)
                {
                    for (int j = 0; j < arbiters.Count; j++)
                    {
                        arbiters[j].ApplyImpulse(dt); //   cpArbiterApplyImpulse((cpArbiter));
                    }

                    foreach (var constraint in constraints)
                    {
                        constraint.ApplyImpulse(dt);
                    }

                }

                // Run the constraint post-solve callbacks
                //cpConstraintPostSolveFunc postSolve;

                foreach (var constraint in constraints)
                    constraint.postSolve(this);

                for (int i = 0; i < arbiters.Count; i++)
                {
                    arbiters[i].handler.postSolve(arbiters[i], this, null);
                }

                // run the post-solve callbacks
                //cpCollisionHandler handler;
                //foreach (var arb in arbiters)
                //{
                //    handler = arb.handler;

                //    handler.postSolve(arb, this, handler.data);
                //}
            }
            Unlock(true);
        }

        private void UpdateFunc(cpShape shape)
        {
            var body = shape.body;
            shape.Update(body.Position, body.Rotation);
        }


    }
}


//#define CP_CONTACTS_BUFFER_SIZE ((CP_BUFFER_BYTES - sizeof(cpContactBufferHeader))/sizeof(cpContact))
//struct cpContactBuffer {
//    cpContactBufferHeader header;
//    cpContact contacts[CP_CONTACTS_BUFFER_SIZE];
//} cpContactBuffer;

//public static cpContactBufferHeader  cpSpaceAllocContactBuffer(cpSpace space)
//{
//    cpContactBuffer buffer = (cpContactBuffer)cpcalloc(1, sizeof(cpContactBuffer));
//    cpArrayPush(space.allocatedBuffers, buffer);
//    return (cpContactBufferHeader *)buffer;
//}

//static cpContactBufferHeader cpContactBufferHeaderInit(cpContactBufferHeader header, int stamp, cpContactBufferHeader splice)
//{
//    header.stamp = stamp;
//    header.next = (splice != null ? splice.next : header);
//    header.numContacts = 0;
//    return header;
//}

//void cpSpacePushFreshContactBuffer(cpSpace space)
//{
//    int stamp = space.stamp;

//    //List<cpContactBufferHeader> head = ;

//    if(space.contactBuffersHead!=null){
//        // No buffers have been allocated, make one
//        space.contactBuffersHead = new List<cpContact>();
//    } else if(stamp - head.next.stamp > space.collisionPersistence){
//        // The tail buffer is available, rotate the ring
//    cpContactBufferHeader tail = head.next;
//        space.contactBuffersHead = cpContactBufferHeaderInit(tail, stamp, tail);
//    } else {
//        // Allocate a new buffer and push it into the ring
//        cpContactBufferHeader *buffer = cpContactBufferHeaderInit(cpSpaceAllocContactBuffer(space), stamp, head);
//        space.contactBuffersHead = head.next = buffer;
//    }
//}


//cpContact cpContactBufferGetArray(cpSpace space)
//{
//    if (space.contactBuffersHead.numContacts + CP_MAX_CONTACTS_PER_ARBITER > CP_CONTACTS_BUFFER_SIZE)
//    {
//        // contact buffer could overflow on the next collision, push a fresh one.
//        cpSpacePushFreshContactBuffer(space);
//    }

//    cpContactBufferHeader head = space.contactBuffersHead;
//    return ((cpContactBuffer)head).contacts + head.numContacts;
//}

//void
//cpSpacePushContacts(cpSpace space, int count)
//{
//    cpAssertHard(count <= CP_MAX_CONTACTS_PER_ARBITER, "Internal Error: Contact buffer overflow!");
//    space.contactBuffersHead.numContacts += count;
//}

//static void cpSpacePopContacts(cpSpace space, int count)
//{
//    space.contactBuffersHead.numContacts -= count;
//}

//MARK: Collision Detection Functions

//static void cpSpaceArbiterSetTrans(cpShape shapes, cpSpace space)
//{
//    if (space.pooledArbiters.Count == 0)
//    {
//        // arbiter pool is exhausted, make more
//        int count = CP_BUFFER_BYTES / sizeof(cpArbiter);
//        cpAssertHard(count, "Internal Error: Buffer size too small.");

//        cpArbiterbuffer = (cpArbiter)cpcalloc(1, CP_BUFFER_BYTES);
//        cpArrayPush(space.allocatedBuffers, buffer);

//        for (int i = 0; i < count; i++) cpArrayPush(space.pooledArbiters, buffer + i);
//    }

//    return cpArbiterInit(space.pooledArbiters, shapes[0], shapes[1]);
//}
