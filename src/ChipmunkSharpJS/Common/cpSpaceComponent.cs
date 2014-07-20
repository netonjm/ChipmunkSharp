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
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{
    public partial class cpSpace
    {

        public void ActivateShapesTouchingShape(cpShape shape)
        {
            if (sleepTimeThreshold != cp.Infinity)
            {
                shapeQuery(shape, (s, p) =>
                {
                    shape.body.Activate();
                });
            }
        }


        //MARK: Sleeping Functions

        //        public void ActivateBody(cpBody body)
        //        {
        //            cpEnvironment.AssertHard(!body.isRogue(), "Internal error: Attempting to activate a rogue body.");

        //            if (isLocked)
        //            {
        //                // cpSpaceActivateBody() is called again once the space is unlocked
        //                if (!rousedBodies.Contains(body))
        //                    rousedBodies.Add(body); //   cpArrayPush(space->rousedBodies, );
        //            }
        //            else
        //            {

        //                cpEnvironment.AssertSoft(body.node.root == null && body.node.next == null, "Internal error: Activating body non-NULL node pointers.");
        //                bodies.Add(body);
        //                //cpArrayPush(space->bodies, body);

        //                body.eachShape(s =>
        //                {
        //                    staticShapes.Remove(s.hashid);
        //                    activeShapes.Insert(s.hashid, s);

        //                });

        //                //CP_BODY_FOREACH_SHAPE(body, shape){

        //                body.eachArbiter(arb =>
        //                {
        //                    cpBody bodyA = arb.body_a;

        //                    // Arbiters are shared between two bodies that are always woken up together.
        //                    // You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
        //                    // The edge case is when static bodies are involved as the static bodies never actually sleep.
        //                    // If the static body is bodyB then all is good. If the static body is bodyA, that can easily be checked.
        //                    if (body == bodyA || bodyA.isStatic())
        //                    {

        //                        //int numContacts = arb.numContacts;
        //                        //cpContact contacts = arb.contacts;
        //                        // Restore contact values back to the space's contact buffer memory
        //                        //arb.contacts = contactBuffersHead;
        //                        //memcpy(arb.contacts, contacts, numContacts*sizeof(cpContact));
        //                        //(this, numContacts);


        //                        // Reinsert the arbiter into the arbiter cache
        //                        cpShape a = arb.a, b = arb.b;
        //                        List<cpShape> shape_pair = new List<cpShape>() { a, b };

        //                        cachedArbiters.Insert(cpEnvironment.CP_HASH_PAIR(a.hashid, b.hashid), arb);

        //                        //cpHashSetInsert(space->cachedArbiters, arbHashID, shape_pair, arb, NULL);

        //                        // Update the arbiter's state
        //                        arb.stamp = stamp;

        //                        arb.handler = LookupHandler(a.collision_type, b.collision_type);
        //                        //cpArrayPush(arbiters, arb);
        //                        arbiters.Add(arb);
        //                        //cpfree(contacts);
        //                    }

        //                });
        //                body.eachConstraint(c =>
        //                {
        //                    cpBody bodyA = c.a;
        //                    if (body == bodyA || bodyA.isStatic())
        //                        constraints.Add(c);


        //                });


        //            }
        //        }

        //        public void DeactivateBody(cpBody body)
        //        {
        //            cpEnvironment.AssertHard(!body.isRogue(), "Internal error: Attempting to deactivate a rouge body.");
        //            bodies.Remove(body);
        //            //cpArrayDeleteObj(, body);

        //            body.eachShape(s =>
        //            {

        //                activeShapes.Remove(s.hashid);
        //                staticShapes.Insert(s.hashid, s);
        //                //cpSpatialIndexRemove(, shape, );
        //                //cpSpatialIndexInsert(space->staticShapes, shape, shape->hashid);

        //            });

        //            body.eachArbiter(arb =>
        //            {

        //                cpBody bodyA = arb.body_a;
        //                if (body == bodyA || bodyA.isStatic())
        //                {

        //                    uncacheArbiter(arb);


        //                    // Save contact values to a new block of memory so they won't time out
        //                    //size_t bytes = arb->numContacts*sizeof(cpContact);
        //                    //	cpContact *contacts = (cpContact *)cpcalloc(1, bytes);
        //                    //memcpy(contacts, arb->contacts, bytes);

        //                    List<ContactPoint> contacts = new List<ContactPoint>();

        //                    foreach (var item in arb.contacts)
        //                        contacts.Add(item);

        //                    arb.contacts = contacts;
        //                }
        //                //cpSpatialIndexRemove(, shape, );
        //                //cpSpatialIndexInsert(space->staticShapes, shape, shape->hashid);

        //            });

        //            //CP_BODY_FOREACH_SHAPE(body, shape){

        //            //}

        //            //CP_BODY_FOREACH_ARBITER(body, arb){

        //            //}
        //            body.eachConstraint(
        //               con =>
        //               {

        //                   cpBody bodyA = con.a;
        //                   if (body == bodyA || bodyA.isStatic())
        //                       constraints.Remove(con);

        //               });


        //        }




        //        public static void ComponentAdd(cpBody root, cpBody body)
        //        {
        //            body.node.root = root;

        //            if (body != root)
        //            {
        //                body.node.next = root.node.next;
        //                root.node.next = body;
        //            }
        //        }

        //        public void ProcessComponents(float dt)
        //        {
        //            bool sleep = (sleepTimeThreshold != cpEnvironment.Infinity);
        //            //cpArray *bodies = space.bodies;

        //#if DEBUG

        //            foreach (var body in bodies)
        //            {
        //                cpEnvironment.AssertSoft(body.node.next == null, "Internal Error: Dangling next pointer detected in contact graph.");
        //                cpEnvironment.AssertSoft(body.node.root == null, "Internal Error: Dangling root pointer detected in contact graph.");
        //            }

        //#endif
        //            // Calculate the kinetic energy of all the bodies.
        //            if (sleep)
        //            {
        //                float dv = idleSpeedThreshold;
        //                float dvsq = (dv != 0 ? dv * dv : cpVect.cpvlengthsq(gravity) * dt * dt);

        //                // update idling and reset component nodes

        //                foreach (var body in bodies)
        //                {
        //                    // Need to deal with infinite mass objects
        //                    float keThreshold = (dvsq != 0 ? body.Mass * dvsq : 0.0f);
        //                    body.node.idleTime = (body.kineticEnergy() > keThreshold ? 0.0f : body.node.idleTime + dt);
        //                }

        //            }

        //            // Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
        //            //cpArray *arbiters = space.arbiters;

        //            foreach (var arb in arbiters)
        //            {
        //                cpBody a = arb.body_a, b = arb.body_b;

        //                if (sleep)
        //                {
        //                    if ((b.isRogue() && !b.isStatic()) || a.isSleeping()) a.activate();// cpBodyActivate(a);
        //                    if ((a.isRogue() && !a.isStatic()) || b.isSleeping()) b.activate();// cpBodyActivate(b);
        //                }

        //                a.pushArbiter(arb);
        //                b.pushArbiter(arb);

        //            }

        //            if (sleep)
        //            {
        //                // Bodies should be held active if connected by a joint to a non-static rouge body.
        //                //cpArray *constraints = space.constraints;

        //                foreach (var constraint in constraints)
        //                {
        //                    cpBody a = constraint.a, b = constraint.b;

        //                    if (b.isRogue() && !b.isStatic())
        //                        a.activate();
        //                    if (a.isRogue() && !a.isStatic())
        //                        b.activate();
        //                }

        //                // Generate components and deactivate sleeping ones
        //                cpBody body;
        //                for (int i = 0; i < bodies.Count; )
        //                {
        //                    body = bodies[i];
        //                    if (body.componentRoot() == null)
        //                    {
        //                        // Body not in a component yet. Perform a DFS to flood fill mark 
        //                        // the component in the contact graph using this body as the root.
        //                        //FloodFillComponent(body, body);

        //                        body.floodFillComponent(body);

        //                        // Check if the component should be put to sleep.
        //                        if (!body.componentActive(sleepTimeThreshold))
        //                        {

        //                            sleepingComponents.Add(body);
        //                            //cpArrayPush(space.sleepingComponents, body);

        //                            for (cpBody component = body.node.root; component != null; component = component.node.next)
        //                                DeactivateBody(component);


        //                            // cpSpaceDeactivateBody() removed the current body from the list.
        //                            // Skip incrementing the index counter.
        //                            continue;
        //                        }
        //                    }
        //                    i++;

        //                    // Only sleeping bodies retain their component node pointers.
        //                    body.node.root = null;
        //                    body.node.next = null;

        //                }
        //            }
        //        }


        //        public static void activateTouchingHelper(cpShape shape, List<ContactPoint> points, cpShape other)
        //        {
        //            shape.body.activate();
        //        }




    }
}
