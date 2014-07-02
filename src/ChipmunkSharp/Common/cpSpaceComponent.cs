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
        //MARK: Sleeping Functions

        public void ActivateBody(cpBody body)
        {
            cpEnvironment.cpAssertHard(!body.IsRogue(), "Internal error: Attempting to activate a rogue body.");

            if (locked)
            {
                // cpSpaceActivateBody() is called again once the space is unlocked
                if (!rousedBodies.Contains(body))
                    rousedBodies.Add(body); //   cpArrayPush(space->rousedBodies, );
            }
            else
            {

                cpEnvironment.cpAssertSoft(body.node.root == null && body.node.next == null, "Internal error: Activating body non-NULL node pointers.");
                bodies.Add(body);
                //cpArrayPush(space->bodies, body);

                body.EachShape((bod, s, o) =>
                {
                    staticShapes.Remove(s.hashid);
                    activeShapes.Add(s.hashid, s);

                }, null);

                //CP_BODY_FOREACH_SHAPE(body, shape){

                body.EachArbiter((bod, arb, o) =>
                {
                    cpBody bodyA = arb.body_a;

                    // Arbiters are shared between two bodies that are always woken up together.
                    // You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
                    // The edge case is when static bodies are involved as the static bodies never actually sleep.
                    // If the static body is bodyB then all is good. If the static body is bodyA, that can easily be checked.
                    if (body == bodyA || bodyA.IsStatic())
                    {

                        //int numContacts = arb.numContacts;
                        //cpContact contacts = arb.contacts;
                        // Restore contact values back to the space's contact buffer memory
                        //arb.contacts = contactBuffersHead;
                        //memcpy(arb.contacts, contacts, numContacts*sizeof(cpContact));
                        //(this, numContacts);


                        // Reinsert the arbiter into the arbiter cache
                        cpShape a = arb.a, b = arb.b;
                        List<cpShape> shape_pair = new List<cpShape>() { a, b };

                        cachedArbiters.Add(cpEnvironment.CP_HASH_PAIR(a.hashid, b.hashid), arb);

                        //cpHashSetInsert(space->cachedArbiters, arbHashID, shape_pair, arb, NULL);

                        // Update the arbiter's state
                        arb.stamp = stamp;

                        arb.handler = LookupHandler(a.collision_type, b.collision_type);
                        //cpArrayPush(arbiters, arb);
                        arbiters.Add(arb);
                        //cpfree(contacts);
                    }

                }, null);


                body.EachConstraint((bod, c, o) =>
                {
                    cpBody bodyA = c.a;
                    if (body == bodyA || bodyA.IsStatic())
                        constraints.Add(c);


                }, null);


            }
        }



        public void DeactivateBody(cpBody body)
        {
            cpEnvironment.cpAssertHard(!body.IsRogue(), "Internal error: Attempting to deactivate a rouge body.");
            bodies.Remove(body);
            //cpArrayDeleteObj(, body);

            body.EachShape((b, s, d) =>
            {

                activeShapes.Remove(s.hashid);
                staticShapes.Add(s.hashid, s);
                //cpSpatialIndexRemove(, shape, );
                //cpSpatialIndexInsert(space->staticShapes, shape, shape->hashid);

            }, null);

            body.EachArbiter((b, arb, d) =>
            {

                cpBody bodyA = arb.body_a;
                if (body == bodyA || bodyA.IsStatic())
                {

                    UncacheArbiter(arb);


                    // Save contact values to a new block of memory so they won't time out
                    //size_t bytes = arb->numContacts*sizeof(cpContact);
                    //	cpContact *contacts = (cpContact *)cpcalloc(1, bytes);
                    //memcpy(contacts, arb->contacts, bytes);

                    List<cpContact> contacts = new List<cpContact>();

                    foreach (var item in arb.contacts)
                        contacts.Add(item);

                    arb.contacts = contacts;
                }
                //cpSpatialIndexRemove(, shape, );
                //cpSpatialIndexInsert(space->staticShapes, shape, shape->hashid);

            }, null);

            //CP_BODY_FOREACH_SHAPE(body, shape){

            //}

            //CP_BODY_FOREACH_ARBITER(body, arb){

            //}
            body.EachConstraint(
                (b, con, d) =>
                {

                    cpBody bodyA = con.a;
                    if (body == bodyA || bodyA.IsStatic())
                        constraints.Remove(con);

                }, null);


        }


        //public static void ComponentActivate(cpBody root)
        //{
        //    if (!root || !cpBodyIsSleeping(root)) return;
        //    cpAssertHard(!cpBodyIsRogue(root), "Internal Error: ComponentActivate() called on a rogue body.");

        //    cpSpace space = root.space;
        //    cpBody body = root;
        //    while (body)
        //    {
        //        cpBody next = body.node.next;

        //        body.node.idleTime = 0.0f;
        //        body.node.root = null;
        //        body.node.next = null;
        //        cpSpaceActivateBody(space, body);

        //        body = next;
        //    }

        //    cpArrayDeleteObj(space.sleepingComponents, root);
        //}

        public static void ComponentAdd(cpBody root, cpBody body)
        {
            body.node.root = root;

            if (body != root)
            {
                body.node.next = root.node.next;
                root.node.next = body;
            }
        }

        public void ProcessComponents(float dt)
        {
            bool sleep = (sleepTimeThreshold != cpEnvironment.INFINITY_FLOAT);
            //cpArray *bodies = space.bodies;

#if DEBUG

            foreach (var body in bodies)
            {
                cpEnvironment.cpAssertSoft(body.node.next == null, "Internal Error: Dangling next pointer detected in contact graph.");
                cpEnvironment.cpAssertSoft(body.node.root == null, "Internal Error: Dangling root pointer detected in contact graph.");
            }

#endif
            // Calculate the kinetic energy of all the bodies.
            if (sleep)
            {
                float dv = idleSpeedThreshold;
                float dvsq = (dv != 0 ? dv * dv : cpVect.cpvlengthsq(gravity) * dt * dt);

                // update idling and reset component nodes

                foreach (var body in bodies)
                {
                    // Need to deal with infinite mass objects
                    float keThreshold = (dvsq != 0 ? body.Mass * dvsq : 0.0f);
                    body.node.idleTime = (body.KineticEnergy() > keThreshold ? 0.0f : body.node.idleTime + dt);
                }

            }

            // Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
            //cpArray *arbiters = space.arbiters;

            foreach (var arb in arbiters)
            {
                cpBody a = arb.body_a, b = arb.body_b;

                if (sleep)
                {
                    if ((b.IsRogue() && !b.IsStatic()) || a.IsSleeping()) a.Activate();// cpBodyActivate(a);
                    if ((a.IsRogue() && !a.IsStatic()) || b.IsSleeping()) b.Activate();// cpBodyActivate(b);
                }

                a.PushArbiter(arb);
                b.PushArbiter(arb);

            }

            if (sleep)
            {
                // Bodies should be held active if connected by a joint to a non-static rouge body.
                //cpArray *constraints = space.constraints;

                foreach (var constraint in constraints)
                {
                    cpBody a = constraint.a, b = constraint.b;

                    if (b.IsRogue() && !b.IsStatic())
                        a.Activate();
                    if (a.IsRogue() && !a.IsStatic())
                        b.Activate();
                }

                // Generate components and deactivate sleeping ones
                cpBody body;
                for (int i = 0; i < bodies.Count; )
                {
                    body = bodies[i];
                    if (body.ComponentRoot() == null)
                    {
                        // Body not in a component yet. Perform a DFS to flood fill mark 
                        // the component in the contact graph using this body as the root.
                        //FloodFillComponent(body, body);

                        body.FloodFillComponent(body);

                        // Check if the component should be put to sleep.
                        if (!body.ComponentActive(sleepTimeThreshold))
                        {

                            sleepingComponents.Add(body);
                            //cpArrayPush(space.sleepingComponents, body);

                            for (cpBody component = body.node.root; component != null; component = component.node.next)
                                DeactivateBody(component);


                            // cpSpaceDeactivateBody() removed the current body from the list.
                            // Skip incrementing the index counter.
                            continue;
                        }
                    }
                    i++;

                    // Only sleeping bodies retain their component node pointers.
                    body.node.root = null;
                    body.node.next = null;

                }
            }
        }


        //        void cpBodySleepWithGroup(cpBody body, cpBody group){
        //    cpAssertHard(!cpBodyIsRogue(body), "Rogue (and static) bodies cannot be put to sleep.");

        //    cpSpace *space = body.space;
        //    cpAssertHard(!space.locked, "Bodies cannot be put to sleep during a query or a call to cpSpaceStep(). Put these calls into a post-step callback.");
        //    cpAssertHard(group == null || cpBodyIsSleeping(group), "Cannot use a non-sleeping body as a group identifier.");

        //    if(cpBodyIsSleeping(body)){
        //        cpAssertHard(ComponentRoot(body) == ComponentRoot(group), "The body is already sleeping and it's group cannot be reassigned.");
        //        return;
        //    }

        //    CP_BODY_FOREACH_SHAPE(body, shape) cpShapeUpdate(shape, body.p, body.rot);
        //    cpSpaceDeactivateBody(space, body);

        //    if(group){
        //        cpBody root = ComponentRoot(group);

        //        cpComponentNode node = {root, root.node.next, 0.0f};
        //        body.node = node;

        //        root.node.next = body;
        //    } else {
        //        cpComponentNode node = {body, null, 0.0f};
        //        body.node = node;

        //        cpArrayPush(space.sleepingComponents, body);
        //    }

        //    cpArrayDeleteObj(space.bodies, body);
        //}

        public static void activateTouchingHelper(cpShape shape, List<cpContact> points, cpShape other)
        {
            shape.body.Activate();
        }

        public void ActivateShapesTouchingShape(cpShape shape)
        {
            if (sleepTimeThreshold != cpEnvironment.INFINITY_FLOAT)
            {
                ShapeQuery(shape, (s, p, o) =>
                {
                    activateTouchingHelper(shape, p, (cpShape)o);
                }, shape);
            }
        }


    }
}

//        void ActivateBody(cpBody body)
//{
//    cpEnvironment.cpAssertHard(!body.IsRogue(), "Internal error: Attempting to activate a rogue body.");

//    if(locked){
//        // cpSpaceActivateBody() is called again once the space is unlocked
//        if(!cpArrayContains(space.rousedBodies, body)) cpArrayPush(space.rousedBodies, body);
//    } else {
//        cpAssertSoft(body.node.root == null && body.node.next == null, "Internal error: Activating body non-null node pointers.");
//        cpArrayPush(space.bodies, body);

//        CP_BODY_FOREACH_SHAPE(body, shape){
//            cpSpatialIndexRemove(space.staticShapes, shape, shape.hashid);
//            cpSpatialIndexInsert(space.activeShapes, shape, shape.hashid);
//        }

//        CP_BODY_FOREACH_ARBITER(body, arb){
//            cpBody bodyA = arb.body_a;

//            // Arbiters are shared between two bodies that are always woken up together.
//            // You only want to restore the arbiter once, so bodyA is arbitrarily chosen to own the arbiter.
//            // The edge case is when static bodies are involved as the static bodies never actually sleep.
//            // If the static body is bodyB then all is good. If the static body is bodyA, that can easily be checked.
//            if(body == bodyA || cpBodyIsStatic(bodyA)){
//                int numContacts = arb.numContacts;
//                cpContact *contacts = arb.contacts;

//                // Restore contact values back to the space's contact buffer memory
//                arb.contacts = cpContactBufferGetArray(space);
//                memcpy(arb.contacts, contacts, numContacts*sizeof(cpContact));
//                cpSpacePushContacts(space, numContacts);

//                // Reinsert the arbiter into the arbiter cache
//                cpShape a = arb.a, *b = arb.b;
//                cpShape shape_pair[] = {a, b};
//                cpHashValue arbHashID = CP_HASH_PAIR((cpHashValue)a, (cpHashValue)b);
//                cpHashSetInsert(space.cachedArbiters, arbHashID, shape_pair, arb, null);

//                // Update the arbiter's state
//                arb.stamp = space.stamp;
//                arb.handler = cpSpaceLookupHandler(space, a.collision_type, b.collision_type);
//                cpArrayPush(space.arbiters, arb);

//                cpfree(contacts);
//            }
//        }

//        CP_BODY_FOREACH_CONSTRAINT(body, constraint){
//            cpBody bodyA = constraint.a;
//            if(body == bodyA || cpBodyIsStatic(bodyA)) cpArrayPush(space.constraints, constraint);
//        }
//    }
//}
//        static void cpSpaceDeactivateBody(cpSpace space, cpBody body)
//{
//    cpAssertHard(!cpBodyIsRogue(body), "Internal error: Attempting to deactivate a rouge body.");

//    cpArrayDeleteObj(space.bodies, body);

//    CP_BODY_FOREACH_SHAPE(body, shape){
//        cpSpatialIndexRemove(space.activeShapes, shape, shape.hashid);
//        cpSpatialIndexInsert(space.staticShapes, shape, shape.hashid);
//    }

//    CP_BODY_FOREACH_ARBITER(body, arb){
//        cpBody bodyA = arb.body_a;
//        if(body == bodyA || cpBodyIsStatic(bodyA)){
//            cpSpaceUncacheArbiter(space, arb);

//            // Save contact values to a new block of memory so they won't time out
//            size_t bytes = arb.numContacts*sizeof(cpContact);
//            cpContact *contacts = (cpContact *)cpcalloc(1, bytes);
//            memcpy(contacts, arb.contacts, bytes);
//            arb.contacts = contacts;
//        }
//    }

//    CP_BODY_FOREACH_CONSTRAINT(body, constraint){
//        cpBody bodyA = constraint.a;
//        if(body == bodyA || cpBodyIsStatic(bodyA)) cpArrayDeleteObj(space.constraints, constraint);
//    }
//}