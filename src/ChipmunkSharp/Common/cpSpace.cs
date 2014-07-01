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
using System;
using System.Collections.Generic;
using System.Linq;

namespace ChipmunkSharp
{

    public struct spaceShapeContext
    {
        //cpSpaceShapeIteratorFunc func;
        public object data;
    } ;

    public struct cpPostStepCallback
    {
        public cpPostStepFunc func;
        public int key;
        public object data;
    }


    /// Post Step callback function type.
    public delegate void cpPostStepFunc(cpSpace space, object key, object data);
    /// Space/constraint iterator callback function type.
    public delegate void cpSpaceConstraintIteratorFunc(cpConstraint constraint, object data);

    public delegate void cpSpaceArbiterApplyImpulseFunc(cpArbiter arb);

    /// Basic Unit of Simulation in Chipmunk
    /// 
    public partial class cpSpace
    {

        private cpDraw m_debugDraw;


        public static cpCollisionHandler cpDefaultCollisionHandler = new cpCollisionHandler(0, 0, alwaysCollide, alwaysCollide, nothing, nothing, null);

        // public float damping;

        /// Number of iterations to use in the impulse solver to solve contacts.
        public int iterations;

        /// Gravity to pass to rigid bodies when integrating velocity.
        public cpVect gravity;

        /// Damping rate expressed as the fraction of velocity bodies retain each second.
        /// A value of 0.9 would mean that each body's velocity will drop 10% per second.
        /// The default value is 1.0, meaning no damping is applied.
        /// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
        public float damping;

        /// Speed threshold for a body to be considered idle.
        /// The default value of 0 means to let the space guess a good threshold based on gravity.
        public float idleSpeedThreshold;

        /// Time a group of bodies must remain idle in order to fall asleep.
        /// Enabling sleeping also implicitly enables the the contact graph.
        /// The default value of INFINITY disables the sleeping algorithm.
        public float sleepTimeThreshold;

        /// Amount of encouraged penetration between colliding shapes.
        /// Used to reduce oscillating contacts and keep the collision cache warm.
        /// Defaults to 0.1. If you have poor simulation quality,
        /// increase this number as much as possible without allowing visible amounts of overlap.
        public float collisionSlop;

        /// Determines how fast overlapping shapes are pushed apart.
        /// Expressed as a fraction of the error remaining after each second.
        /// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
        public float collisionBias;

        /// Number of frames that contact information should persist.
        /// Defaults to 3. There is probably never a reason to change this value.
        public int collisionPersistence;

        /// Rebuild the contact graph during each step. Must be enabled to use the cpBodyEachArbiter() function.
        /// Disabled by default for a small performance boost. Enabled implicitly when the sleeping feature is enabled.
        public bool enableContactGraph;

        /// User definable data pointer.
        /// Generally this points to your game's controller or game state
        /// class so you can access it when given a cpSpace reference in a callback.
        public object data;

        /// The designated static body for this space.
        /// You can modify this body, or replace it with your own static body.
        /// By default it points to a statically allocated cpBody in the cpSpace struct.
        public cpBody staticBody;
        // public cpBody _staticBody;

        public int stamp;
        public float curr_dt;

        public List<cpBody> bodies;
        public List<cpBody> rousedBodies;
        public List<cpBody> sleepingComponents;

        public cpBBTree staticShapes;
        public cpBBTree activeShapes;

        public List<cpArbiter> arbiters;
        public List<cpContact> contactBuffersHead;
        public cpBBTree cachedArbiters;
        public List<cpArbiter> pooledArbiters;
        public List<cpConstraint> constraints;

        // public cpArray allocatedBuffers;
        public bool locked;
        public cpBBTree collisionHandlers;
        //public cpCollisionHandler defaultHandler;

        public bool skipPostStep;

        public List<cpPostStepCallback> postStepCallbacks;

        public cpCollisionHandler defaultHandler { get; set; }

        //MARK: Contact Set Helpers
        // Equal function for arbiterSet.
        public static bool arbiterSetEql(List<cpShape> shapes, cpArbiter arb)
        {
            cpShape a = shapes[0];
            cpShape b = shapes[1];

            return ((a == arb.a && b == arb.b) || (b == arb.a && a == arb.b));
        }


        public static bool handlerSetEql(cpCollisionHandler check, cpCollisionHandler pair)
        {
            return ((check.a == pair.a && check.b == pair.b) || (check.b == pair.a && check.a == pair.b));
        }


        // Transformation function for collisionHandlers.
        public static cpCollisionHandler handlerSetTrans(cpCollisionHandler handler, object unused)
        {
            // (cpCollisionHandler)cpcalloc(1, sizeof(cpCollisionHandler));
            return handler.Clone();
        }


        static bool cachedArbitersFilter(cpArbiter arb, arbiterFilterContext context)
        {
            cpShape shape = context.shape;
            cpBody body = context.body;


            // Match on the filter shape, or if it's null the filter body
            if (
                (body == arb.body_a && (shape == arb.a || shape == null)) ||
                (body == arb.body_b && (shape == arb.b || shape == null))
            )
            {
                // Call separate when removing shapes.
                if (shape != null && arb.state != cpArbiterState.cpArbiterStateCached)
                    arb.CallSeparate(context.space);

                arb.Unthread();

                context.space.arbiters.Remove(arb);
                context.space.pooledArbiters.Add(arb);

                return false;
            }

            return true;
        }

        void cpSpaceFilterArbiters(cpSpace space, cpBody body, cpShape filter)
        {

            lock (space)
            {
                space.locked = true;

                arbiterFilterContext context = new arbiterFilterContext(space, body, filter);

                //TODO: cpSpaceFilterArbiters
                // cpHashSetFilter(space.cachedArbiters, cachedArbitersFilter, context);
                space.locked = false;
            }


        }



        //MARK: Misc Helper Funcs

        // Default collision functions.
        static bool alwaysCollide(cpArbiter arb, cpSpace space, object data) { return true; }
        static void nothing(cpArbiter arb, cpSpace space, object data) { }

        // function to get the estimated velocity of a shape for the cpBBTree.
        static cpVect shapeVelocityFunc(cpShape shape) { return shape.body.v; }

        static void freeWrap(object ptr, object unused)
        { //cpfree(ptr);
        }




        /// Allocate a cpSpace.

        /// Initialize a cpSpace.
        public cpSpace Init()
        {
#if DEBUG
            bool done = false;
            if (!done)
            {
                Console.WriteLine("Initializing cpSpace - Chipmunk v%s (Debug Enabled)\n", cpEnvironment.cpVersionString);
                Console.WriteLine("Compile with -DNDEBUG defined to disable debug mode and runtime assertion checks\n");
                done = true;
            }
#endif

            iterations = 10;

            gravity = cpVect.ZERO;
            damping = 1.0f;

            collisionSlop = 0.1f;
            collisionBias = cpEnvironment.cpfpow(1.0f - 0.1f, 60.0f);
            collisionPersistence = 3;

            locked = false;
            stamp = 0;


            staticShapes = new cpBBTree(null); // cpBBTree.cpBBTreeNew((cpSpatialIndexBBFunc)cpShapeGetBB, null);
            activeShapes = new cpBBTree(staticShapes);// cpBBTree.cpBBTreeNew((cpSpatialIndexBBFunc)cpShapeGetBB, space.staticShapes);

            //space.activeShapes.set
            //cpBBTree.cpBBTreeSetVelocityFunc(, shapeVelocityFunc);
            //space.activeShapes.Set

            // space.activeShapes = new cpSpatialIndex();

            //allocatedBuffers = cpArrayNew(0);

            bodies = new List<cpBody>();
            sleepingComponents = new List<cpBody>(); // cpArrayNew(0);
            rousedBodies = new List<cpBody>();// cpArrayNew(0);

            sleepTimeThreshold = cpEnvironment.INFINITY_FLOAT;// INFINITY;
            idleSpeedThreshold = 0.0f;
            enableContactGraph = false;

            arbiters = new List<cpArbiter>(); // cpArrayNew(0);
            pooledArbiters = new List<cpArbiter>();// cpArrayNew(0);

            contactBuffersHead = new List<cpContact>();
            cachedArbiters = new cpBBTree(null);  // cpHashSetNew(0, (cpHashSetEqlFunc)arbiterSetEql);

            constraints = new List<cpConstraint>(); // cpArrayNew(0);

            defaultHandler = cpDefaultCollisionHandler;
            collisionHandlers = new cpBBTree(null); // cpHashSetNew(0, (cpHashSetEqlFunc)handlerSetEql);

            collisionHandlers.SetDefaultValue(cpDefaultCollisionHandler);

            //cpHashSetSetDefaultValue(space.collisionHandlers, cpDefaultCollisionHandler);

            postStepCallbacks = new List<cpPostStepCallback>();
            skipPostStep = false;
            staticBody = new cpBody();

            return this;
        }
        /// Allocate and initialize a cpSpace.
        public cpSpace cpSpaceNew()
        {
            var space = new cpSpace();
            return space.Init();
            //return cpSpaceInit();

        }

        /// Destroy a cpSpace.
        public void cpSpaceDestroy(cpSpace space)
        {
            //TODO:
            //cpSpaceEachBody(space, (cpSpaceBodyIteratorFunc)cpBodyActivate, null);

            //cpSpatialIndexFree(space.staticShapes);
            //cpSpatialIndexFree(space.activeShapes);


            //cpArrayFree(space.bodies);
            //cpArrayFree(space.sleepingComponents);
            //cpArrayFree(space.rousedBodies);

            //cpArrayFree(space.constraints);

            //cpHashSetFree(space.cachedArbiters);

            //cpArrayFree(space.arbiters);
            //cpArrayFree(space.pooledArbiters);

            //if (space.allocatedBuffers)
            //{
            //    cpArrayFreeEach(space.allocatedBuffers, cpfree);
            //    cpArrayFree(space.allocatedBuffers);
            //}

            //if (space.postStepCallbacks)
            //{
            //    cpArrayFreeEach(space.postStepCallbacks, cpfree);
            //    cpArrayFree(space.postStepCallbacks);
            //}

            //if (space.collisionHandlers) cpHashSetEach(space.collisionHandlers, freeWrap, null);
            //cpHashSetFree(space.collisionHandlers);
        }
        /// Destroy and free a cpSpace.
        public void cpSpaceFree(cpSpace space)
        {
            if (space != null)
            {
                cpSpaceDestroy(space);
                // cpfree(space);
            }
        }

        //#define CP_DefineSpaceStructGetter(type, member, name) \
        //static  type cpSpaceGet##name(const cpSpace space){return space.member;}

        //#define CP_DefineSpaceStructSetter(type, member, name) \
        //static  void cpSpaceSet##name(cpSpace space, type value){space.member = value;}

        //#define CP_DefineSpaceStructProperty(type, member, name) \
        //CP_DefineSpaceStructGetter(type, member, name) \
        //CP_DefineSpaceStructSetter(type, member, name)

        //CP_DefineSpaceStructProperty(int, iterations, Iterations)
        //CP_DefineSpaceStructProperty(cpVect, gravity, Gravity)
        //CP_DefineSpaceStructProperty(float, damping, Damping)
        //CP_DefineSpaceStructProperty(float, idleSpeedThreshold, IdleSpeedThreshold)
        //CP_DefineSpaceStructProperty(float, sleepTimeThreshold, SleepTimeThreshold)
        //CP_DefineSpaceStructProperty(float, collisionSlop, CollisionSlop)
        //CP_DefineSpaceStructProperty(float, collisionBias, CollisionBias)
        //CP_DefineSpaceStructProperty(cpTimestamp, collisionPersistence, CollisionPersistence)
        //CP_DefineSpaceStructProperty(bool, enableContactGraph, EnableContactGraph)
        //CP_DefineSpaceStructProperty(cpDataPointer, data, UserData)
        //CP_DefineSpaceStructGetter(cpBody, staticBody, StaticBody)
        //CP_DefineSpaceStructGetter(float, CP_PRIVATE(curr_dt), CurrentTimeStep)

        public void cpAssertSpaceUnlocked(cpSpace space)
        {
            cpEnvironment.cpAssertHard(space.locked, "This operation cannot be done safely during a call to cpSpaceStep() or during a query. Put these calls into a post-step callback.");
        }

        /// returns true from inside a callback and objects cannot be added/removed.
        public bool IsLocked()
        {
            return locked;
        }

        /// Set a default collision handler for this space.
        /// The default collision handler is invoked for each colliding pair of shapes
        /// that isn't explicitly handled by a specific collision handler.
        /// You can pass null for any function you don't want to implement.
        public void cpSpaceSetDefaultCollisionHandler(
               cpSpace space,
               cpCollisionBeginFunc begin,
               cpCollisionPreSolveFunc preSolve,
               cpCollisionPostSolveFunc postSolve,
               cpCollisionSeparateFunc separate,
               object data
           )
        {
            cpAssertSpaceUnlocked(space);

            cpCollisionHandler handler = new cpCollisionHandler(
                0, 0,
                begin != null ? begin : alwaysCollide,
                preSolve != null ? preSolve : alwaysCollide,
                postSolve != null ? postSolve : nothing,
                separate != null ? separate : nothing,
                data
            );

            space.defaultHandler = handler;
            collisionHandlers.SetDefaultValue(space.defaultHandler);
            //cpHashSetSetDefaultValue(space.collisionHandlers,space.defaultHandler);
        }

        /// Set a collision handler to be used whenever the two shapes with the given collision types collide.
        /// You can pass null for any function you don't want to implement.
        public void cpSpaceAddCollisionHandler(
              cpSpace space,
              int a, int b,
              cpCollisionBeginFunc begin,
              cpCollisionPreSolveFunc preSolve,
              cpCollisionPostSolveFunc postSolve,
              cpCollisionSeparateFunc separate,
              object data
          )
        {
            cpAssertSpaceUnlocked(space);

            // Remove any old function so the new one will get added.
            RemoveCollisionHandler(a, b);

            cpCollisionHandler handler = new cpCollisionHandler(
        a, b,
        begin != null ? begin : alwaysCollide,
        preSolve != null ? preSolve : alwaysCollide,
        postSolve != null ? postSolve : nothing,
        separate != null ? separate : nothing,
        data
    );

            //  cpHashSetInsert(space.collisionHandlers, cpEnvironment.CP_HASH_PAIR(a, b), handler, null, (cpHashSetTransFunc)handlerSetTrans);

        }

        /// Unset a collision handler.
        public void RemoveCollisionHandler(int a, int b)
        {

            cpAssertSpaceUnlocked(this);

            // space.collisionHandlers.

            //struct { cpCollisionType a, b; } ids = {a, b};
            //cpCollisionHandler old_handler = new cpCollisionHandler();

            collisionHandlers.Remove(cpEnvironment.CP_HASH_PAIR(a, b));
            //cpHashSetRemove(space.collisionHandlers,ids);
            // space.collisionHandlers.Remove()

            //space.collisionHandlers.

            // cpfree(old_handler);
        }

        /// Add a collision shape to the simulation.
        /// If the shape is attached to a static body, it will be added as a static shape.
        public cpShape AddShape(cpShape shape)
        {

            cpBody body = shape.body;
            if (body.IsStatic()) return AddStaticShape(shape);

            cpEnvironment.cpAssertHard(shape.space != this, "You have already added this shape to this space. You must not add it a second time.");
            cpEnvironment.cpAssertHard(shape.space != null, "You have already added this shape to another space. You cannot add it to a second.");
            cpAssertSpaceUnlocked(this);


            body.Activate();
            //cpBodyActivate(body.Activate);
            //cpBodyAddShape(body, shape);

            body.AddShape(shape);

            shape.Update(body.Position, body.Rotation);

            this.activeShapes.Add(shape.hashid, shape);

            //cpSpatialIndexInsert(space.activeShapes, shape, shape.hashid);
            shape.space = this;

            return shape;

        }
        /// Explicity add a shape as a static shape to the simulation.
        public cpShape AddStaticShape(cpShape shape)
        {

            // cpEnvironment.cpAssertHard(shape.space != this, "You have already added this shape to this space. You must not add it a second time.");
            // cpEnvironment.cpAssertHard(shape.space != null, "You have already added this shape to another space. You cannot add it to a second.");
            // cpEnvironment.cpAssertHard(shape.body.IsRogue(), "You are adding a static shape to a dynamic body. Did you mean to attach it to a static or rogue body? See the documentation for more information.");
            cpAssertSpaceUnlocked(this);

            cpBody body = shape.body;
            body.AddShape(shape);
            //cpBodyAddShape(body, shape);
            shape.Update(body.Position, body.Rotation);
            //cpShapeUpdate(shape,);

            this.staticShapes.Add(shape.hashid, shape);
            //cpSpatialIndexInsert(space.staticShapes, );
            shape.space = this;

            return shape;

        }
        /// Add a rigid body to the simulation.
        public cpBody AddBody(cpBody body)
        {
            cpEnvironment.cpAssertHard(!body.IsStatic(), "Do not add static bodies to a space. Static bodies do not move and should not be simulated.");
            cpEnvironment.cpAssertHard(body.space != this, "You have already added this body to this space. You must not add it a second time.");
            cpEnvironment.cpAssertHard(body.space != null, "You have already added this body to another space. You cannot add it to a second.");
            cpAssertSpaceUnlocked(this);

            bodies.Add(body);
            //cpArrayPush(space.bodies, body);
            body.space = this;

            return body;

        }
        /// Add a constraint to the simulation.
        public cpConstraint AddConstraint(cpConstraint constraint)
        {

            cpEnvironment.cpAssertHard(constraint.space != this, "You have already added this constraint to this space. You must not add it a second time.");
            cpEnvironment.cpAssertHard(constraint.space != null, "You have already added this constraint to another space. You cannot add it to a second.");
            cpEnvironment.cpAssertHard(constraint.a != null && constraint.b != null, "Constraint is attached to a null body.");
            cpAssertSpaceUnlocked(this);

            constraint.a.Activate();
            constraint.b.Activate();

            constraints.Add(constraint);

            // Push onto the heads of the bodies' constraint lists
            cpBody a = constraint.a, b = constraint.b;
            constraint.next_a = a.constraintList; a.constraintList = constraint;
            constraint.next_b = b.constraintList; b.constraintList = constraint;
            constraint.space = this;

            return constraint;

        }

        /// Remove a collision shape from the simulation.
        public void RemoveShape(cpShape shape)
        {


            cpBody body = shape.body;
            if (body.IsStatic())
            {
                RemoveStaticShape(shape);
            }
            else
            {
                cpEnvironment.cpAssertHard(ContainsShape(shape), "Cannot remove a shape that was not added to the space. (Removed twice maybe?)");
                cpAssertSpaceUnlocked(this);

                //cpBodyActivate(body);body

                body.Activate();
                body.RemoveShape(shape);

                cpSpaceFilterArbiters(this, body, shape);

                this.activeShapes.Remove(shape.hashid);

                shape.space = null;
            }

        }
        /// Remove a collision shape added using cpSpaceAddStaticShape() from the simulation.
        public void RemoveStaticShape(cpShape shape)
        {

            cpEnvironment.cpAssertHard(ContainsShape(shape), "Cannot remove a static or sleeping shape that was not added to the space. (Removed twice maybe?)");
            cpAssertSpaceUnlocked(this);

            cpBody body = shape.body;
            if (body.IsStatic())
                body.ActivateStatic(shape);// cpBodyActivateStatic(body, shape);

            body.RemoveShape(shape);
            //cpBodyRemoveShape(body, shape);

            cpSpaceFilterArbiters(this, body, shape);

            //cpSpatialIndexRemove(space.staticShapes, shape, );

            staticShapes.Remove(shape.hashid);


            shape.space = null;

        }
        /// Remove a rigid body from the simulation.
        public void RemoveBody(cpBody body)
        {


            cpEnvironment.cpAssertHard(ContainsBody(body), "Cannot remove a body that was not added to the space. (Removed twice maybe?)");
            cpAssertSpaceUnlocked(this);

            body.Activate();
            //	cpSpaceFilterArbiters(space, body, null);
            this.bodies.Remove(body);

            body.space = null;

        }
        /// Remove a constraint from the simulation.
        public void RemoveConstraint(cpConstraint constraint)
        {

            cpEnvironment.cpAssertHard(ContainsConstraint(constraint), "Cannot remove a constraint that was not added to the space. (Removed twice maybe?)");
            cpAssertSpaceUnlocked(this);

            constraint.a.Activate();
            constraint.b.Activate();
            constraints.Remove(constraint);

            constraint.a.RemoveConstraint(constraint);
            constraint.b.RemoveConstraint(constraint);

            //cpBodyRemoveConstraint(constraint.a, constraint);
            //cpBodyRemoveConstraint(constraint.b, constraint);
            constraint.space = null;


        }

        /// Test if a collision shape has been added to the space.
        public bool ContainsShape(cpShape shape)
        {
            return (shape.space == this);
        }
        /// Test if a rigid body has been added to the space.
        public bool ContainsBody(cpBody body)
        {
            return (body.space == this);
        }
        /// Test if a constraint has been added to the space.
        public bool ContainsConstraint(cpConstraint constraint)
        {

            return (constraint.space == this);

        }

        /// Convert a dynamic rogue body to a static one.
        /// If the body is active, you must remove it from the space first.
        public void ConvertBodyToStatic(cpBody body)
        {

            cpEnvironment.cpAssertHard(!body.IsStatic(), "Body is already static.");
            cpEnvironment.cpAssertHard(body.IsRogue(), "Remove the body from the space before calling this function.");
            cpAssertSpaceUnlocked(this);

            body.SetMass(cpEnvironment.INFINITY_FLOAT); // cpBodySetMass(body, INFINITY);
            body.SetMoment(cpEnvironment.INFINITY_FLOAT);  //cpBodySetMoment(body, INFINITY);

            body.Vel = cpVect.ZERO;
            body.AngVel = 0.0f;


            body.node.idleTime = cpEnvironment.INFINITY_FLOAT;
            body.EachShape((b, s, d) =>
            {

                activeShapes.Remove(s.hashid);
                staticShapes.Add(s.hashid, s);
                //cpSpatialIndexRemove(, shape, shape.hashid);
                //cpSpatialIndexInsert(space.staticShapes, shape, shape.hashid);

            }, null);

            //CP_BODY_FOREACH_SHAPE(body, shape){

        }

        public static void PostStepDoNothing(cpSpace space, object obj, object data) { }

        /// Convert a body to a dynamic rogue body.
        /// If you want the body to be active after the transition, you must add it to the space also.
        public void ConvertBodyToDynamic(cpBody body, float mass, float moment)
        {

            cpEnvironment.cpAssertHard(body.IsStatic(), "Body is already dynamic.");
            cpAssertSpaceUnlocked(this);

            body.ActivateStatic(null);
            body.SetMass(mass);

            //cpBodySetMass(body, m);
            body.SetMoment(moment);

            body.node.idleTime = 0.0f;

            body.EachShape((cpBody b, cpShape s, object d) =>
            {
                this.staticShapes.Remove(s.hashid);
                //cpSpatialIndexRemove(space.staticShapes, shape, shape.hashid);
                //cpSpatialIndexInsert(space.activeShapes, shape, shape.hashid);
                this.activeShapes.Add(s.hashid, s);

            }, null);

            //CP_BODY_FOREACH_SHAPE(body, shape){

        }

        public cpPostStepCallback? GetPostStepCallback(int key)
        {
            foreach (var callback in this.postStepCallbacks)
                if (callback.key == key) return callback;

            return null;

        }

        //MARK: Spatial Index Management

        public static void updateBBCache(cpShape shape, object unused)
        {
            cpBody body = shape.body;
            shape.Update(body.Position, body.Rotation);
        }



        /// Update the collision detection info for the static shapes in the space.
        public void ReindexStatic()
        {
            cpEnvironment.cpAssertHard(!locked, "You cannot manually reindex objects while the space is locked. Wait until the current query or step is complete.");

            // cpSpatialIndexEach(space.staticShapes, updateBBCache, null);

            cpShape shp;
            foreach (var item in staticShapes)
            {
                shp = (cpShape)item.Value;
                updateBBCache(shp, null);
            }
            //space.staticShapes.IndexEach(updateBBCache,null);

            staticShapes.Reindex();

        }
        /// Update the collision detection data for a specific shape in the space.
        public void cpSpaceReindexShape(cpSpace space, cpShape shape)
        {
            cpEnvironment.cpAssertHard(!space.locked, "You cannot manually reindex objects while the space is locked. Wait until the current query or step is complete.");

            cpBody body = shape.body;
            shape.Update(body.Position, body.Rotation);
            //cpShapeUpdate(shape, );

            space.activeShapes.ReindexObject(shape.hashid, shape);
            space.staticShapes.ReindexObject(shape.hashid, shape);
            // attempt to rehash the shape in both hashes
            //cpSpatialIndexReindexObject(space.activeShapes, shape, shape.hashid);
            //cpSpatialIndexReindexObject(space.staticShapes, shape, shape.hashid);
        }
        /// Update the collision detection data for all shapes attached to a body.
        public void cpSpaceReindexShapesForBody(cpSpace space, cpBody body)
        {
            //TODO:REINDEX
            //CP_BODY_FOREACH_SHAPE(body, shape) cpSpaceReindexShape(space, shape);
        }


        public static void copyShapes(cpShape shape, cpBBTree index)
        {
            index.Add(shape.hashid, shape);
            //cpSpatialIndexInsert(index, shape, shape.hashid);
        }

        /// Switch the space to use a spatial has as it's spatial index.
        //public void cpSpaceUseSpatialHash(cpSpace space, float dim, int count)
        //{
        //    cpSpatialIndex staticShapes = new cpSpatialIndex(); // cpSpaceHashNew(dim, count, (cpSpatialIndexBBFunc)cpShapeGetBB, null);
        //    cpSpatialIndex activeShapes = new cpSpatialIndex(); // cpSpaceHashNew(dim, count, (cpSpatialIndexBBFunc)cpShapeGetBB, staticShapes);

        //    cpSpatialIndexEach(space.staticShapes, (cpSpatialIndexIteratorFunc)copyShapes, staticShapes);
        //    cpSpatialIndexEach(space.activeShapes, (cpSpatialIndexIteratorFunc)copyShapes, activeShapes);

        //    cpSpatialIndexFree(space.staticShapes);
        //    cpSpatialIndexFree(space.activeShapes);

        //    space.staticShapes = staticShapes;
        //    space.activeShapes = activeShapes;
        //}

        public void UncacheArbiter(cpArbiter arb)
        {
            cpShape a = arb.a, b = arb.b;
            List<cpShape> shape_pair = new List<cpShape>() { a, b };

            int arbHashID = cpEnvironment.CP_HASH_PAIR(a.hashid, b.hashid);
            cachedArbiters.Remove(arbHashID);

            arbiters.Remove(arb);

            //cpHashSetRemove(, arbHashID, shape_pair);
            //cpArrayDeleteObj(space->arbiters, arb);
        }


        public cpCollisionHandler LookupHandler(int a, int b)
        {
            object test;
            if (collisionHandlers.TryGetValue(cpEnvironment.CP_HASH_PAIR(a, b), out test))
                return (cpCollisionHandler)test;
            else
                return new cpCollisionHandler();

            //TODO: NOT SURE
            //        cpCollisionType types[] = {a, b};
            //return (cpCollisionHandler *)cpHashSetFind(space->collisionHandlers,, types);
        }

        public void SetGravity(cpVect gravity)
        {
            this.gravity = gravity;
        }


        #region DEBUG DRAW

        public void SetDebugDraw(cpDraw debug)
        {
            m_debugDraw = debug;
        }

        public void DrawDebugData()
        {
            if (m_debugDraw == null)
            {
                return;
            }
            m_debugDraw.DrawString(0, 15, string.Format("Step: {0}", stamp));
            m_debugDraw.DrawString(0, 50, string.Format("Bodies : {0}/{1}", bodies.Count, bodies.Capacity));
            m_debugDraw.DrawString(0, 80, string.Format("Arbiters: {0}/{1}", arbiters.Count, arbiters.Capacity));

            var contacts = 0;
            for (var i = 0; i < arbiters.Count; i++)
            {
                contacts += arbiters[i].contacts.Count;
            }

            m_debugDraw.DrawString(0, 110, "Contact points: " + contacts);
            //this.ctx.fillText("Contact points: " + contacts + " (Max: " + this.maxContacts + ")", 10, 140, maxWidth);


            ChipmunkDrawFlags flags = m_debugDraw.Flags;

            if ((flags & ChipmunkDrawFlags.e_shapeBit) != 0)
            {
                Type actualType;
                cpShape shape;
                foreach (var item in activeShapes)
                {
                    shape = (cpShape)item.Value;
                    shape.Draw(m_debugDraw);
                    Console.WriteLine("dsadasdsa");
                }

                //for (b2Body b = m_bodyList; b != null; b = b.Next)
                //{
                //    for (b2Fixture f = b.FixtureList; f != null; f = f.Next)
                //    {
                //        if (b.IsActive() == false)
                //        {
                //            DrawShape(f, ref b.Transform, new b2Color(0.5f, 0.5f, 0.3f));
                //        }
                //        else if (b.BodyType == b2BodyType.b2_staticBody)
                //        {
                //            DrawShape(f, ref b.Transform, new b2Color(0.5f, 0.9f, 0.5f));
                //        }
                //        else if (b.BodyType == b2BodyType.b2_kinematicBody)
                //        {
                //            DrawShape(f, ref b.Transform, new b2Color(0.5f, 0.5f, 0.9f));
                //        }
                //        else if (b.IsAwake() == false)
                //        {
                //            DrawShape(f, ref b.Transform, new b2Color(0.6f, 0.6f, 0.6f));
                //        }
                //        else
                //        {
                //            DrawShape(f, ref b.Transform, new b2Color(0.9f, 0.7f, 0.7f));
                //        }
                //    }
                //}
            }

            if ((flags & ChipmunkDrawFlags.e_jointBit) != 0)
            {
                //for (b2Joint j = m_jointList; j != null; j = j.GetNext())
                //{
                //    DrawJoint(j);
                //}
            }

            if ((flags & ChipmunkDrawFlags.e_pairBit) != 0)
            {
                //b2Color color = new b2Color(0.3f, 0.9f, 0.9f);
                //for (b2Contact c = m_contactManager.ContactList; c != null; c = c.Next)
                //{
                //b2Fixture fixtureA = c.FixtureA;
                //b2Fixture fixtureB = c.FixtureB;

                //b2Vec2 cA = fixtureA.GetAABB().Center;
                //b2Vec2 cB = fixtureB.GetAABB().Center;

                //m_debugDraw.DrawSegment(cA, cB, color);
                //}
            }

            if ((flags & ChipmunkDrawFlags.e_aabbBit) != 0)
            {
                //b2Color color = new b2Color(0.9f, 0.3f, 0.9f);
                //b2BroadPhase bp = m_contactManager.BroadPhase;

                //for (b2Body b = m_bodyList; b != null; b = b.Next)
                //{
                //    if (b.IsActive() == false)
                //    {
                //        continue;
                //    }

                //    for (b2Fixture f = b.FixtureList; f != null; f = f.Next)
                //    {
                //        for (int i = 0; i < f.ProxyCount; ++i)
                //        {
                //            b2FixtureProxy proxy = f.Proxies[i];
                //            b2AABB aabb;
                //            bp.GetFatAABB(proxy.proxyId, out aabb);
                //            b2Vec2[] vs = new b2Vec2[4];
                //            vs[0].Set(aabb.LowerBound.x, aabb.LowerBound.y);
                //            vs[1].Set(aabb.UpperBound.x, aabb.LowerBound.y);
                //            vs[2].Set(aabb.UpperBound.x, aabb.UpperBound.y);
                //            vs[3].Set(aabb.LowerBound.x, aabb.UpperBound.y);

                //            m_debugDraw.DrawPolygon(vs, 4, color);
                //        }
                //    }
                //}
            }

            if ((flags & ChipmunkDrawFlags.e_centerOfMassBit) != 0)
            {
                //for (b2Body b = m_bodyList; b != null; b = b.Next)
                //{
                //    b2Transform xf = b.Transform;
                //    xf.p = b.WorldCenter;
                //    m_debugDraw.DrawTransform(xf);
                //}
            }

        }

        //public void DrawShape(cpShape shape, CCColor4B color)
        //{

        //    // if (fixture.type == cpShapeType.CP_CIRCLE_SHAPE)

        //    switch (shape.klass.type)
        //    {
        //        case cpShapeType.CP_CIRCLE_SHAPE:
        //            {

        //                cpCircleShape circle = (cpCircleShape)shape;

        //                //circle.Draw(_)

        //                //b2CircleShape circle = (b2CircleShape)fixture.Shape;

        //                //b2Vec2 center = b2Math.b2Mul(ref xf, ref circle.Position);
        //                //float radius = circle.Radius;
        //                //cpVect v = new cpVect(1.0f, 0.0f);
        //                //cpVect axis = cpVect b2Math.b2Mul(ref xf.q, ref v);

        //                //m_debugDraw.DrawSolidCircle(center, circle.r, axis, color);
        //            }
        //            break;

        //        //case cpShapeType.e_edge:
        //        //    {
        //        //        b2EdgeShape edge = (b2EdgeShape)fixture.Shape;
        //        //        b2Vec2 v1 = b2Math.b2Mul(ref xf, ref edge.Vertex1);
        //        //        b2Vec2 v2 = b2Math.b2Mul(ref xf, ref edge.Vertex2);
        //        //        m_debugDraw.DrawSegment(v1, v2, color);
        //        //    }
        //        //    break;

        //        //case b2ShapeType.e_chain:
        //        //    {
        //        //        b2ChainShape chain = (b2ChainShape)fixture.Shape;
        //        //        int count = chain.Count;
        //        //        b2Vec2[] vertices = chain.Vertices;

        //        //        b2Vec2 v1 = b2Math.b2Mul(ref xf, ref vertices[0]);
        //        //        for (int i = 1; i < count; ++i)
        //        //        {
        //        //            b2Vec2 v2 = b2Math.b2Mul(ref xf, ref vertices[i]);
        //        //            m_debugDraw.DrawSegment(v1, v2, color);
        //        //            m_debugDraw.DrawCircle(v1, 0.05f, color);
        //        //            v1 = v2;
        //        //        }
        //        //    }
        //        //    break;

        //        case cpShapeType.CP_POLY_SHAPE:
        //            {
        //                //b2PolygonShape poly = (b2PolygonShape)fixture.Shape;
        //                //int vertexCount = poly.VertexCount;
        //                //var vertices = b2ArrayPool<b2Vec2>.Create(b2Settings.b2_maxPolygonVertices, true);

        //                //for (int i = 0; i < vertexCount; ++i)
        //                //{
        //                //    vertices[i] = b2Math.b2Mul(ref xf, ref poly.Vertices[i]);
        //                //}

        //                //m_debugDraw.DrawSolidPolygon(vertices, vertexCount, color);

        //                //b2ArrayPool<b2Vec2>.Free(vertices);
        //            }
        //            break;

        //        default:
        //            break;
        //    }
        //}

        #endregion




    }
}

/// Call @c func for each body in the space.
//        void cpSpaceEachBody(cpSpace space, cpSpaceBodyIteratorFunc func, object data) {

//cpSpaceLock(space); {
//        cpArray *bodies = space.bodies;

//        for(int i=0; i<bodies.num; i++){
//            func((cpBody )bodies.arr[i], data);
//        }

//        cpArray *components = space.sleepingComponents;
//        for(int i=0; i<components.num; i++){
//            cpBody root = (cpBody )components.arr[i];

//            cpBody body = root;
//            while(body){
//                cpBody next = body.node.next;
//                func(body, data);
//                body = next;
//            }
//        }
//    } cpSpaceUnlock(space, true);

//}

/// Space/body iterator callback function type.
//        delegate void cpSpaceShapeIteratorFunc(cpShape shape, object data) {

//}
/// Call @c func for each shape in the space.
//        void cpSpaceEachShape(cpSpace space, cpSpaceShapeIteratorFunc func, object data) {

//    cpSpaceLock(space); {
//        spaceShapeContext context = {func, data};
//        cpSpatialIndexEach(space.activeShapes, (cpSpatialIndexIteratorFunc)spaceEachShapeIterator, context);
//        cpSpatialIndexEach(space.staticShapes, (cpSpatialIndexIteratorFunc)spaceEachShapeIterator, context);
//    } cpSpaceUnlock(space, true);

//}

/// Call @c func for each shape in the space.
//    void cpSpaceEachConstraint(cpSpace space, cpSpaceConstraintIteratorFunc func, object data) {

//cpSpaceLock(space); {
//    cpArray *constraints = space.constraints;

//    for(int i=0; i<constraints.num; i++){
//        func((cpConstraint *)constraints.arr[i], data);
//    }
//} cpSpaceUnlock(space, true);
