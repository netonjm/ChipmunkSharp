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
using System.Diagnostics;
using System.Linq;

namespace ChipmunkSharp
{



	/// Basic Unit of Simulation in Chipmunk
	/// 
	public partial class cpSpace
	{

		private cpDebugDraw m_debugDraw;

		#region PARAMS
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


		/// User definable data pointer.
		/// Generally this points to your game's controller or game state
		/// class so you can access it when given a cpSpace reference in a callback.
		public object userData;

		public int stamp;
		public float curr_dt;

		public List<cpBody> dynamicBodies;
		public List<cpBody> staticBodies;

		public List<cpBody> rousedBodies;
		public List<cpBody> sleepingComponents;


		public cpBBTree staticShapes;
		public cpBBTree dynamicShapes;

		// public List<cpArbiter> pooledArbiters;
		public List<cpConstraint> constraints;

		public List<cpArbiter> arbiters { get; set; }
		public Dictionary<string, cpArbiter> cachedArbiters;

		// public cpArray allocatedBuffers;
		public int locked;

		public bool usesWildcards;
		public Dictionary<string, cpCollisionHandler> collisionHandlers;
		public cpCollisionHandler defaultHandler;

		public bool skipPostStep;
		public List<cpPostStepCallback> postStepCallbacks;


		List<cpContact> contactsBuffer = new List<cpContact>(); //Not used

		/// The designated static body for this space.
		/// You can modify this body, or replace it with your own static body.
		/// By default it points to a statically allocated cpBody in the cpSpace struct.
		public cpBody staticBody;
		// public cpBody _staticBody;

		/// returns true from inside a callback and objects cannot be added/removed.
		public bool IsLocked
		{
			get { return (this.locked > 0); }
		}

		//public CollisionHandler DefaultHandler { get; set; }
		#endregion

		//MARK: Misc Helper Funcs
		// Default collision functions.
		static bool DefaultBegin(cpArbiter arb, cpSpace space, object data)
		{
			bool retA = arb.CallWildcardBeginA(space);// cpArbiterCallWildcardBeginA(arb, space);
			bool retB = arb.CallWildcardBeginB(space);// cpArbiterCallWildcardBeginB(arb, space);
			return retA && retB;
		}

		static bool DefaultPreSolve(cpArbiter arb, cpSpace space, object data)
		{
			bool retA = arb.CallWildcardPreSolveA(space);
			bool retB = arb.CallWildcardPreSolveB(space);
			return retA && retB;
		}

		static void DefaultPostSolve(cpArbiter arb, cpSpace space, object data)
		{
			arb.CallWildcardPostSolveA(space);
			arb.CallWildcardPostSolveB(space);

		}

		static void DefaultSeparate(cpArbiter arb, cpSpace space, object data)
		{
			arb.CallWildcardSeparateA(space);
			arb.CallWildcardSeparateB(space);
		}

		// Use the wildcard identifier since  the default handler should never match any type pair.
		static cpCollisionHandler cpCollisionHandlerDefault = new cpCollisionHandler(
	cp.WILDCARD_COLLISION_TYPE, cp.WILDCARD_COLLISION_TYPE,
	DefaultBegin, DefaultPreSolve, DefaultPostSolve, DefaultSeparate, null
);

		static bool AlwaysCollide(cpArbiter arb, cpSpace space, object data) { return true; }
		static void DoNothing(cpArbiter arb, cpSpace space, object data) { }

		cpCollisionHandler cpCollisionHandlerDoNothing = new cpCollisionHandler(
	cp.WILDCARD_COLLISION_TYPE, cp.WILDCARD_COLLISION_TYPE,
	AlwaysCollide, AlwaysCollide, DoNothing, DoNothing, null);


		// function to get the estimated velocity of a shape for the cpBBTree.
		static cpVect ShapeVelocityFunc(cpShape shape) { return shape.body.v; }

		// Used for disposing of collision handlers.
		//static void FreeWrap(void* ptr, void* unused) { cpfree(ptr); }

		//MARK: Memory Management Functions


		public cpSpace()
		{

#if DEBUG
			Debug.WriteLine("Initializing cpSpace - Chipmunk v{0} (Debug Enabled)\n", cp.cpVersionString);
			Debug.WriteLine("Compile with -DNDEBUG defined to disable debug mode and runtime assertion checks\n");
#endif


			/// Number of iterations to use in the impulse solver to solve contacts.
			this.iterations = 10;

			/// Gravity to pass to rigid bodies when integrating velocity.
			this.gravity = cpVect.Zero;

			/// Damping rate expressed as the fraction of velocity bodies retain each second.
			/// A value of 0.9 would mean that each body's velocity will drop 10% per second.
			/// The default value is 1.0, meaning no damping is applied.
			/// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
			this.damping = 1;

			/// Amount of encouraged penetration between colliding shapes..
			/// Used to reduce oscillating contacts and keep the collision cache warm.
			/// Defaults to 0.1. If you have poor simulation quality,
			/// increase this number as much as possible without allowing visible amounts of overlap.
			this.collisionSlop = 0.1f;

			/// Determines how fast overlapping shapes are pushed apart.
			/// Expressed as a fraction of the error remaining after each second.
			/// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
			this.collisionBias = cp.cpfpow(1f - 0.1f, 60f);

			/// Number of frames that contact information should persist.
			/// Defaults to 3. There is probably never a reason to change this value.
			this.collisionPersistence = 3;

			this.locked = 0;
			this.stamp = 0;

			this.staticShapes = new cpBBTree(null);
			this.dynamicShapes = new cpBBTree(this.staticShapes);

			this.dynamicShapes.SetVelocityFunc(o => ShapeVelocityFunc(o as cpShape));

			this.dynamicBodies = new List<cpBody>();
			this.staticBodies = new List<cpBody>();
			this.rousedBodies = new List<cpBody>();
			this.sleepingComponents = new List<cpBody>();

			/// Time a group of bodies must remain idle in order to fall asleep.
			/// Enabling sleeping also implicitly enables the the contact graph.
			/// The default value of Infinity disables the sleeping algorithm.
			this.sleepTimeThreshold = cp.Infinity;
			/// Speed threshold for a body to be considered idle.
			/// The default value of 0 means to let the space guess a good threshold based on gravity.
			this.idleSpeedThreshold = 0;

			this.arbiters = new List<cpArbiter>();
			this.cachedArbiters = new Dictionary<string, cpArbiter>();

			this.constraints = new List<cpConstraint>();

			this.usesWildcards = false;
			this.defaultHandler = cpCollisionHandlerDoNothing;

			this.collisionHandlers = new Dictionary<string, cpCollisionHandler>();

			this.postStepCallbacks = new List<cpPostStepCallback>();

			this.skipPostStep = false;

			/// The designated static body for this space.
			/// You can modify this body, or replace it with your own static body.
			/// By default it points to a statically allocated cpBody in the cpSpace struct.
			this.staticBody = cpBody.NewStatic();

		}

		public void Clear()
		{
			//TODO: Add static/dinamic bodies
			List<object> safeList = new List<object>();
			EachBody((b) => safeList.Add(b), null);
			foreach (var item in safeList)
				RemoveBody(item as cpBody);

			safeList.Clear();
			EachShape((b) => safeList.Add(b));
			foreach (var item in safeList)
				RemoveShape(item as cpShape);

			safeList.Clear();

			EachConstraint((b) => safeList.Add(b));
			foreach (var item in safeList)
				RemoveConstraint(item as cpConstraint);

			dynamicShapes.leaves.Clear();
			staticShapes.leaves.Clear();

			constraints.Clear();
			collisionHandlers.Clear();
			dynamicBodies.Clear();
			cachedArbiters.Clear();
			arbiters.Clear();

			stamp = 0;
			cp.numLeaves = 0; cp.numNodes = 0; cp.numPairs = 0;

			//foreach (var item in space.collisionHandlers)
			//	space.removeCollisionHandler(item.Value.a, item.Value.b);
		}

		//MARK: Basic properties:

		public int GetIterations()
		{
			return this.iterations;
		}

		public void SetIterations(int iterations)
		{
			cp.assertHard(iterations > 0, "Iterations must be positive and non-zero.");
			this.iterations = iterations;
		}

		public cpVect GetGravity()
		{
			return this.gravity;
		}

		public void SetGravity(cpVect gravity)
		{
			this.gravity = gravity;
		}

		public float GetDamping()
		{
			return this.damping;
		}

		public void SetDamping(float damping)
		{
			cp.assertHard(damping >= 0.0, "Damping must be positive.");
			this.damping = damping;
		}

		public float GetIdleSpeedThreshold()
		{
			return this.idleSpeedThreshold;
		}

		public void SetIdleSpeedThreshold(float idleSpeedThreshold)
		{
			this.idleSpeedThreshold = idleSpeedThreshold;
		}

		public float GetSleepTimeThreshold()
		{
			return this.sleepTimeThreshold;
		}

		public void SetSleepTimeThreshold(float sleepTimeThreshold)
		{
			this.sleepTimeThreshold = sleepTimeThreshold;
		}

		public float GetCollisionSlop()
		{
			return this.collisionSlop;
		}

		public void SetCollisionSlop(float collisionSlop)
		{
			this.collisionSlop = collisionSlop;
		}

		public float GetCollisionBias()
		{
			return this.collisionBias;
		}

		public void SetCollisionBias(float collisionBias)
		{
			this.collisionBias = collisionBias;
		}

		public float GetCollisionPersistence()
		{
			return this.collisionPersistence;
		}

		public void SetCollisionPersistence(int collisionPersistence)
		{
			this.collisionPersistence = collisionPersistence;
		}

		public object GetUserData()
		{
			return this.userData;
		}

		public void SetUserData(object userData)
		{
			this.userData = userData;
		}

		public cpBody GetStaticBody()
		{
			return this.staticBody;
		}

		public float GetCurrentTimeStep()
		{
			return this.curr_dt;
		}

		public void SetStaticBody(cpBody body)
		{
			if (this.staticBody != null)
			{
				cp.assertHard(this.staticBody.shapeList == null, "Internal Error: Changing the designated static body while the old one still had shapes attached.");
				this.staticBody.space = null;
			}

			this.staticBody = body;
			body.space = this;
		}

		public void UseWildcardDefaultHandler()
		{
			// Spaces default to using the slightly faster "do nothing" default handler until wildcards are potentially needed.
			if (!this.usesWildcards)
			{
				this.usesWildcards = true;
				this.defaultHandler = cpCollisionHandlerDefault;
			}
		}

		public cpCollisionHandler AddDefaultCollisionHandler()
		{
			UseWildcardDefaultHandler();
			return this.defaultHandler;
		}

		public cpCollisionHandler AddCollisionHandler(string a, string b)
		{

			string hash = cp.hashPair(a, b);

			var handlers = this.collisionHandlers;

			cpCollisionHandler handler;
			if (!handlers.TryGetValue(hash, out handler))
			{
				handler = new cpCollisionHandler(a, b, DefaultBegin, DefaultPreSolve, DefaultPostSolve, DefaultSeparate, null);
				handlers.Add(hash, handler);
			}
			return handler;
		}

		public cpCollisionHandler AddWildcardHandler(string type)
		{
			UseWildcardDefaultHandler();

			string hash = cp.hashPair(type, cp.WILDCARD_COLLISION_TYPE);

			var handlers = this.collisionHandlers;

			cpCollisionHandler handler;
			if (!handlers.TryGetValue(hash, out handler))
			{
				handler = new cpCollisionHandler(type, cp.WILDCARD_COLLISION_TYPE, cpCollisionHandler.AlwaysCollide, cpCollisionHandler.AlwaysCollide, cpCollisionHandler.DoNothing, cpCollisionHandler.DoNothing, null);
				handlers.Add(hash, handler);
			}
			return handler;
		}

		/// Add a collision shape to the simulation.
		/// If the shape is attached to a static body, it will be added as a static shape.
		public cpShape AddShape(cpShape shape)
		{

			var body = shape.body;

			cp.assertHard(shape.space != this, "You have already added this shape to this space. You must not add it a second time.");
			cp.assertHard(shape.space == null, "You have already added this shape to another space. You cannot add it to a second.");
			cp.assertSpaceUnlocked(this);

			bool isStatic = body.bodyType == cpBodyType.STATIC;
			if (!isStatic)
				body.Activate();

			body.AddShape(shape);

			shape.hashid = (cp.shapeIDCounter++).ToString();

			shape.Update(body.transform);

			(isStatic ? this.staticShapes : this.dynamicShapes).Insert(shape.hashid, shape);
			shape.space = this;

			return shape;

		}

		/// Add a rigid body to the simulation.
		public cpBody AddBody(cpBody body)
		{
			cp.assertHard(body.space != this, "You have already added this body to this space. You must not add it a second time.");
			cp.assertHard(body.space == null, "You have already added this body to another space. You cannot add it to a second.");
			cp.assertSpaceUnlocked(this);

			ArrayForBodyType(body.bodyType).Add(body);

			body.space = this;
			return body;

		}

		/// Add a constraint to the simulation.
		public cpConstraint AddConstraint(cpConstraint constraint)
		{

			cp.assertHard(constraint.space != this, "You have already added this constraint to this space. You must not add it a second time.");
			cp.assertHard(constraint.space == null, "You have already added this constraint to another space. You cannot add it to a second.");

			cp.assertSpaceUnlocked(this);

			cpBody a = constraint.a, b = constraint.b;

			cp.assertHard(a != null && b != null, "Constraint is attached to a NULL body.");

			a.Activate();
			b.Activate();

			this.constraints.Add(constraint);

			// Push onto the heads of the bodies' constraint lists
			constraint.next_a = a.constraintList; a.constraintList = constraint;
			constraint.next_b = b.constraintList; b.constraintList = constraint;
			constraint.space = this;

			return constraint;

		}

		public void filterArbiters(cpBody body, cpShape filter)
		{

			List<string> safeDelete = new List<string>();

			foreach (var hash in this.cachedArbiters)
			{
				cpArbiter arb = hash.Value;

				// Match on the filter shape, or if it's null the filter body
				if (
					(body == arb.body_a && (filter == arb.a || filter == null)) ||
					(body == arb.body_b && (filter == arb.b || filter == null))
				)
				{
					// Call separate when removing shapes.
					if (filter != null && arb.state != cpArbiterState.Cached)
					{
						arb.state = cpArbiterState.Invalidated;
						cpCollisionHandler handler = arb.handler;
						handler.separateFunc(arb, this, handler.userData);
					}

					arb.Unthread();
					this.arbiters.Remove(arb);
					safeDelete.Add(hash.Key);
				}

			}

			foreach (var item in safeDelete)
			{
				cachedArbiters.Remove(item);
			}

		}

		/// Remove a collision shape from the simulation.
		public void RemoveShape(cpShape shape)
		{
			var body = shape.body;

			cp.assertHard(ContainsShape(shape), "Cannot remove a shape that was not added to the space. (Removed twice maybe?)");
			cp.assertSpaceUnlocked(this);

			bool isStatic = body.bodyType == cpBodyType.STATIC;

			if (isStatic)
			{
				body.ActivateStatic(shape);
			}
			else
			{
				body.Activate();
			}

			body.RemoveShape(shape);
			this.filterArbiters(body, shape);

			(isStatic ? this.staticShapes : this.dynamicShapes).Remove(shape.hashid);

			shape.space = null;
			shape.hashid = "0";
		}

		/// Remove a rigid body from the simulation.
		public void RemoveBody(cpBody body)
		{
			cp.assertHard(body != GetStaticBody(), "Cannot remove the designated static body for the space.");
			cp.assertHard(ContainsBody(body), "Cannot remove a body that was not added to the space. (Removed twice maybe?)");
			//	cpAssertHard(body->shapeList == NULL, "Cannot remove a body from the space before removing the bodies attached to it.");
			//	cpAssertHard(body->constraintList == NULL, "Cannot remove a body from the space before removing the constraints attached to it.");
			cp.assertSpaceUnlocked(this);

			body.Activate();

			ArrayForBodyType(body.bodyType).Remove(body);

			body.space = null;
		}


		/// Remove a constraint from the simulation.
		public void RemoveConstraint(cpConstraint constraint)
		{

			cp.assertSoft(this.ContainsConstraint(constraint),
		   "Cannot remove a constraint that was not added to the space. (Removed twice maybe?)");
			cp.assertSpaceUnlocked(this);

			constraint.a.Activate();
			constraint.b.Activate();

			this.constraints.Remove(constraint);

			constraint.a.RemoveConstraint(constraint);
			constraint.b.RemoveConstraint(constraint);
			constraint.space = null;
		}

		/// Test if a constraint has been added to the space.
		public bool ContainsConstraint(cpConstraint constraint)
		{
			return (constraint.space == this);
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

		//MARK: Iteration

		public List<cpBody> ArrayForBodyType(cpBodyType type)
		{
			return type == cpBodyType.STATIC ?
				staticBodies : dynamicBodies;
		}

		/// Call @c func for each shape in the space.
		// **** Iteration
		public void EachBody(Action<cpBody> func, object data)
		{
			Lock();
			{
				List<cpBody> bodies = this.dynamicBodies;
				for (var i = 0; i < bodies.Count; i++)
				{
					func(bodies[i]);
				}

				bodies = this.staticBodies;
				for (var i = 0; i < bodies.Count; i++)
				{
					func(bodies[i]);
				}

				var components = this.sleepingComponents;
				for (var i = 0; i < components.Count; i++)
				{
					var root = components[i];

					var body = root;
					while (body != null)
					{
						var next = body.nodeNext;
						func(body);
						body = next;
					}
				}
			} this.Unlock(true);
		}

		public void EachShape(Action<cpShape> func)
		{
			this.Lock();
			{
				this.dynamicShapes.Each(s => func(s as cpShape));
				this.staticShapes.Each(s => func(s as cpShape));
			} this.Unlock(true);
		}

		public void EachConstraint(Action<cpConstraint> func)
		{
			Lock();
			{
				var constraints = this.constraints;
				for (var i = 0; i < constraints.Count; i++)
					func(constraints[i]);
			} this.Unlock(true);
		}

		public void UncacheArbiter(cpArbiter arb)
		{
			cachedArbiters.Remove(arb.Key);
			arbiters.Remove(arb);
		}

		/// Update the collision detection info for the static shapes in the space.
		public void ReindexStatic()
		{
			cp.assertSoft(!this.IsLocked, "You cannot manually reindex objects while the space is locked. Wait until the current query or step is complete.");

			this.staticShapes.Each(s => cpShape.UpdateFunc((s as cpShape), null));
			this.staticShapes.Reindex();

		}



		/// Update the collision detection data for a specific shape in the space.
		public void ReindexShape(cpShape shape)
		{
			cp.assertHard(!IsLocked, "You cannot manually reindex objects while the space is locked. Wait until the current query or step is complete.");

			//var body = shape.body;
			shape.CacheBB();
			//shape.Update(body.GetPosition(), body.GetRotation());

			// attempt to rehash the shape in both hashes
			this.dynamicShapes.ReindexObject(shape, shape.hashid);
			this.staticShapes.ReindexObject(shape, shape.hashid);
		}


		/// Update the collision detection data for all shapes attached to a body.
		public void ReindexShapesForBody(cpBody body)
		{

			foreach (var shape in body.shapeList)
			{
				this.ReindexShape(shape);
			}

			//for (var shape = body.shapeList; shape != null; shape = shape.next)
			//    this.reindexShape(shape);
		}


		/// Remove a collision shape added using cpSpaceAddStaticShape() from the simulation.
		//public void removeStaticShape(cpShape shape)
		//{

		//	cp.assertSoft(this.ContainsShape(shape),
		//   "Cannot remove a static or sleeping shape that was not added to the space. (Removed twice maybe?)");
		//	cp.assertSpaceUnlocked(this);

		//	var body = shape.body;
		//	if (body.bodyType == cpBodyType.STATIC) body.ActivateStatic(shape);
		//	body.RemoveShape(shape);
		//	this.filterArbiters(body, shape);
		//	this.staticShapes.Remove(shape.hashid);
		//	shape.space = null;

		//}


		/// /////////////////////////////////////////////////////////

		//#region OBSOLETE

		///// Set a collision handler to be used whenever the two shapes with the given collision types collide.
		///// You can pass null for any function you don't want to implement.
		//[Obsolete("OBSOLETE: JS")]
		//public void AddCollisionHandler(string a, string b,
		//	Func<cpArbiter, cpSpace, object, bool> begin,
		//	Func<cpArbiter, cpSpace, object, bool> preSolve,
		//	Action<cpArbiter, cpSpace, object> postSolve,
		//	Action<cpArbiter, cpSpace, object> separate)
		//{

		//	cp.assertSpaceUnlocked(this);

		//	// Remove any old function so the new one will get added.
		//	this.removeCollisionHandler(a, b);

		//	var handler = new cpCollisionHandler();
		//	handler.typeA = a;
		//	handler.typeB = b;
		//	if (begin != null)
		//		handler.beginFunc = begin;
		//	if (preSolve != null)
		//		handler.preSolveFunc = preSolve;
		//	if (postSolve != null)
		//		handler.postSolveFunc = postSolve;
		//	if (separate != null)
		//		handler.separateFunc = separate;

		//	collisionHandlers.Add(cp.hashPair(a, b), handler);
		//}

		//#endregion






		/// Explicity add a shape as a static shape to the simulation.
		//public cpShape addStaticShape(cpShape shape)
		//{

		//	// cpEnvironment.cpAssertHard(shape.space != this, "You have already added this shape to this space. You must not add it a second time.");
		//	// cpEnvironment.cpAssertHard(shape.space != null, "You have already added this shape to another space. You cannot add it to a second.");
		//	// cpEnvironment.cpAssertHard(shape.body.IsRogue(), "You are adding a static shape to a dynamic body. Did you mean to attach it to a static or rogue body? See the documentation for more information.");
		//	cp.assertSpaceUnlocked(this);


		//	var body = shape.body;
		//	body.AddShape(shape);

		//	shape.Update(body.GetPosition(), body.GetRotation());
		//	this.staticShapes.Insert(shape.hashid, shape);
		//	shape.space = this;

		//	return shape;

		//}

		public cpCollisionHandler LookupHandler(string a, string b, cpCollisionHandler defaultValue)
		{
			cpCollisionHandler test;
			if (collisionHandlers.TryGetValue(cp.hashPair(a, b), out test))
				return test;
			else
				return defaultValue;
		}

		/// Unset a collision handler.
		//public void removeCollisionHandler(string a, string b)
		//{
		//	cp.assertSpaceUnlocked(this);
		//	collisionHandlers.Remove(cp.hashPair(a, b));

		//}

		#region DEBUG DRAW

		public void SetDebugDraw(cpDebugDraw debug)
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
			m_debugDraw.DrawString(0, 50, string.Format("Bodies : {0}/{1}", dynamicBodies.Count + staticBodies.Count, dynamicBodies.Capacity));
			m_debugDraw.DrawString(0, 80, string.Format("Arbiters: {0}/{1}", arbiters.Count, arbiters.Capacity));

			if (m_debugDraw.Flags == cpDrawFlags.All || m_debugDraw.Flags == cpDrawFlags.Shapes)
			{
				EachShape(s => s.Draw(m_debugDraw));
			}

			if (m_debugDraw.Flags == cpDrawFlags.All || m_debugDraw.Flags == cpDrawFlags.Joints)
			{
				EachConstraint(c => c.Draw(m_debugDraw));
			}

			var contacts = 0;
			if (m_debugDraw.Flags == cpDrawFlags.All || m_debugDraw.Flags == cpDrawFlags.ContactPoints)
			{
				for (var i = 0; i < arbiters.Count; i++)
				{
					for (int j = 0; j < arbiters[i].contacts.Count; j++)
					{
						arbiters[i].contacts[j].Draw(m_debugDraw);
					}
					contacts += arbiters[i].contacts.Count;
				}

			}

			m_debugDraw.DrawString(0, 110, "Contact points: " + contacts);
			m_debugDraw.DrawString(0, 140, string.Format("Nodes:{1} Leaf:{0} Pairs:{2}", cp.numLeaves, cp.numNodes, cp.numPairs));


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

	public class cpPointQueryInfo
	{

		public cpPointQueryInfo(cpShape shape, cpVect point, float distance, cpVect gradient)
		{
			/// The nearest shape, NULL if no shape was within range.
			this.shape = shape;
			/// The closest point on the shape's surface. (in world space coordinates)
			this.point = point;
			/// The distance to the point. The distance is negative if the point is inside the shape.
			this.distance = distance;

			this.gradient = gradient;
		}

		public void Set(cpPointQueryInfo newPointInfo)
		{
			/// The nearest shape, NULL if no shape was within range.
			shape = newPointInfo.shape;
			point = newPointInfo.point;
			distance = newPointInfo.distance;
			gradient = newPointInfo.gradient;
			// g = newPointInfo.g;
		}

		/// The nearest shape, NULL if no shape was within range.
		public cpShape shape;
		/// The closest point on the shape's surface. (in world space coordinates)
		public cpVect point;
		/// The distance to the point. The distance is negative if the point is inside the shape.
		public float distance;
		/// The gradient of the signed distance function.
		/// The same as info.p/info.d, but accurate even for very small values of info.d.
		public cpVect gradient;
	}



}
