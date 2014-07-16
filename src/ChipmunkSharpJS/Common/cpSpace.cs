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

		public List<cpArbiter> arbiters { get; set; }

		public List<ContactPoint> contactBuffersHead;

		public Dictionary<string, cpArbiter> cachedArbiters;
		// public List<cpArbiter> pooledArbiters;
		public List<cpConstraint> constraints;

		// public cpArray allocatedBuffers;
		public int locked;

		public CollisionHandler defaultHandler { get; set; }
		//public Action<cpShape, cpShape> CollideShapes { get; set; }

		/// returns true from inside a callback and objects cannot be added/removed.
		public bool isLocked
		{
			get { return (locked != 0); }
		}


		public Dictionary<string, CollisionHandler> collisionHandlers;
		//public cpCollisionHandler defaultHandler;

		public bool skipPostStep;

		public List<Action> postStepCallbacks;

		//public CollisionHandler DefaultHandler { get; set; }
		#endregion

		public List<ContactPoint> collideShapes(cpShape a, cpShape b)
		{

			cp.assert((a as ICollisionShape).collisionCode <= (b as ICollisionShape).collisionCode, "Collided shapes must be sorted by type");
			return (a as ICollisionShape).collisionTable[(b as ICollisionShape).collisionCode](a, b);

		}

		public cpSpace()
		{

			this.stamp = 0;
			this.curr_dt = 0;

			this.bodies = new List<cpBody>();
			this.rousedBodies = new List<cpBody>();
			this.sleepingComponents = new List<cpBody>();

			this.staticShapes = new cpBBTree(null);
			this.activeShapes = new cpBBTree(this.staticShapes);

			this.arbiters = new List<cpArbiter>();
			this.contactBuffersHead = null;

			this.cachedArbiters = new Dictionary<string, cpArbiter>();
			//this.pooledArbiters = [];

			this.constraints = new List<cpConstraint>();

			this.locked = 0;

			this.collisionHandlers = new Dictionary<string, CollisionHandler>();

			this.defaultHandler = cp.defaultCollisionHandler;

			this.postStepCallbacks = new List<Action>();

			/// Number of iterations to use in the impulse solver to solve contacts.
			this.iterations = 10;

			/// Gravity to pass to rigid bodies when integrating velocity.
			this.gravity = cpVect.Zero;

			/// Damping rate expressed as the fraction of velocity bodies retain each second.
			/// A value of 0.9 would mean that each body's velocity will drop 10% per second.
			/// The default value is 1.0, meaning no damping is applied.
			/// @note This damping value is different than those of cpDampedSpring and cpDampedRotarySpring.
			this.damping = 1;

			/// Speed threshold for a body to be considered idle.
			/// The default value of 0 means to let the space guess a good threshold based on gravity.
			this.idleSpeedThreshold = 0;

			/// Time a group of bodies must remain idle in order to fall asleep.
			/// Enabling sleeping also implicitly enables the the contact graph.
			/// The default value of Infinity disables the sleeping algorithm.
			this.sleepTimeThreshold = cp.Infinity;

			/// Amount of encouraged penetration between colliding shapes..
			/// Used to reduce oscillating contacts and keep the collision cache warm.
			/// Defaults to 0.1. If you have poor simulation quality,
			/// increase this number as much as possible without allowing visible amounts of overlap.
			this.collisionSlop = 0.1f;

			/// Determines how fast overlapping shapes are pushed apart.
			/// Expressed as a fraction of the error remaining after each second.
			/// Defaults to pow(1.0 - 0.1, 60.0) meaning that Chipmunk fixes 10% of overlap each frame at 60Hz.
			this.collisionBias = (float)Math.Pow(1 - 0.1, 60);

			/// Number of frames that contact information should persist.
			/// Defaults to 3. There is probably never a reason to change this value.
			this.collisionPersistence = 3;

			/// Rebuild the contact graph during each step. Must be enabled to use the cpBodyEachArbiter() function.
			/// Disabled by default for a small performance boost. Enabled implicitly when the sleeping feature is enabled.
			this.enableContactGraph = false;

			/// The designated static body for this space.
			/// You can modify this body, or replace it with your own static body.
			/// By default it points to a statically allocated cpBody in the cpSpace struct.
			this.staticBody = new cpBody();


			// Cache the collideShapes callback function for the space.
			collideShapeFunc = this.makeCollideShapes();

		}

		public Action<object, object> collideShapeFunc { get; set; }

		public Action<object, object> makeCollideShapes()
		{
			// It would be nicer to use .bind() or something, but this is faster.
			return new Action<object, object>((obj1, obj2) =>
			{

				var a = obj1 as cpShape;
				var b = obj2 as cpShape;

				// Reject any of the simple cases
				if (
					// BBoxes must overlap
					//!bbIntersects(a.bb, b.bb)
					!(a.bb_l <= b.bb_r && b.bb_l <= a.bb_r && a.bb_b <= b.bb_t && b.bb_b <= a.bb_t)
					// Don't collide shapes attached to the same body.
					|| a.body == b.body
					// Don't collide objects in the same non-zero group
					|| (a.group != 0 && a.group == b.group)
					// Don't collide objects that don't share at least on layer.
					|| !(a.layers != 0 & b.layers != 0)
				) return;

				var handler = lookupHandler(a.collision_type, b.collision_type);

				var sensor = a.sensor || b.sensor;
				if (sensor && handler == cp.defaultCollisionHandler) return;

				// Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
				if ((a as ICollisionShape).collisionCode > (b as ICollisionShape).collisionCode)
				{
					var temp = a;
					a = b;
					b = temp;
				}

				// Narrow-phase collision detection.
				//cpContact *contacts = cpContactBufferGetArray(space);
				//int numContacts = cpCollideShapes(a, b, contacts);
				var contacts = collideShapes(a, b);
				if (contacts == null || contacts.Count == 0)
					return; // Shapes are not colliding.
				//cpSpacePushContacts(space, numContacts);

				// Get an arbiter from space.arbiterSet for the two shapes.
				// This is where the persistant contact magic comes from.
				var arbHash = cp.hashPair(a.hashid, b.hashid);

				cpArbiter arb;
				if (!cachedArbiters.TryGetValue(arbHash, out arb))
				{
					arb = new cpArbiter(a, b);
					cachedArbiters.Add(arbHash, arb);
				}

				arb.update(contacts, handler, a, b);

				// Call the begin function first if it's the first step
				if (arb.state == cpArbiterState.FirstColl && !handler.begin(arb, this))
				{
					arb.ignore(); // permanently ignore the collision until separation
				}

				if (
					// Ignore the arbiter if it has been flagged
					(arb.state != cpArbiterState.Ignore) &&
					// Call preSolve
					handler.preSolve(arb, this) &&
					// Process, but don't add collisions for sensors.
					!sensor
				)
				{
					this.arbiters.Add(arb);
				}
				else
				{
					//cpSpacePopContacts(space, numContacts);

					arb.contacts = null;

					// Normally arbiters are set as used after calling the post-solve callback.
					// However, post-solve callbacks are not called for sensors or arbiters rejected from pre-solve.
					if (arb.state != cpArbiterState.Ignore) arb.state = cpArbiterState.Normal;
				}

				// Time stamp the arbiter so we know it was used recently.
				arb.stamp = this.stamp;
			});
		}

		// Hashset filter func to throw away old arbiters.
		public bool arbiterSetFilter(cpArbiter arb)
		{
			var ticks = this.stamp - arb.stamp;

			cpBody a = arb.body_a, b = arb.body_b;

			// TODO should make an arbiter state for this so it doesn't require filtering arbiters for
			// dangling body pointers on body removal.
			// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
			// This prevents errant separate callbacks from happenening.
			if (
				(a.isStatic() || a.isSleeping()) &&
				(b.isStatic() || b.isSleeping())
			)
			{
				return true;
			}

			// Arbiter was used last frame, but not this one
			if (ticks >= 1 && arb.state != cpArbiterState.Cached)
			{
				arb.callSeparate(this);
				arb.state = cpArbiterState.Cached;
			}

			if (ticks >= this.collisionPersistence)
			{
				arb.contacts = null;

				//cpArrayPush(this.pooledArbiters, arb);
				return false;
			}

			return true;
		}

		//MARK: All Important cpSpaceStep() Function

		public void step(float dt)
		{
			// don't step if the timestep is 0!
			if (dt == 0) return;

			//assert(vzero.x == 0 && vzero.y == 0, "vzero is invalid");

			this.stamp++;

			var prev_dt = this.curr_dt;
			this.curr_dt = dt;

			int i;
			int j;


			var bodies = this.bodies;
			var constraints = this.constraints;
			var arbiters = this.arbiters;


			// Reset and empty the arbiter lists.
			for (i = 0; i < arbiters.Count; i++)
			{
				//var arb = ;
				arbiters[i].state = cpArbiterState.Normal;

				// If both bodies are awake, unthread the arbiter from the contact graph.
				if (!arbiters[i].body_a.isSleeping() && !arbiters[i].body_b.isSleeping())
				{
					arbiters[i].unthread();
				}
			}

			arbiters.Clear();

			Lock();
			{

				// Integrate positions
				for (i = 0; i < bodies.Count; i++)
				{
					bodies[i].position_func(dt);
				}

				// Find colliding pairs.
				this.activeShapes.each(s => cp.updateFunc(s as cpShape));
				this.activeShapes.reindexQuery(makeCollideShapes());

			} Unlock(false);

			// Rebuild the contact graph (and detect sleeping components if sleeping is enabled)
			this.processComponents(dt);

			Lock();
			{

				List<string> safeDelete = new List<string>();
				// Clear out old cached arbiters and call separate callbacks
				foreach (var hash in this.cachedArbiters)
				{
					if (!this.arbiterSetFilter(hash.Value))
						safeDelete.Add(hash.Key);
				}

				foreach (var item in safeDelete)
					cachedArbiters.Remove(item);

				// Prestep the arbiters and constraints.
				var slop = this.collisionSlop;
				var biasCoef = 1 - cp.cpfpow(this.collisionBias, dt);
				for (i = 0; i < arbiters.Count; i++)
				{
					arbiters[i].preStep(dt, slop, biasCoef);
				}

				for (i = 0; i < constraints.Count; i++)
				{
					var constraint = constraints[i];

					constraint.PreSolve(this);
					constraint.PreStep(dt);
				}

				// Integrate velocities.
				var damping = cp.cpfpow(this.damping, dt);
				var gravity = this.gravity;
				for (i = 0; i < bodies.Count; i++)
				{
					bodies[i].velocity_func(gravity, damping, dt);
				}

				// Apply cached impulses
				var dt_coef = (prev_dt == 0 ? 0 : dt / prev_dt);
				for (i = 0; i < arbiters.Count; i++)
				{
					arbiters[i].applyCachedImpulse(dt_coef);
				}

				for (i = 0; i < constraints.Count; i++)
				{
					constraints[i].ApplyCachedImpulse(dt_coef);
				}

				// Run the impulse solver.
				for (i = 0; i < this.iterations; i++)
				{
					for (j = 0; j < arbiters.Count; j++)
					{
						arbiters[j].applyImpulse(dt);
					}

					for (j = 0; j < constraints.Count; j++)
					{
						constraints[j].ApplyImpulse(dt);
					}
				}

				// Run the constraint post-solve callbacks
				for (i = 0; i < constraints.Count; i++)
				{
					constraints[i].postSolve(this);
				}

				// run the post-solve callbacks
				for (i = 0; i < arbiters.Count; i++)
				{
					arbiters[i].handler.postSolve(arbiters[i], this);
				}
			}
			this.Unlock(true);
		}


		public float getCurrentTimeStep()
		{
			return this.curr_dt;
		}

		public void setIterations(int iter) { this.iterations = iter; }


		public void useSpatialHash(int dim, int count)
		{

			throw new NotImplementedException("Spatial Hash not implemented.");

			//var staticShapes = new SpaceHash(dim, count, null);
			//var activeShapes = new SpaceHash(dim, count, staticShapes);

			//this.staticShapes.each(function(shape){
			//    staticShapes.insert(shape, shape.hashid);
			//});
			//this.activeShapes.each(function(shape){
			//    activeShapes.insert(shape, shape.hashid);
			//});

			//this.staticShapes = staticShapes;
			//this.activeShapes = activeShapes;
		}


		/// Update the collision detection info for the static shapes in the space.
		public void reindexStatic()
		{
			cp.assertSoft(!this.isLocked, "You cannot manually reindex objects while the space is locked. Wait until the current query or step is complete.");

			this.staticShapes.each(s =>
			{
				var shape = s as cpShape;
				var body = shape.body;
				shape.update(body.Position, body.Rotation);
			});
			this.staticShapes.reindex();

		}

		/// Update the collision detection data for a specific shape in the space.
		public void reindexShape(cpShape shape)
		{
			cp.assertHard(!isLocked, "You cannot manually reindex objects while the space is locked. Wait until the current query or step is complete.");

			var body = shape.body;
			shape.update(body.Position, body.Rotation);

			// attempt to rehash the shape in both hashes
			this.activeShapes.reindexObject(shape.hashid, shape);
			this.staticShapes.reindexObject(shape.hashid, shape);
		}

		/// Update the collision detection data for all shapes attached to a body.
		public void reindexShapesForBody(cpBody body)
		{

			foreach (var shape in body.shapeList)
			{
				this.reindexShape(shape);
			}

			//for (var shape = body.shapeList; shape != null; shape = shape.next)
			//    this.reindexShape(shape);
		}

		public void uncacheArbiter(cpArbiter arb)
		{
			cachedArbiters.Remove(arb.Key);
			arbiters.Remove(arb);
		}

		/// Test if a constraint has been added to the space.
		public bool containsConstraint(cpConstraint constraint)
		{

			return (constraint.space == this);

		}

		/// Test if a collision shape has been added to the space.
		public bool containsShape(cpShape shape)
		{
			return (shape.space == this);
		}
		/// Test if a rigid body has been added to the space.
		public bool containsBody(cpBody body)
		{
			return (body.space == this);
		}



		/// Remove a collision shape from the simulation.
		public void removeShape(cpShape shape)
		{
			var body = shape.body;
			if (body.isStatic())
			{
				this.removeStaticShape(shape);
			}
			else
			{
				cp.assertSoft(this.containsShape(shape),
					"Cannot remove a shape that was not added to the space. (Removed twice maybe?)");
				cp.assertSpaceUnlocked(this);

				body.activate();
				body.removeShape(shape);
				this.filterArbiters(body, shape);
				this.activeShapes.remove(shape.hashid);
				shape.space = null;
			}

		}
		/// Remove a collision shape added using cpSpaceAddStaticShape() from the simulation.
		public void removeStaticShape(cpShape shape)
		{

			cp.assertSoft(this.containsShape(shape),
		   "Cannot remove a static or sleeping shape that was not added to the space. (Removed twice maybe?)");
			cp.assertSpaceUnlocked(this);

			var body = shape.body;
			if (body.isStatic()) body.activateStatic(shape);
			body.removeShape(shape);
			this.filterArbiters(body, shape);
			this.staticShapes.remove(shape.hashid);
			shape.space = null;

		}

		/// Remove a rigid body from the simulation.
		public void removeBody(cpBody body)
		{
			cp.assertSoft(this.containsBody(body),
		 "Cannot remove a body that was not added to the space. (Removed twice maybe?)");
			cp.assertSpaceUnlocked(this);

			body.activate();
			//	this.filterArbiters(body, null);
			this.bodies.Remove(body);
			body.space = null;
		}



		/// Remove a constraint from the simulation.
		public void removeConstraint(cpConstraint constraint)
		{

			cp.assertSoft(this.containsConstraint(constraint),
		   "Cannot remove a constraint that was not added to the space. (Removed twice maybe?)");
			cp.assertSpaceUnlocked(this);

			constraint.a.activate();
			constraint.b.activate();

			this.constraints.Remove(constraint);

			constraint.a.removeConstraint(constraint);
			constraint.b.removeConstraint(constraint);
			constraint.space = null;
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
						arb.callSeparate(this);

					arb.unthread();

					this.arbiters.Remove(arb);
					safeDelete.Add(hash.Key);
				}

			}

			foreach (var item in safeDelete)
			{
				cachedArbiters.Remove(item);
			}

		}



		/// Add a rigid body to the simulation.
		public cpBody addBody(cpBody body)
		{
			cp.assertHard(!body.isStatic(), "Do not add static bodies to a space. Static bodies do not move and should not be simulated.");
			cp.assertHard(body.space != this, "You have already added this body to this space. You must not add it a second time.");
			cp.assertHard(body.space == null, "You have already added this body to another space. You cannot add it to a second.");
			cp.assertSpaceUnlocked(this);

			bodies.Add(body);
			body.space = this;
			return body;

		}
		/// Add a constraint to the simulation.
		public cpConstraint addConstraint(cpConstraint constraint)
		{

			//cp.assertHard(constraint.space != this, "You have already added this constraint to this space. You must not add it a second time.");
			cp.assertHard(constraint.space == null, "You have already added this constraint to another space. You cannot add it to a second.");
			//cp.assertHard(constraint.a != null && constraint.b != null, "Constraint is attached to a null body.");
			cp.assertSpaceUnlocked(this);

			cpBody a = constraint.a, b = constraint.b;

			a.activate();
			b.activate();
			this.constraints.Add(constraint);

			// Push onto the heads of the bodies' constraint lists
			constraint.next_a = a.constraintList; a.constraintList = constraint;
			constraint.next_b = b.constraintList; b.constraintList = constraint;
			constraint.space = this;

			return constraint;

		}


		/// Add a collision shape to the simulation.
		/// If the shape is attached to a static body, it will be added as a static shape.
		public cpShape addShape(cpShape shape)
		{

			var body = shape.body;

			if (shape.body.isStatic())
				return this.addStaticShape(shape);

			cp.assertHard(shape.space != this, "You have already added this shape to this space. You must not add it a second time.");
			cp.assertHard(shape.space == null, "You have already added this shape to another space. You cannot add it to a second.");
			cp.assertSpaceUnlocked(this);

			body.activate();
			body.addShape(shape);

			shape.update(body.Position, body.Rotation);
			this.activeShapes.insert(shape.hashid, shape);
			shape.space = this;

			return shape;

		}
		/// Explicity add a shape as a static shape to the simulation.
		public cpShape addStaticShape(cpShape shape)
		{

			// cpEnvironment.cpAssertHard(shape.space != this, "You have already added this shape to this space. You must not add it a second time.");
			// cpEnvironment.cpAssertHard(shape.space != null, "You have already added this shape to another space. You cannot add it to a second.");
			// cpEnvironment.cpAssertHard(shape.body.IsRogue(), "You are adding a static shape to a dynamic body. Did you mean to attach it to a static or rogue body? See the documentation for more information.");
			cp.assertSpaceUnlocked(this);


			var body = shape.body;
			body.addShape(shape);

			shape.update(body.Position, body.Rotation);
			this.staticShapes.insert(shape.hashid, shape);
			shape.space = this;

			return shape;

		}

		/// Call @c func for each shape in the space.
		// **** Iteration
		public void eachBody(Action<cpBody> func)
		{
			Lock();
			{
				var bodies = this.bodies;

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

		public void eachShape(Action<cpShape> func)
		{
			this.Lock();
			{
				this.activeShapes.each(s => func(s as cpShape));
				this.staticShapes.each(s => func(s as cpShape));
			} this.Unlock(true);
		}

		public void eachConstraint(Action<cpConstraint> func)
		{
			Lock();
			{
				var constraints = this.constraints;
				for (var i = 0; i < constraints.Count; i++)
					func(constraints[i]);
			} this.Unlock(true);
		}


		public CollisionHandler lookupHandler(string a, string b)
		{
			CollisionHandler test;
			if (collisionHandlers.TryGetValue(cp.hashPair(a, b), out test))
				return test;
			else
				return defaultHandler;
		}

		/// Unset a collision handler.
		public void removeCollisionHandler(string a, string b)
		{
			cp.assertSpaceUnlocked(this);
			collisionHandlers.Remove(cp.hashPair(a, b));

		}

		/// Set a collision handler to be used whenever the two shapes with the given collision types collide.
		/// You can pass null for any function you don't want to implement.
		public void addCollisionHandler(string a, string b,
			Func<cpArbiter, cpSpace, bool> begin, Func<cpArbiter, cpSpace, bool> preSolve, Action<cpArbiter, cpSpace> postSolve, Action<cpArbiter, cpSpace> separate)
		{

			cp.assertSpaceUnlocked(this);

			// Remove any old function so the new one will get added.
			this.removeCollisionHandler(a, b);

			var handler = new CollisionHandler();
			handler.a = a;
			handler.b = b;
			if (begin != null)
				handler.begin = begin;
			if (preSolve != null)
				handler.preSolve = preSolve;
			if (postSolve != null)
				handler.postSolve = postSolve;
			if (separate != null)
				handler.separate = separate;

			collisionHandlers.Add(cp.hashPair(a, b), handler);
		}




		public void activateBody(cpBody body)
		{

			cp.assert(!body.isRogue(), "Internal error: Attempting to activate a rogue body.");

			if (this.isLocked)
			{
				// cpSpaceActivateBody() is called again once the space is unlocked
				if (this.rousedBodies.IndexOf(body) == -1) this.rousedBodies.Add(body);
			}
			else
			{
				this.bodies.Add(body);

				for (var i = 0; i < body.shapeList.Count; i++)
				{
					var shape = body.shapeList[i];
					this.staticShapes.remove(shape.hashid);
					this.activeShapes.insert(shape.hashid, shape);
				}

				for (var arb = body.arbiterList; arb != null; arb = arb.next(body))
				{
					var bodyA = arb.body_a;
					if (body == bodyA || bodyA.isStatic())
					{

						// Reinsert the arbiter into the arbiter cache
						cpShape a = arb.a, b = arb.b;
						cachedArbiters.Add(cp.hashPair(a.hashid, b.hashid), arb);

						// Update the arbiter's state
						arb.stamp = this.stamp;
						arb.handler = this.lookupHandler(a.collision_type, b.collision_type);
						this.arbiters.Add(arb);
					}
				}

				for (var constraint = body.constraintList; constraint != null; constraint = constraint.next(null))
				{
					var bodyA = constraint.a;
					if (body == bodyA || bodyA.isStatic()) this.constraints.Add(constraint);
				}
			}

		}


		public void deactivateBody(cpBody body)
		{
			cp.assert(!body.isRogue(), "Internal error: Attempting to deactivate a rogue body.");

			this.bodies.Remove(body);


			for (var i = 0; i < body.shapeList.Count; i++)
			{
				var shape = body.shapeList[i];
				this.activeShapes.remove(shape.hashid);
				this.staticShapes.insert(shape.hashid, shape);
			}

			for (var arb = body.arbiterList; arb != null; arb = arb.next(body))
			{
				var bodyA = arb.body_a;
				if (body == bodyA || bodyA.isStatic())
				{
					this.uncacheArbiter(arb);

					// Save contact values to a new block of memory so they won't time out
					//size_t bytes = arb.numContacts*sizeof(cpContact);
					//cpContact *contacts = (cpContact *)cpcalloc(1, bytes);
					//memcpy(contacts, arb.contacts, bytes);
					//arb.contacts = contacts;
				}
			}

			for (var constraint = body.constraintList; constraint != null; constraint = constraint.next(null))
			{
				var bodyA = constraint.a;
				if (body == bodyA || bodyA.isStatic())
					this.constraints.Remove(constraint);
			}
		}


		public void processComponents(float dt)
		{
			var sleep = (this.sleepTimeThreshold != cp.Infinity);
			var bodies = this.bodies;

			// These checks can be removed at some stage (if DEBUG == undefined)
			for (var i = 0; i < bodies.Count; i++)
			{
				var body = bodies[i];

				cp.assertSoft(body.nodeNext == null, "Internal Error: Dangling next pointer detected in contact graph.");
				cp.assertSoft(body.nodeRoot == null, "Internal Error: Dangling root pointer detected in contact graph.");
			}

			// Calculate the kinetic energy of all the bodies
			if (sleep)
			{
				var dv = this.idleSpeedThreshold;
				var dvsq = (dv != 0 ? dv * dv : cpVect.cpvlengthsq(this.gravity) * dt * dt);

				for (var i = 0; i < bodies.Count; i++)
				{
					//var body = ;

					// Need to deal with infinite mass objects
					var keThreshold = (dvsq != 0 ? bodies[i].Mass * dvsq : 0);
					bodies[i].nodeIdleTime = (bodies[i].kineticEnergy() > keThreshold ? 0 : bodies[i].nodeIdleTime + dt);
				}
			}

			// Awaken any sleeping bodies found and then push arbiters to the bodies' lists.

			var count = arbiters.Count; //FIX: we cannot read the count values of the array because it changes inside

			for (int i = 0; i < count; i++)
			{
				cpArbiter arb = arbiters[i];
				cpBody a = arb.body_a, b = arb.body_b;

				if (sleep)
				{

					if ((b.isRogue() && !b.isStatic()) || a.isSleeping())
						a.activate();

					if ((a.isRogue() && !a.isStatic()) || b.isSleeping())
						b.activate();
				}

				a.pushArbiter(arb);
				b.pushArbiter(arb);
			}



			if (sleep)
			{
				// Bodies should be held active if connected by a joint to a non-static rouge body.
				var constraints = this.constraints;
				for (var i = 0; i < constraints.Count; i++)
				{
					cpConstraint constraint = constraints[i];
					cpBody a = constraint.a, b = constraint.b;

					if (b.isRogue() && !b.isStatic())
						a.activate();

					if (a.isRogue() && !a.isStatic())
						b.activate();
				}

				// Generate components and deactivate sleeping ones
				for (var i = 0; i < bodies.Count; )
				{
					var body = bodies[i];

					if (cp.componentRoot(body) == null)
					{
						// Body not in a component yet. Perform a DFS to flood fill mark 
						// the component in the contact graph using this body as the root.
						cp.floodFillComponent(body, body);

						// Check if the component should be put to sleep.
						if (!cp.componentActive(body, this.sleepTimeThreshold))
						{
							this.sleepingComponents.Add(body);
							for (var other = body; other != null; other = other.nodeNext)
							{
								this.deactivateBody(other);
							}

							// deactivateBody() removed the current body from the list.
							// Skip incrementing the index counter.
							continue;
						}
					}

					i++;

					// Only sleeping bodies retain their component node pointers.
					body.nodeRoot = null;
					body.nodeNext = null;
				}
			}
		}

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
			m_debugDraw.DrawString(0, 50, string.Format("Bodies : {0}/{1}", bodies.Count, bodies.Capacity));
			m_debugDraw.DrawString(0, 80, string.Format("Arbiters: {0}/{1}", arbiters.Count, arbiters.Capacity));

			var contacts = 0;
			for (var i = 0; i < arbiters.Count; i++)
			{
				contacts += arbiters[i].contacts.Count;
			}

			m_debugDraw.DrawString(0, 110, "Contact points: " + contacts);
			m_debugDraw.DrawString(0, 140, string.Format("Nodes:{1} Leaf:{0} Pairs:{2}", cp.numLeaves, cp.numNodes, cp.numPairs));

			//cpDrawFlags flags = m_debugDraw.Flags;

			if (m_debugDraw.Flags == cpDrawFlags.ALL || m_debugDraw.Flags == cpDrawFlags.Shape)
			{
				eachShape(s => s.Draw(m_debugDraw));
			}

			if (m_debugDraw.Flags == cpDrawFlags.ALL || m_debugDraw.Flags == cpDrawFlags.Joint)
			{
				eachConstraint(c => c.Draw(m_debugDraw));
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


		public cpSegmentQueryInfo segmentQueryFirst(cpVect start, cpVect end, int layers, int group)
		{
			cpSegmentQueryInfo output = null;

			var helper = new Func<object, float>(o1 =>
			{
				cpShape shape = o1 as cpShape;

				cpSegmentQueryInfo info = shape.segmentQuery(start, end);


				if (
					!(shape.group != 0 && group == shape.group) && (layers != 0 & shape.layers != 0) &&
					!shape.sensor && info != null &&
					(output == null || info.t < output.t)
				)
				{
					output = info;
				}

				return output != null ? output.t : 1;
			}
				);

			this.staticShapes.SegmentQuery(start, end, 1f, helper);
			this.activeShapes.SegmentQuery(start, end, output != null ? output.t : 1, helper);

			return output;
		}


		//		Space.prototype.nearestPointQuery = function(point, maxDistance, layers, group, func)
		//{
		//	var helper = function(shape){
		//		if(!(shape.group && group === shape.group) && (layers & shape.layers)){
		//			var info = shape.nearestPointQuery(point);

		//			if(info.d < maxDistance) func(shape, info.d, info.p);
		//		}
		//	};

		//	var bb = bbNewForCircle(point, maxDistance);

		//	this.lock(); {
		//		this.activeShapes.query(bb, helper);
		//		this.staticShapes.query(bb, helper);
		//	} this.unlock(true);
		//};

		public void NearestPointQuery(cpVect point, int maxDistance, int layers, int group, Action<cpShape, float, cpVect> func)
		{
			var helper = new Action<object, object>((o1, o2) =>
			{
				cpShape shape = o1 as cpShape;

				if (!(shape.group != 0 && group == shape.group) && (layers != 0 & shape.layers != 0))
				{
					cpNearestPointQueryInfo info = shape.nearestPointQuery(point);

					if (info.d < maxDistance)
						func(shape, info.d, info.p);

				}
			});

			cpBB bb = cp.bbNewForCircle(point, maxDistance);
			this.Lock();
			{
				this.activeShapes.query(bb, helper);
				this.staticShapes.query(bb, helper);
			} this.Unlock(true);
		}

		//public cpNearestPointQueryInfo NearestPointQuery(cpVect point, int maxDistance, int layers, int group)
		//{

		//	cpNearestPointQueryInfo output = null;

		//	var helper = new Action<object, object>((o1, o2) =>
		//	{
		//		cpShape shape = o1 as cpShape;

		//		if (!(shape.group != 0 && group == shape.group) && (layers != 0 & shape.layers != 0) && !shape.sensor)
		//		{
		//			cpNearestPointQueryInfo info = shape.nearestPointQuery(point);

		//			if (info.d < maxDistance && (output == null || info.d < output.d))
		//				output = info;
		//		}
		//	});

		//	cpBB bb = cp.bbNewForCircle(point, maxDistance);

		//	this.activeShapes.query(bb, helper);
		//	this.staticShapes.query(bb, helper);

		//	return output;
		//}

		public class NearestPointQueryInfo
		{

			public NearestPointQueryInfo(cpShape shape, cpVect p, float d)
			{
				/// The nearest shape, NULL if no shape was within range.
				this.shape = shape;
				/// The closest point on the shape's surface. (in world space coordinates)
				this.p = p;
				/// The distance to the point. The distance is negative if the point is inside the shape.
				this.d = d;
			}

			public cpShape shape { get; set; }
			public cpVect p { get; set; }
			public float d { get; set; }
		}

		public cpNearestPointQueryInfo NearestPointQuery(cpVect point, int maxDistance, int layers, int group)
		{

			cpNearestPointQueryInfo output = null;

			var helper = new Action<object, object>((o1, o2) =>
			{
				cpShape shape = o1 as cpShape;

				if (!(shape.group > 0 && group == shape.group) && (layers > 0 & shape.layers > 0) && !shape.sensor)
				{
					cpNearestPointQueryInfo info = shape.nearestPointQuery(point);

					if (info.d < maxDistance && (output == null || info.d < output.d))
						output = info;
				}
			});

			cpBB bb = cp.bbNewForCircle(point, maxDistance);

			this.activeShapes.query(bb, helper);
			this.staticShapes.query(bb, helper);

			return output;
		}

		public void clear()
		{
			List<object> safeList = new List<object>();
			eachBody((b) => safeList.Add(b));
			foreach (var item in safeList)
				removeBody(item as cpBody);

			safeList.Clear();
			eachShape((b) => safeList.Add(b));
			foreach (var item in safeList)
				removeShape(item as cpShape);

			safeList.Clear();

			eachConstraint((b) => safeList.Add(b));
			foreach (var item in safeList)
				removeConstraint(item as cpConstraint);

			activeShapes.leaves.Clear();
			staticShapes.leaves.Clear();

			constraints.Clear();
			collisionHandlers.Clear();
			bodies.Clear();
			cachedArbiters.Clear();
			arbiters.Clear();

			cp.step = 0;
			stamp = 0;
			cp.numLeaves = 0; cp.numNodes = 0; cp.numPairs = 0;

			//foreach (var item in space.collisionHandlers)
			//	space.removeCollisionHandler(item.Value.a, item.Value.b);

		}
	}
}
