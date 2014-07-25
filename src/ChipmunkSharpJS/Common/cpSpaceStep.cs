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

	public class cpContactBufferHeader
	{
		public int stamp;
		public cpContactBufferHeader next;
		public int numContacts;
	};

	public class cpPostStepCallback
	{
		public Action<cpSpace, object, object> func;
		public object key;
		public object data;
	}



	public partial class cpSpace
	{

		// **** Post Step Callback Functions

		static void PostStepDoNothing(cpSpace space, object obj, object data) { }


		public cpPostStepCallback GetPostStepCallback(object key)
		{
			List<cpPostStepCallback> arr = postStepCallbacks;
			for (int i = 0; i < arr.Count; i++)
			{
				cpPostStepCallback callback = arr[i];
				if (callback != null && callback.key == key)
					return callback;
			}

			return null;
		}

		public bool AddPostStepCallback(Action<cpSpace, object, object> func, object key, object data)
		{

			//cpPostStepCallback func
			cp.assertSoft(this.IsLocked,
		 "Adding a post-step callback when the space is not locked is unnecessary. " +
		 "Post-step callbacks will not called until the end of the next call to cpSpaceStep() or the next query.");


			if (GetPostStepCallback(key) == null)
			{
				cpPostStepCallback callback = new cpPostStepCallback();// (cpPostStepCallback*)cpcalloc(1, sizeof(cpPostStepCallback));
				callback.func = (func != null ? func : PostStepDoNothing);
				callback.key = key;
				callback.data = data;
				this.postStepCallbacks.Add(callback);
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
			locked++;
		}

		public void Unlock(bool runPostStep)
		{

			this.locked--;
			cp.assertHard(this.locked >= 0, "Internal Error: Space lock underflow.");

			if (this.locked == 0)
			{
				List<cpBody> waking = this.rousedBodies;

				for (int i = 0, count = waking.Count; i < count; i++)
				{
					this.activateBody(waking[i]);
					waking[i] = null;
				}

				this.rousedBodies.Clear();


				if (this.locked == 0 && runPostStep && !this.skipPostStep)
				{
					this.skipPostStep = true;

					List<cpPostStepCallback> arr = this.postStepCallbacks;
					for (int i = 0; i < arr.Count; i++)
					{
						cpPostStepCallback callback = (cpPostStepCallback)arr[i];
						var func = callback.func;

						// Mark the func as NULL in case calling it calls cpSpaceRunPostStepCallbacks() again.
						// TODO: need more tests around this case I think.
						callback.func = null;
						if (func != null)
							func(this, callback.key, callback.data);

						arr[i] = null;

					}
					this.postStepCallbacks.Clear();
					this.skipPostStep = false;
				}
			}

		}

		//MARK: Collision Detection Functions

		public bool QueryRejectConstraint(cpBody a, cpBody b)
		{
			bool returnValue = false;
			a.EachConstraint(

				(constraint, o) =>
				{
					if (!constraint.collideBodies && ((constraint.a == a && constraint.b == b) || (constraint.a == b && constraint.b == a)))
						returnValue = true;

				}

				, null);

			return returnValue;
		}

		public bool QueryReject(cpShape a, cpShape b)
		{
			return (
				// BBoxes must overlap
		!a.bb.Intersects(b.bb)
				// Don't collide shapes attached to the same body.
		|| a.body == b.body
				// Don't collide shapes that are filtered.
		|| a.filter.Reject(b.filter)
				// Don't collide bodies if they have a constraint with collideBodies == cpFalse.
		|| QueryRejectConstraint(a.body, b.body)
	);

		}

		//TODO: THIS METHOD NEEDS A REVISION FROM ORIGIAL
		public List<cpContact> CollideShapes(cpShape a, cpShape b)
		{

			// Reject any of the simple cases
			if (QueryReject(a, b))
				return new List<cpContact>();

			// Narrow-phase collision detection.
			//cpCollisionInfo info = cpCollide(a, b);

			if ((int)a.shapeType > (int)b.shapeType)
			{
				var tmp = a;
				a = b;
				b = tmp;
			}
			//cp.assert((a as ICollisionShape).CollisionCode <= (b as ICollisionShape).CollisionCode, "Collided shapes must be sorted by type");
			return (a as ICollisionShape).CollisionTable[(b as ICollisionShape).CollisionCode](a, b);

		}

		// Hashset filter func to throw away old arbiters.
		public bool ArbiterSetFilter(cpArbiter arb)
		{
			var ticks = this.stamp - arb.stamp;

			cpBody a = arb.body_a, b = arb.body_b;

			// TODO should make an arbiter state for this so it doesn't require filtering arbiters for
			// dangling body pointers on body removal.
			// Preserve arbiters on sensors and rejected arbiters for sleeping objects.
			// This prevents errant separate callbacks from happenening.
			if (
				(a.bodyType == cpBodyType.STATIC || a.IsSleeping()) &&
				(b.bodyType == cpBodyType.STATIC || b.IsSleeping())
			)
			{
				return true;
			}

			// Arbiter was used last frame, but not this one
			if (ticks >= 1 && arb.state != cpArbiterState.Cached)
			{
				arb.state = cpArbiterState.Cached;

				arb.handler.separateFunc(arb, this, arb.handler.userData);

			}

			if (ticks >= this.collisionPersistence)
			{
				arb.contacts = null;

				//cpArrayPush(this.pooledArbiters, arb);
				return false;
			}

			return true;
		}


		public void step(float dt)
		{
			// don't step if the timestep is 0!
			if (dt == 0) return;

			this.stamp++;

			float prev_dt = this.curr_dt;
			this.curr_dt = dt;

			int i, j;

			var bodies = this.dynamicBodies;
			var constraints = this.constraints;
			var arbiters = this.arbiters;


			// Reset and empty the arbiter lists.
			for (i = 0; i < arbiters.Count; i++)
			{
				//var arb = ;
				arbiters[i].state = cpArbiterState.Normal;

				// If both bodies are awake, unthread the arbiter from the contact graph.
				if (!arbiters[i].body_a.IsSleeping() && !arbiters[i].body_b.IsSleeping())
				{
					arbiters[i].Unthread();
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
				this.dynamicShapes.Each(s => cpShape.UpdateFunc(s as cpShape, null));
				this.dynamicShapes.ReindexQuery((a, b) => CollideShapes(a as cpShape, b as cpShape));

			} Unlock(false);

			// Rebuild the contact graph (and detect sleeping components if sleeping is enabled)
			this.processComponents(dt);

			Lock();
			{


				List<string> safeDelete = new List<string>();
				// Clear out old cached arbiters and call separate callbacks
				foreach (var hash in this.cachedArbiters)
				{
					if (!this.ArbiterSetFilter(hash.Value))
						safeDelete.Add(hash.Key);
				}

				foreach (var item in safeDelete)
					cachedArbiters.Remove(item);

				// Prestep the arbiters and constraints.
				var slop = this.collisionSlop;
				var biasCoef = 1 - cp.cpfpow(this.collisionBias, dt);
				for (i = 0; i < arbiters.Count; i++)
				{
					arbiters[i].PreStep(dt, slop, biasCoef);
				}

				for (i = 0; i < constraints.Count; i++)
				{
					if (constraints[i].preSolve != null)
						constraints[i].preSolve(this);
					constraints[i].PreStep(dt);
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
					arbiters[i].ApplyCachedImpulse(dt_coef);
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
						arbiters[j].ApplyImpulse(dt);
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
					arbiters[i].handler.postSolveFunc(arbiters[i], this, null);
				}
			}
			this.Unlock(true);
		}


		//////////////////////////////////////////////////////////////////

		//public void runPostStepCallbacks()
		//{
		//	// Don't cache length because post step callbacks may add more post step callbacks
		//	// directly or indirectly.
		//	for (var i = 0; i < this.postStepCallbacks.Count; i++)
		//		this.postStepCallbacks[i].;
		//	this.postStepCallbacks.Clear();

		//}



		//static bool QueryReject(cpShape a, cpShape b)
		//{
		//    return (
		//        // BBoxes must overlap
		//        !a.bb.Intersects(b.bb)
		//        // Don't collide shapes attached to the same body.
		//        || a.body == b.body
		//        // Don't collide objects in the same non-zero group
		//        || (a.group != 0 && a.group == b.group)
		//        // Don't collide objects that don't share at least on layer.
		//        || !(a.layers != 0 & b.layers != 0)
		//        // Don't collide infinite mass objects
		//        || (a.body.Mass == cpEnvironment.Infinity && b.body.Mass == cpEnvironment.Infinity)
		//    );
		//}

		//// Callback from the spatial hash.
		//public List<ContactPoint> collideShapes(cpShape a, cpShape b)
		//{
		//    cpEnvironment.assertWarn((a as ICollisionShape).collisionCode <= (b as ICollisionShape).collisionCode, "Collided shapes must be sorted by type");
		//    return (a as ICollisionShape).collisionTable[(b as ICollisionShape).collisionCode](a, b);
		//    //return null;

		//}

		//private void UpdateFunc(cpShape shape)
		//{
		//    var body = shape.body;
		//    shape.update(body.Position, body.Rotation);
		//}


	}
}

