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
using System.Linq;
using System;
using System.Collections.Generic;

namespace ChipmunkSharp
{

	public class PointsDistance
	{
		public cpVect pointA, pointB;
		/// Penetration distance of the two shapes. Overlapping means it will be negative.
		/// This value is calculated as cpvdot(cpvsub(point2, point1), normal) and is ignored by cpArbiterSetContactPointSet().
		public float distance;
	}

	public class cpContactPointSet
	{

		public int count;

		/// The normal of the collision.
		public cpVect normal;

		public PointsDistance[] points;
	}

	/// @private
	public class cpCollisionHandler
	{

		public static bool AlwaysCollide(cpArbiter arb, cpSpace space, object data) { return true; }
		public static void DoNothing(cpArbiter arb, cpSpace space, object data) { }

		public static cpCollisionHandler cpCollisionHandlerDoNothing
		{
			get
			{

				return new cpCollisionHandler(
				   cp.WILDCARD_COLLISION_TYPE,
	   cp.WILDCARD_COLLISION_TYPE,
	   AlwaysCollide,
	   AlwaysCollide,
	   DoNothing,
	   DoNothing, null
					);


			}
		}


		// Use the wildcard identifier since  the default handler should never match any type pair.
		public static cpCollisionHandler cpCollisionHandlerDefault
		{
			get
			{
				return new cpCollisionHandler(
	   cp.WILDCARD_COLLISION_TYPE,
	   cp.WILDCARD_COLLISION_TYPE,
	   DefaultBegin,
	   DefaultPreSolve,
	   DefaultPostSolve,
	   DefaultSeparate, null
	   );
			}
		}

		public ulong typeA;
		public ulong typeB;

		public Func<cpArbiter, cpSpace, object, bool> beginFunc;
		public Func<cpArbiter, cpSpace, object, bool> preSolveFunc;
		public Action<cpArbiter, cpSpace, object> postSolveFunc;
		public Action<cpArbiter, cpSpace, object> separateFunc;

		public object userData;

		public cpCollisionHandler()
		{
			this.typeA = cp.WILDCARD_COLLISION_TYPE;
			this.typeB = cp.WILDCARD_COLLISION_TYPE;

			beginFunc = DefaultBegin;
			preSolveFunc = DefaultPreSolve;
			postSolveFunc = DefaultPostSolve;
			separateFunc = DefaultSeparate;
		}


		public cpCollisionHandler(ulong a, ulong b,
			Func<cpArbiter, cpSpace, object, bool> begin,
			Func<cpArbiter, cpSpace, object, bool> preSolve,
			Action<cpArbiter, cpSpace, object> postSolve,
			Action<cpArbiter, cpSpace, object> separate,
			object userData

			)
		{
			this.typeA = a;
			this.typeB = b;

			this.beginFunc = begin;
			this.preSolveFunc = preSolve;
			this.postSolveFunc = postSolve;
			this.separateFunc = separate;
			this.userData = userData;
		}


		public cpCollisionHandler Clone()
		{
			cpCollisionHandler copy = new cpCollisionHandler();
			copy.typeA = typeA;
			copy.typeB = typeB;

			copy.beginFunc = beginFunc;
			copy.preSolveFunc = preSolveFunc;
			copy.postSolveFunc = postSolveFunc;
			copy.separateFunc = separateFunc;

			copy.userData = userData;


			return copy;
		}

		public static bool DefaultBegin(cpArbiter arb, cpSpace space, object o)
		{
			return true;
		}


		public static bool DefaultPreSolve(cpArbiter arb, cpSpace space, object o)
		{
			return true;
		}

		public static void DefaultPostSolve(cpArbiter arb, cpSpace space, object o)
		{
		}

		public static void DefaultSeparate(cpArbiter arb, cpSpace space, object o)
		{
		}


		// Equals function for collisionHandlers.
		public static bool SetEql(cpCollisionHandler check, cpCollisionHandler pair)
		{
			return (
				(check.typeA == pair.typeA && check.typeB == pair.typeB) ||
				(check.typeB == pair.typeA && check.typeA == pair.typeB)
			);
		}



	};

	/// @private
	public enum cpArbiterState
	{
		// Arbiter is active and its the first collision.
		FirstCollision = 1,
		// Arbiter is active and its not the first collision.
		Normal = 2,
		// Collision has been explicitly ignored.
		// Either by returning false from a begin collision handler or calling cpArbiterIgnore().
		Ignore = 3,
		// Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
		Cached = 4,
		Invalidated = 5
	} ;

	/// A colliding pair of shapes.
	public class cpArbiter
	{
		public ulong Key { get { return cp.CP_HASH_PAIR(a.hashid, b.hashid); } }

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

		public cpShape a, b;

		public cpBody body_a, body_b;

		public cpArbiterThread thread_a, thread_b;

		public List<cpContact> contacts { get; set; }
		cpVect n;

		public cpCollisionHandler handler, handlerA, handlerB;
		public bool swapped;

		public int stamp;
		public cpArbiterState state;

		#endregion

		public int Count { get { return contacts.Count; } }

		/// A colliding pair of shapes.
		public cpArbiter(cpShape a, cpShape b)
		{
			this.handler = null;
			this.swapped = false;

			this.handlerA = null;
			this.handlerB = null;

			/// Calculated value to use for the elasticity coefficient.
			/// Override in a pre-solve collision handler for custom behavior.
			this.e = 0;
			/// Calculated value to use for the friction coefficient.
			/// Override in a pre-solve collision handler for custom behavior.
			this.u = 0;
			/// Calculated value to use for applying surface velocities.
			/// Override in a pre-solve collision handler for custom behavior.
			this.surface_vr = cpVect.Zero;

			this.a = a; this.body_a = a.body;
			this.b = b; this.body_b = b.body;

			this.thread_a = new cpArbiterThread(null, null);
			this.thread_b = new cpArbiterThread(null, null);

			this.contacts = new List<cpContact>();

			this.stamp = 0;

			this.state = cpArbiterState.FirstCollision;
		}


		public void Unthread()
		{
			UnthreadHelper(this, this.body_a, this.thread_a.prev, this.thread_a.next);
			UnthreadHelper(this, this.body_b, this.thread_b.prev, this.thread_b.next);
			this.thread_a.prev = null;
			this.thread_a.next = null;
			this.thread_b.prev = null;
			this.thread_b.next = null;
		}

		//public static void UnthreadHelper(cpArbiter arb, cpBody body)
		//{

		//	cpArbiterThread thread = arb.ThreadForBody(body);

		//	cpArbiter prev = thread.prev;
		//	cpArbiter next = thread.next;

		//	// thread_x_y is quite ugly, but it avoids making unnecessary js objects per arbiter.
		//	if (prev != null)
		//	{
		//		cpArbiterThread nextPrev = prev.ThreadForBody(body);
		//		nextPrev.next = next;
		//	}
		//	else if (body.arbiterList == arb)
		//	{
		//		// IFF prev is NULL and body->arbiterList == arb, is arb at the head of the list.
		//		// This function may be called for an arbiter that was never in a list.
		//		// In that case, we need to protect it from wiping out the body->arbiterList pointer.
		//		body.arbiterList = next;
		//	}

		//	if (next != null)
		//	{
		//		cpArbiterThread threadNext = next.ThreadForBody(body);
		//		threadNext.prev = prev;
		//	}

		//	thread.prev = null;
		//	thread.next = null;

		//}


		public static void UnthreadHelper(cpArbiter arb, cpBody body, cpArbiter prev, cpArbiter next)
		{
			// thread_x_y is quite ugly, but it avoids making unnecessary js objects per arbiter.
			if (prev != null)
			{
				// cpArbiterThreadForBody(prev, body)->next = next;
				if (prev.body_a == body)
				{
					prev.thread_a.next = next;
				}
				else
				{
					prev.thread_b.next = next;
				}
			}
			else
			{
				body.arbiterList = next;
			}

			if (next != null)
			{
				// cpArbiterThreadForBody(next, body)->prev = prev;
				if (next.body_a == body)
				{
					next.thread_a.prev = prev;
				}
				else
				{
					next.thread_b.prev = prev;
				}
			}
		}

		/// Returns true if this is the first step a pair of objects started colliding.
		public bool IsFirstContact()
		{
			return state == cpArbiterState.FirstCollision;
		}

		public bool IsRemoval()
		{
			return state == cpArbiterState.Invalidated;
		}

		public int GetCount()
		{
			// Return 0 contacts if we are in a separate callback.
			return ((int)state < (int)cpArbiterState.Cached ? Count : 0);
		}

		/// Get the normal of the @c ith contact point.
		public cpVect GetNormal()
		{
			return cpVect.cpvmult(n, this.swapped ? -1.0f : 1.0f);
		}

		public cpVect GetPointA(int i)
		{
			cp.AssertHard(0 <= i && i < GetCount(), "Index error: The specified contact index is invalid for this arbiter");
			return cpVect.cpvadd(this.body_a.p, this.contacts[i].r1);
		}

		public cpVect GetPointB(int i)
		{
			cp.AssertHard(0 <= i && i < GetCount(), "Index error: The specified contact index is invalid for this arbiter");
			return cpVect.cpvadd(this.body_b.p, this.contacts[i].r2);
		}

		/// Get the depth of the @c ith contact point.
		public float GetDepth(int i)
		{
			// return this.contacts[i].dist;
			cp.AssertHard(0 <= i && i < GetCount(), "Index error: The specified contact index is invalid for this arbiter");

			cpContact con = contacts[i];
			return cpVect.cpvdot(cpVect.cpvadd(cpVect.cpvsub(con.r2, con.r1), cpVect.cpvsub(this.body_b.p, this.body_a.p)), this.n);
		}


		/// Return a contact set from an arbiter.
		public cpContactPointSet GetContactPointSet()
		{
			cpContactPointSet set = new cpContactPointSet();
			set.count = GetCount();
			set.points = new PointsDistance[set.count];

			bool swapped = this.swapped;

			set.normal = (swapped ? cpVect.cpvneg(this.n) : this.n);

			for (int i = 0; i < set.count; i++)
			{
				// Contact points are relative to body CoGs;
				cpVect p1 = cpVect.cpvadd(this.body_a.p, this.contacts[i].r1);
				cpVect p2 = cpVect.cpvadd(this.body_b.p, this.contacts[i].r2);

				set.points[i] = new PointsDistance();
				set.points[i].pointA = (swapped ? p2 : p1);
				set.points[i].pointB = (swapped ? p1 : p2);
				set.points[i].distance = cpVect.cpvdot(cpVect.cpvsub(p2, p1), this.n);

			}
			return set;
		}

		public void SetContactPointSet(ref cpContactPointSet set)
		{
			int count = set.count;

			cp.AssertHard(count == Count, "The number of contact points cannot be changed.");

			this.n = (this.swapped ? cpVect.cpvneg(set.normal) : set.normal);

			for (int i = 0; i < count; i++)
			{
				// Convert back to CoG relative offsets.
				cpVect p1 = set.points[i].pointA;
				cpVect p2 = set.points[i].pointB;

				this.contacts[i].r1 = cpVect.cpvsub(swapped ? p2 : p1, this.body_a.p);
				this.contacts[i].r2 = cpVect.cpvsub(swapped ? p1 : p2, this.body_b.p);
			}

		}

		/// Calculate the total impulse that was applied by this arbiter.
		/// This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.

		public cpVect TotalImpulse()
		{

			cpVect sum = cpVect.Zero;

			for (int i = 0, count = GetCount(); i < count; i++)
			{
				cpContact con = contacts[i];
				// sum.Add(con.n.Multiply(con.jnAcc));
				sum = cpVect.cpvadd(sum, cpVect.cpvrotate(n, new cpVect(con.jnAcc, con.jtAcc)));
			}

			return this.swapped ? sum : cpVect.cpvneg(sum);
		}

		/// Calculate the amount of energy lost in a collision including static, but not dynamic friction.
		/// This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.
		public float TotalKE()
		{

			float eCoef = (1f - this.e) / (1f + this.e);
			float sum = 0.0f;

			//cpContact contacts = contacts;

			for (int i = 0, count = GetCount(); i < count; i++)
			{

				cpContact con = contacts[i];
				float jnAcc = con.jnAcc;
				float jtAcc = con.jtAcc;

				sum += eCoef * jnAcc * jnAcc / con.nMass + jtAcc * jtAcc / con.tMass;
			}

			return sum;
		}



		/// Causes a collision pair to be ignored as if you returned false from a begin callback.
		/// If called from a pre-step callback, you will still need to return false
		/// if you want it to be ignored in the current step.
		public bool Ignore()
		{
			this.state = cpArbiterState.Ignore;
			return false;
		}

		public float GetRestitution()
		{
			return this.e;
		}

		public void SetRestitution(float restitution)
		{
			this.e = restitution;
		}

		public float GetFriction()
		{
			return this.u;
		}

		public void SetFriction(float friction)
		{
			this.u = friction;
		}


		public cpVect GetSurfaceVelocity()
		{
			return cpVect.cpvmult(this.surface_vr, this.swapped ? -1.0f : 1.0f);
		}


		public void SetSurfaceVelocity(cpVect vr)
		{
			this.surface_vr = cpVect.cpvmult(vr, this.swapped ? -1.0f : 1.0f);
		}

		public object GetUserData()
		{
			return this.data;
		}

		public void SetUserData(object userData)
		{
			this.data = userData;
		}


		/// Return the colliding shapes involved for this arbiter.
		/// The order of their cpSpace.collision_type values will match
		/// the order set when the collision handler was registered.
		public void GetShapes(out cpShape a, out cpShape b)
		{
			if (swapped)
			{
				a = this.b; b = this.a;
			}
			else
				a = this.a; b = this.b;
		}

		public void GetBodies(out cpBody a, out cpBody b)
		{
			cpShape shape_a, shape_b;
			GetShapes(out shape_a, out shape_b);
			a = shape_a.body;
			b = shape_b.body;
		}


		public bool CallWildcardBeginA(cpSpace space)
		{
			cpCollisionHandler handler = this.handlerA;
			return handler.beginFunc(this, space, handler.userData);
		}

		public bool CallWildcardBeginB(cpSpace space)
		{
			cpCollisionHandler handler = this.handlerB;
			this.swapped = !this.swapped;
			bool retval = handler.beginFunc(this, space, handler.userData);
			this.swapped = !this.swapped;
			return retval;
		}

		public bool CallWildcardPreSolveA(cpSpace space)
		{
			cpCollisionHandler handler = this.handlerA;
			return handler.preSolveFunc(this, space, handler.userData);
		}


		public bool CallWildcardPreSolveB(cpSpace space)
		{
			cpCollisionHandler handler = this.handlerB;
			this.swapped = !this.swapped;
			bool retval = handler.preSolveFunc(this, space, handler.userData);
			this.swapped = !this.swapped;
			return retval;
		}

		public void CallWildcardPostSolveA(cpSpace space)
		{
			cpCollisionHandler handler = this.handlerA;
			handler.postSolveFunc(this, space, handler.userData);
		}


		public void CallWildcardPostSolveB(cpSpace space)
		{
			cpCollisionHandler handler = this.handlerB;
			this.swapped = !this.swapped;
			handler.postSolveFunc(this, space, handler.userData);
			this.swapped = !this.swapped;

		}

		public void CallWildcardSeparateA(cpSpace space)
		{
			cpCollisionHandler handler = this.handlerA;
			handler.separateFunc(this, space, handler.userData);
		}


		public void CallWildcardSeparateB(cpSpace space)
		{
			cpCollisionHandler handler = this.handlerB;
			this.swapped = !this.swapped;
			handler.separateFunc(this, space, handler.userData);
			this.swapped = !this.swapped;

		}

		public void Update(cpCollisionInfo info, cpSpace space)
		{

			cpShape a = info.a, b = info.b;

			// For collisions between two similar primitive types, the order could have been swapped since the last frame.
			this.a = a; this.body_a = a.body;
			this.b = b; this.body_b = b.body;

			// Iterate over the possible pairs to look for hash value matches.
			for (int i = 0; i < info.count; i++)
			{
				cpContact con = info.arr[i];

				// r1 and r2 store absolute offsets at init time.
				// Need to convert them to relative offsets.
				con.r1 = cpVect.cpvsub(con.r1, a.body.p);
				con.r2 = cpVect.cpvsub(con.r2, b.body.p);

				// Cached impulses are not zeroed at init time.
				con.jnAcc = con.jtAcc = 0.0f;

				for (int j = 0; j < this.Count; j++)
				{
					cpContact old = this.contacts[j];

					// This could trigger false positives, but is fairly unlikely nor serious if it does.
					if (con.hash == old.hash)
					{
						// Copy the persistant contact information.
						con.jnAcc = old.jnAcc;
						con.jtAcc = old.jtAcc;
					}
				}
			}
			//TODO: revise
			this.contacts = info.arr.ToList();
			//this.count = info.count;
			this.n = info.n;

			this.e = a.e * b.e;
			this.u = a.u * b.u;

			cpVect surface_vr = cpVect.cpvsub(b.surfaceV, a.surfaceV);
			this.surface_vr = cpVect.cpvsub(surface_vr, cpVect.cpvmult(info.n, cpVect.cpvdot(surface_vr, info.n)));

			ulong typeA = info.a.type, typeB = info.b.type;
			cpCollisionHandler defaultHandler = space.defaultHandler;
			cpCollisionHandler handler = this.handler = space.LookupHandler(typeA, typeB, defaultHandler);

			// Check if the types match, but don't swap for a default handler which use the wildcard for type A.
			bool swapped = this.swapped = (typeA != handler.typeA && handler.typeA != cp.WILDCARD_COLLISION_TYPE);

			if (handler != defaultHandler || space.usesWildcards)
			{
				// The order of the main handler swaps the wildcard handlers too. Uffda.
				this.handlerA = space.LookupHandler(swapped ? typeB : typeA, cp.WILDCARD_COLLISION_TYPE, cpCollisionHandler.cpCollisionHandlerDoNothing);
				this.handlerB = space.LookupHandler(swapped ? typeA : typeB, cp.WILDCARD_COLLISION_TYPE, cpCollisionHandler.cpCollisionHandlerDoNothing);
			}

			// mark it as new if it's been cached
			if (this.state == cpArbiterState.Cached)
				this.state = cpArbiterState.FirstCollision;

		}

		public void PreStep(float dt, float slop, float bias)
		{
			cpBody a = this.body_a;
			cpBody b = this.body_b;
			cpVect n = this.n;
			cpVect body_delta = cpVect.cpvsub(b.p, a.p);

			for (var i = 0; i < Count; i++)
			{
				var con = this.contacts[i];

				// Calculate the mass normal and mass tangent.
				con.nMass = 1f / cp.k_scalar(a, b, con.r1, con.r2, n);
				con.tMass = 1f / cp.k_scalar(a, b, con.r1, con.r2, cpVect.cpvperp(n));

				// Calculate the target bias velocity.
				// Calculate the target bias velocity.
				float dist = cpVect.cpvdot(cpVect.cpvadd(cpVect.cpvsub(con.r2, con.r1), body_delta), n);
				con.bias = -bias * cp.cpfmin(0.0f, dist + slop) / dt;
				con.jBias = 0.0f;

				// Calculate the target bounce velocity.
				con.bounce = cp.normal_relative_velocity(a, b, con.r1, con.r2, n) * this.e;
			}
		}

		public static cpArbiterThread ThreadForBody(cpArbiter arb, cpBody body)
		{
			return (arb.body_a == body ? arb.thread_a : arb.thread_b);
		}

		public cpArbiterThread ThreadForBody(cpBody body)
		{
			//TODO: THIS NEEDS RETURN THE ORIGINAL MEMORY REFERENCE IN ARBITER
			if (this.body_a == body)
				return thread_a;
			else
				return thread_b;
		}


		// Equal function for arbiterSet.
		public static bool SetEql(cpShape[] shapes, cpArbiter arb)
		{
			cpShape a = shapes[0];
			cpShape b = shapes[1];
			return ((a == arb.a && b == arb.b) || (b == arb.a && a == arb.b));
		}


		public void ApplyCachedImpulse(float dt_coef)
		{
			if (this.IsFirstContact()) return;

			var a = this.body_a;
			var b = this.body_b;
			cpVect n = this.n;

			for (int i = 0; i < this.Count; i++)
			{
				cpContact con = this.contacts[i];
				cpVect j = cpVect.cpvrotate(n, new cpVect(con.jnAcc, con.jtAcc));
				cp.apply_impulses(a, b, con.r1, con.r2, cpVect.cpvmult(j, dt_coef));
			}
		}

		public cpArbiter Next(cpBody body)
		{
			return (this.body_a == body ? this.thread_a.next : this.thread_b.next);
		}


		public void ApplyImpulse(float dt)
		{
			cpBody a = this.body_a;
			cpBody b = this.body_b;
			cpVect n = this.n;
			cpVect surface_vr = this.surface_vr;
			float friction = this.u;

			for (int i = 0; i < this.Count; i++)
			{
				cpContact con = this.contacts[i];
				float nMass = con.nMass;
				cpVect r1 = con.r1;
				cpVect r2 = con.r2;

				cpVect vb1 = cpVect.cpvadd(a.v_bias, cpVect.cpvmult(cpVect.cpvperp(r1), a.w_bias));
				cpVect vb2 = cpVect.cpvadd(b.v_bias, cpVect.cpvmult(cpVect.cpvperp(r2), b.w_bias));
				cpVect vr = cpVect.cpvadd(cp.relative_velocity(a, b, r1, r2), surface_vr);

				float vbn = cpVect.cpvdot(cpVect.cpvsub(vb2, vb1), n);
				float vrn = cpVect.cpvdot(vr, n);
				float vrt = cpVect.cpvdot(vr, cpVect.cpvperp(n));

				float jbn = (con.bias - vbn) * nMass;
				float jbnOld = con.jBias;
				con.jBias = cp.cpfmax(jbnOld + jbn, 0.0f);

				float jn = -(con.bounce + vrn) * nMass;
				float jnOld = con.jnAcc;
				con.jnAcc = cp.cpfmax(jnOld + jn, 0.0f);

				float jtMax = friction * con.jnAcc;
				float jt = -vrt * con.tMass;
				float jtOld = con.jtAcc;
				con.jtAcc = cp.cpfclamp(jtOld + jt, -jtMax, jtMax);

				cp.apply_bias_impulses(a, b, r1, r2, cpVect.cpvmult(n, con.jBias - jbnOld));
				cp.apply_impulses(a, b, r1, r2, cpVect.cpvrotate(n, new cpVect(con.jnAcc - jnOld, con.jtAcc - jtOld)));


			};

		}
	}

	///////////////////////////////////////////////////


	public struct cpArbiterThread
	{
		public cpArbiter next, prev;

		public cpArbiterThread(cpArbiter next, cpArbiter prev)
		{
			this.next = next;
			this.prev = prev;
		}


	};

}


