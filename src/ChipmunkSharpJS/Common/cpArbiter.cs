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


	/// @private
	public class CollisionHandler
	{


		public string a;
		public string b;

		public Func<cpArbiter, cpSpace, bool> begin;
		public Func<cpArbiter, cpSpace, bool> preSolve;
		public Action<cpArbiter, cpSpace> postSolve;
		public Action<cpArbiter, cpSpace> separate;


		public CollisionHandler()
		{
			this.a = cp.CP_WILDCARD_COLLISION_TYPE;
			this.b = cp.CP_WILDCARD_COLLISION_TYPE;

			begin = DefaultBegin;
			preSolve = DefaultPreSolve;
			postSolve = DefaultPostSolve;
			separate = DefaultSeparate;
		}


		public CollisionHandler(string a, string b, Func<cpArbiter, cpSpace, bool> begin, Func<cpArbiter, cpSpace, bool> preSolve,
			Action<cpArbiter, cpSpace> postSolve, Action<cpArbiter, cpSpace> separate

			)
		{
			this.a = a;
			this.b = b;

			this.begin = begin;
			this.preSolve = preSolve;
			this.postSolve = postSolve;
			this.separate = separate;
		}


		public CollisionHandler Clone()
		{
			CollisionHandler copy = new CollisionHandler();
			copy.a = a;
			copy.b = b;

			copy.begin = begin;
			copy.preSolve = preSolve;
			copy.postSolve = postSolve;
			copy.separate = separate;
			return copy;
		}

		public bool DefaultBegin(cpArbiter arb, cpSpace space)
		{
			return true;
		}


		public bool DefaultPreSolve(cpArbiter arb, cpSpace space)
		{
			return true;
		}

		public void DefaultPostSolve(cpArbiter arb, cpSpace space)
		{
		}

		public void DefaultSeparate(cpArbiter arb, cpSpace space)
		{
		}



	};

	/// @private
	public enum cpArbiterState
	{
		// Arbiter is active and its the first collision.
		FirstColl = 1,
		// Arbiter is active and its not the first collision.
		Normal = 2,
		// Collision has been explicitly ignored.
		// Either by returning false from a begin collision handler or calling cpArbiterIgnore().
		Ignore = 3,
		// Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
		Cached = 4,
	} ;

	/// A colliding pair of shapes.
	public class cpArbiter
	{
		public string Key { get { return cp.hashPair(a.hashid, b.hashid); } }

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


		public List<ContactPoint> contacts { get; set; }

		public CollisionHandler handler { get; set; }

		public int stamp;

		public bool swappedColl { get; set; }

		public cpArbiter thread_a_next { get; set; }
		public cpArbiter thread_a_prev { get; set; }
		public cpArbiter thread_b_next { get; set; }
		public cpArbiter thread_b_prev { get; set; }


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


		// Arbiter states
		//
		// Arbiter is active and its the first collision.
		//	'first coll'
		// Arbiter is active and its not the first collision.
		//	'normal',
		// Collision has been explicitly ignored.
		// Either by returning false from a begin collision handler or calling cpArbiterIgnore().
		//	'ignore',
		// Collison is no longer active. A space will cache an arbiter for up to cpSpace.collisionPersistence more steps.
		//	'cached'

		/// A colliding pair of shapes.
		public cpArbiter(cpShape a, cpShape b)
		{
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

			this.thread_a_next = null;
			this.thread_b_next = null;
			this.thread_a_prev = null;
			this.thread_b_prev = null;

			this.contacts = null;

			this.stamp = 0;
			this.handler = null;
			this.swappedColl = false;
			this.state = cpArbiterState.FirstColl;
		}



		/// Return the colliding shapes involved for this arbiter.
		/// The order of their cpSpace.collision_type values will match
		/// the order set when the collision handler was registered.
		public cpShape[] GetShapes()
		{
			if (swappedColl)
				return new cpShape[] { b, a };
			else
				return new cpShape[] { a, b };
		}

		public void GetShapes(out cpShape a, out cpShape b)
		{
			if (swappedColl)
			{
				a = this.b; b = this.a;
			}
			else
				a = this.a; b = this.b;
		}

		/// Calculate the total impulse that was applied by this arbiter.
		/// This function should only be called from a post-solve, post-step or cpBodyEachArbiter callback.

		public cpVect TotalImpulse()
		{
			var sum = new cpVect(0, 0);

			for (int i = 0, count = contacts.Count; i < count; i++)
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
			var sum = cpVect.Zero;

			for (int i = 0, count = contacts.Count; i < count; i++)
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

				ContactPoint con = contacts[i];
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
			state = cpArbiterState.Ignore;
		}


		/// Returns true if this is the first step a pair of objects started colliding.
		public bool IsFirstContact()
		{
			return state == cpArbiterState.FirstColl;
		}


		/// Return a contact set from an arbiter.
		public List<ContactPoint> GetContactPointSet()
		{
			List<ContactPoint> set = new List<ContactPoint>();
			foreach (var item in contacts)
				set.Add(item.Clone());
			return set;
		}

		/// Get the position of the @c ith contact point.
		public cpVect GetPoint(int i)
		{
			cp.assertHard(0 <= i && i < contacts.Count, "Index error: The specified contact index is invalid for this arbiter");
			return contacts[i].p;
			// return contacts[i].point;
		}

		/// Get the normal of the @c ith contact point.
		public cpVect GetNormal(int i)
		{
			cp.assertHard(0 <= i && i < contacts.Count, "Index error: The specified contact index is invalid for this arbiter");

			var n = this.contacts[i].n;
			return this.swappedColl ? n.Neg() : n;
		}


		/// Get the depth of the @c ith contact point.
		public float GetDepth(int i)
		{
			// return this.contacts[i].dist;
			cp.assertHard(0 <= i && i < contacts.Count, "Index error: The specified contact index is invalid for this arbiter");
			return contacts[i].dist;

		}


		public void Unthread()
		{
			cp.unthreadHelper(this, this.body_a, this.thread_a_prev, this.thread_a_next);
			cp.unthreadHelper(this, this.body_b, this.thread_b_prev, this.thread_b_next);
			this.thread_a_prev = null;
			this.thread_a_next = null;
			this.thread_b_prev = null;
			this.thread_b_next = null;
		}


		public void Update(List<ContactPoint> contacts, CollisionHandler handler, cpShape a, cpShape b)
		{
			//throw new NotImplementedException();

			if (this.contacts != null)
			{

				// Iterate over the possible pairs to look for hash value matches.
				for (int i = 0; i < this.contacts.Count; i++)
				{
					ContactPoint old = this.contacts[i];

					for (int j = 0; j < contacts.Count; j++)
					{
						// ContactPoint new_contact = contacts[j];

						// This could trigger false positives, but is fairly unlikely nor serious if it does.
						if (contacts[j].hash == old.hash)
						{
							// Copy the persistant contact information.
							contacts[j].jnAcc = old.jnAcc;
							contacts[j].jtAcc = old.jtAcc;
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
			if (this.state == cpArbiterState.Cached) this.state = cpArbiterState.FirstColl;

		}

		public void PreStep(float dt, float slop, float bias)
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
				con.nMass = 1 / cp.k_scalar(a, b, con.r1, con.r2, con.n);
				con.tMass = 1 / cp.k_scalar(a, b, con.r1, con.r2, cpVect.cpvperp(con.n));

				// Calculate the target bias velocity.
				con.bias = -bias * Math.Min(0, con.dist + slop) / dt;
				con.jBias = 0;

				// Calculate the target bounce velocity.
				con.bounce = cp.normal_relative_velocity(a, b, con.r1, con.r2, con.n) * this.e;
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
				var nx = con.n.x;
				var ny = con.n.y;
				var jx = nx * con.jnAcc - ny * con.jtAcc;
				var jy = nx * con.jtAcc + ny * con.jnAcc;
				cp.apply_impulses(a, b, con.r1, con.r2, jx * dt_coef, jy * dt_coef);
			}
		}

		public void ApplyImpulse(float dt)
		{

			cp.numApplyImpulse++;
			//if (!this.contacts) { throw new Error('contacts is undefined'); }
			var a = this.body_a;
			var b = this.body_b;
			var surface_vr = this.surface_vr;
			var friction = this.u;

			for (var i = 0; i < this.contacts.Count; i++)
			{
				cp.numApplyContact++;
				var con = this.contacts[i];
				var nMass = con.nMass;
				var n = con.n;
				var r1 = con.r1;
				var r2 = con.r2;

				//var vr = relative_velocity(a, b, r1, r2);
				var vrx = b.v.x - r2.y * b.w - (a.v.x - r1.y * a.w);
				var vry = b.v.y + r2.x * b.w - (a.v.y + r1.x * a.w);

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
				con.jtAcc = cp.cpclamp(jtOld + jt, -jtMax, jtMax);

				//apply_bias_impulses(a, b, r1, r2, vmult(n, con.jBias - jbnOld));
				var bias_x = n.x * (con.jBias - jbnOld);
				var bias_y = n.y * (con.jBias - jbnOld);
				cp.apply_bias_impulse(a, -bias_x, -bias_y, r1);
				cp.apply_bias_impulse(b, bias_x, bias_y, r2);

				//apply_impulses(a, b, r1, r2, vrotate(n, new Vect(con.jnAcc - jnOld, con.jtAcc - jtOld)));
				var rot_x = con.jnAcc - jnOld;
				var rot_y = con.jtAcc - jtOld;

				// Inlining apply_impulses decreases speed for some reason :/
				cp.apply_impulses(a, b, r1, r2, n.x * rot_x - n.y * rot_y, n.x * rot_y + n.y * rot_x);
			}
		}


		public void CallSeparate(cpSpace space)
		{
			// The handler needs to be looked up again as the handler cached on the arbiter may have been deleted since the last step.
			var handler = space.lookupHandler(this.a.collision_type, this.b.collision_type);
			handler.separate(this, space);
		}

		public cpArbiter Next(cpBody body)
		{
			return (this.body_a == body ? this.thread_a_next : this.thread_b_next);
		}



	};



}


