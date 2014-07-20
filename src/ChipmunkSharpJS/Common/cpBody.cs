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

namespace ChipmunkSharp
{


	///// @defgroup cpBody cpBody
	///// Chipmunk's rigid body type. Rigid bodies hold the physical properties of an object like
	///// it's mass, and position and velocity of it's center of gravity. They don't have an shape on their own.
	///// They are given a shape by creating collision shapes (cpShape) that point to the body.
	///// @{

	///// Rigid body velocity update function type.
	//public delegate void cpBodyVelocityFunc(cpVect gravity, float damping, float dt);
	///// Rigid body position update function type.
	//public delegate void cpBodyPositionFunc(cpBody body, float dt);


	/// Used internally to track information on the collision graph.
	/// @private
	public struct cpComponentNode
	{
		public cpBody root;
		public cpBody next;
		public float idleTime;

		public cpComponentNode(cpBody root, cpBody next, float idleTime)
		{
			this.root = root;
			this.next = next;
			this.idleTime = idleTime;
		}

	} ;


	/// Chipmunk's rigid body struct.
	public class cpBody
	{

		#region STATIC VARS

		//public static void cpv_assert_nan(cpVect v, char* message) { cpAssertSoft(v.x == v.x && v.y == v.y, message); }
		//public static void cpv_assert_infinite(cpVect v, char* message) { cpAssertSoft(cpfabs(v.x) != INFINITY && cpfabs(v.y) != INFINITY, message); }
		//public static void cpv_assert_sane(cpVect v, string* message) { cpv_assert_nan(v, message); cpv_assert_infinite(v, message); }

		public cpVect Rotation { get { return rot; } }
		public float AngVel { get { return w; } set { w = value; } }
		public float Torque { get { return t; } set { t = value; } }
		public float VelLimit { get { return v_limit; } set { v_limit = value; } }
		public float AngVelLimit { get { return w_limit; } set { w_limit = value; } }
		public object UserData { get { return data; } set { data = value; } }

		public cpVect Vel { get { return v; } set { v = value; } }
		public cpVect Force { get { return f; } set { f = value; } }

		public float Angle { get { return a; } }

		public cpVect Position { get { return p; } set { SetPosition(value); } }

		public float Moment { get { return i; } set { SetMoment(value); } }

		public float Mass { get { return m; } set { SetMass(value); } }



		//public float Position { get { return p; } }

		#endregion

		#region PROPS

		/// Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity)
		public Action<cpVect, float, float> velocity_func;

		/// Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition)
		public Action<float> position_func;

		/// Mass of the body.
		/// Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason.
		private float m;
		/// Mass inverse.
		public float m_inv;


		/// Moment of inertia of the body.
		/// Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for this reason.
		public float i;
		/// Moment of inertia inverse.
		public float i_inv;

		/// Position of the rigid body's center of gravity.
		private cpVect p;
		/// Velocity of the rigid body's center of gravity.
		public cpVect v;
		/// Force acting on the rigid body's center of gravity.
		public cpVect f;

		/// Rotation of the body around it's center of gravity in radians.
		/// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
		public float a;
		/// Angular velocity of the body around it's center of gravity in radians/second.
		public float w;
		/// Torque applied to the body around it's center of gravity.
		public float t;

		/// Cached unit length vector representing the angle of the body.
		/// Used for fast rotations using cpvrotate().
		private cpVect rot;

		/// User definable data pointer.
		/// Generally this points to your the game object class so you can access it
		/// when given a cpBody reference in a callback.
		public object data;

		/// Maximum velocity allowed when updating the velocity.
		float v_limit;
		/// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
		float w_limit;

		public cpVect v_bias;
		public float w_bias;

		public cpSpace space;

		public List<cpShape> shapeList;
		public cpArbiter arbiterList;
		public cpConstraint constraintList;

		//public cpComponentNode node;

		#endregion

		#region CONSTRUCTORS

		public cpBody nodeRoot { get; set; }
		public cpBody nodeNext { get; set; }

		public float nodeIdleTime { get; set; }

		/// <summary>
		/// CREATES A STATIC BODY
		/// </summary>
		public cpBody()
			: this(cp.Infinity, cp.Infinity)
		{
			nodeIdleTime = cp.Infinity;
		}

		/// <summary>
		/// CREATES A BODY WITH MASS AND INERTIA
		/// </summary>
		/// <param name="m"></param>
		/// <param name="i"></param>
		public cpBody(float m, float i)
		{


			velocity_func = VelocityFunc;
			position_func = PositionFunc;

			/// Mass of the body.
			/// Must agree with cpBody.m_inv! Use body.setMass() when changing the mass for this reason.
			//this.m;
			/// Mass inverse.
			//this.m_inv;

			/// Moment of inertia of the body.
			/// Must agree with cpBody.i_inv! Use body.setMoment() when changing the moment for this reason.
			//this.i;
			/// Moment of inertia inverse.
			//this.i_inv;

			/// Position of the rigid body's center of gravity.
			this.p = cpVect.Zero;
			/// Velocity of the rigid body's center of gravity.
			this.v = cpVect.Zero;
			/// Force acting on the rigid body's center of gravity.
			this.f = cpVect.Zero;

			/// Rotation of the body around it's center of gravity in radians.
			/// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
			//this.a;
			/// Angular velocity of the body around it's center of gravity in radians/second.
			this.w = 0;
			/// Torque applied to the body around it's center of gravity.
			this.t = 0;

			/// Cached unit length vector representing the angle of the body.
			/// Used for fast rotations using cpvrotate().
			//cpVect rot;

			/// Maximum velocity allowed when updating the velocity.
			this.v_limit = cp.Infinity;
			/// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
			this.w_limit = cp.Infinity;

			// This stuff is all private.
			this.v_bias = cpVect.Zero; //x = this.v_biasy = 0;
			this.w_bias = 0;

			this.space = null;

			this.shapeList = new List<cpShape>();

			this.arbiterList = null; // These are both wacky linked lists.
			this.constraintList = null;

			// This stuff is used to track information on the collision graph.
			this.nodeRoot = null;

			this.nodeNext = null;
			this.nodeIdleTime = 0;

			// Set this.m and this.m_inv
			this.SetMass(m);

			// Set this.i and this.i_inv
			this.SetMoment(i);

			// Set this.a and this.rot
			this.rot = cpVect.Zero;
			this.SetAngle(0);


		}

		#endregion

		#region PUBLIC METHODS

		public void SanityCheck()
		{
			//cp.v_assert_sane(this.p, "Body's position is invalid.");
			//cp.v_assert_sane(this.f, "Body's force is invalid.");
			//cp.assert(this.vx == this.vx && Math.abs(this.vx) != Infinity, "Body's velocity is invalid.");
			//cp.assert(this.vy == this.vy && Math.abs(this.vy) != Infinity, "Body's velocity is invalid.");
			//cp.assert(this.a == this.a && Math.abs(this.a) != Infinity, "Body's angle is invalid.");
			//cp.assert(this.w == this.w && Math.abs(this.w) != Infinity, "Body's angular velocity is invalid.");
			//cp.assert(this.t == this.t && Math.abs(this.t) != Infinity, "Body's torque is invalid.");
			//cp.v_assert_sane(this.rot, "Body's rotation vector is invalid.");

		}

		public cpVect GetPos() { return this.p; }
		public cpVect GetVel() { return v; }
		public float GetAngVel() { return this.w; }


		/// Returns true if the body is sleeping.
		public bool IsSleeping()
		{
			return this.nodeRoot != null;
		}

		/// Returns true if the body is static.
		public bool IsStatic()
		{
			return nodeIdleTime == cp.Infinity;
		}

		/// Returns true if the body has not been added to a space.
		/// Note: Static bodies are a subtype of rogue bodies.
		public bool IsRogue()
		{
			return space == null;  //(cpSpace)0));
		}

		/// Set the mass of a body.
		public void SetMass(float mass)
		{
			cp.assertHard(mass > 0.0f, "Mass must be positive and non-zero.");

			Activate();
			m = mass;
			m_inv = 1.0f / mass;
		}

		//CP_DefineBodyStructGetter(float, i, Moment)
		/// Set the moment of a body.
		public void SetMoment(float moment)
		{
			cp.assertHard(moment > 0.0f, "Moment of Inertia must be positive and non-zero.");

			Activate();
			i = moment;
			i_inv = 1.0f / moment;
		}

		public void AddShape(cpShape shape)
		{
			this.shapeList.Add(shape);
		}

		public void RemoveShape(cpShape shape)
		{
			// This implementation has a linear time complexity with the number of shapes.
			// The original implementation used linked lists instead, which might be faster if
			// you're constantly editing the shape of a body. I expect most bodies will never
			// have their shape edited, so I'm just going to use the simplest possible implemention.
			shapeList.Remove(shape);
		}


		public void RemoveConstraint(cpConstraint constraint)
		{
			constraintList = cp.filterConstraints(constraintList, this, constraint);
		}


		/// Set the position of a body.
		public void SetPosition(cpVect pos)
		{
			this.Activate();
			this.SanityCheck();
			// If I allow the position to be set to vzero, vzero will get changed.
			//if (pos == cpVect.ZERO) {
			//    pos = cp.v(0,0);
			//}
			this.p = pos;
		}

		public void SetAngle(float angle)
		{
			this.Activate();
			this.SanityCheck();
			this.SetAngleInternal(angle);
		}

		public void SetVelocity(cpVect velocity)
		{
			this.Activate();
			this.v.x = velocity.x;
			this.v.y = velocity.y;
		}


		public void SetAngularVelocity(float w)
		{
			this.Activate();
			this.w = w;
		}


		public void SetAngleInternal(float angle)
		{
			cp.assertHard(!float.IsNaN(angle), "Internal Error: Attempting to set body's angle to NaN");
			this.a = angle;//fmod(a, (cpFloat)M_PI*2.0f);

			//this.rot = vforangle(angle);
			this.rot.x = cp.cpfcos(angle);
			this.rot.y = cp.cpfsin(angle);
		}


		///// Rigid body velocity update function type.
		//public delegate void cpBodyVelocityFunc(cpVect gravity, float damping, float dt);
		///// Rigid body position update function type.
		//public delegate void cpBodyPositionFunc(cpBody body, float dt);


		public void VelocityFunc(cpVect gravity, float damping, float dt)
		{
			//this.v = vclamp(vadd(vmult(this.v, damping), vmult(vadd(gravity, vmult(this.f, this.m_inv)), dt)), this.v_limit);
			var vx = this.v.x * damping + (gravity.x + this.f.x * this.m_inv) * dt;
			var vy = this.v.y * damping + (gravity.y + this.f.y * this.m_inv) * dt;

			//var v = vclamp(new Vect(vx, vy), this.v_limit);
			//this.vx = v.x; this.vy = v.y;
			var v_limit = this.v_limit;
			var lensq = vx * vx + vy * vy;
			var scale = (lensq > v_limit * v_limit) ? v_limit / cp.cpfsqrt(lensq) : 1;
			this.v.x = vx * scale;
			this.v.y = vy * scale;

			var w_limit = this.w_limit;
			this.w = cp.cpfclamp(this.w * damping + this.t * this.i_inv * dt, -w_limit, w_limit);

			this.SanityCheck();
		}


		public void PositionFunc(float dt)
		{
			//this.p = vadd(this.p, vmult(vadd(this.v, this.v_bias), dt));

			//this.p = this.p + (this.v + this.v_bias) * dt;
			this.p.x += (this.v.x + this.v_bias.x) * dt;
			this.p.y += (this.v.y + this.v_bias.y) * dt;

			this.SetAngleInternal(this.a + (this.w + this.w_bias) * dt);

			this.v_bias.x = this.v_bias.y = 0;
			this.w_bias = 0;

			this.SanityCheck();
		}

		/// Set the forces and torque or a body to zero.
		public void ResetForces()
		{
			this.Activate();
			f = cpVect.Zero;
			t = 0.0f;
		}

		/// Apply an force (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
		public void ApplyForce(cpVect force, cpVect r)
		{
			this.Activate();
			f = cpVect.cpvadd(force, force);
			t += cpVect.cpvcross(r, force);
		}

		/// Apply an impulse (in world coordinates) to the body at a point relative to the center of gravity (also in world coordinates).
		public void ApplyImpulse(cpVect j, cpVect r)
		{
			this.Activate();
			cp.apply_impulse(this, j, r);
		}

		public cpVect GetVelAtPoint(cpVect r)
		{
			return cpVect.cpvadd(v, cpVect.cpvmult(cpVect.cpvperp(r), w));
		}

		/// Get the velocity on a body (in world units) at a point on the body in world coordinates.
		public cpVect GetVelAtWorldPoint(cpVect point)
		{
			return GetVelAtPoint(cpVect.cpvsub(point, p));
		}


		/// Get the velocity on a body (in world units) at a point on the body in local coordinates.
		public cpVect GetVelAtLocalPoint(cpVect point)
		{
			return GetVelAtPoint(cpVect.cpvrotate(point, rot));
		}


		//public delegate void cpBodyComponentIteratorFunc(cpBody body, cpBody component, object data);


		///// Body/shape iterator callback function type. 
		//public delegate void cpBodyShapeIteratorFunc(cpBody body, cpShape shape, object data);


		public void EachShape(Action<cpShape> func)
		{
			for (int i = 0, len = this.shapeList.Count; i < len; i++)
			{
				func(this.shapeList[i]);
			}
		}

		///// Body/raint iterator callback function type. 
		//public delegate void cpBodyConstraintIteratorFunc(cpBody body, cpConstraint raint, object data);


		public void EachConstraint(Action<cpConstraint> func)
		{
			var constraint = this.constraintList;
			while (constraint != null)
			{
				var next = constraint.Next(this);
				func(constraint);
				constraint = next;
			}
		}


		/// Body/arbiter iterator callback function type. 
		//public delegate void cpBodyArbiterIteratorFunc(cpBody body, cpArbiter arbiter, object data);

		public void EachArbiter(Action<cpArbiter> func)
		{
			var arb = this.arbiterList;
			while (arb != null)
			{
				var next = arb.Next(this);

				arb.swappedColl = (this == arb.body_b);
				func(arb);

				arb = next;
			}
		}


		/// Convert body relative/local coordinates to absolute/world coordinates.
		public cpVect Local2World(cpVect v)
		{
			return cpVect.cpvadd(p, cpVect.cpvrotate(v, rot));
		}

		/// Convert body absolute/world coordinates to  relative/local coordinates.
		public cpVect World2Local(cpVect v)
		{
			return cpVect.cpvunrotate(cpVect.cpvsub(v, p), rot);
		}

		//        /// Get the kinetic energy of a body.
		public float KineticEnergy()
		{
			// Need to do some fudging to avoid NaNs
			var vsq = this.v.x * this.v.x + this.v.y * this.v.y;
			var wsq = this.w * this.w;
			return (vsq > 0 ? vsq * this.m : 0) + (wsq > 0 ? wsq * this.i : 0);
		}


		//        // Defined in cpSpace.c
		//        /// Wake up a sleeping or idle body.
		public void Activate()
		{
			if (!this.IsRogue())
			{
				this.nodeIdleTime = 0;
				cp.componentActivate(cp.componentRoot(this));
			}

		}
		//        /// Wake up any sleeping or idle bodies touching a static body.
		public void ActivateStatic(cpShape filter)
		{
			cp.assertHard(this.IsStatic(), "Body.activateStatic() called on a non-static body.");

			for (var arb = this.arbiterList; arb != null; arb = arb.Next(this))
			{
				if (filter == null || filter == arb.a || filter == arb.b)
				{
					(arb.body_a == this ? arb.body_b : arb.body_a).Activate();
				}
			}

			// TODO should also activate joints!
		}




		public void PushArbiter(cpArbiter arb)
		{

			cp.assertSoft((arb.body_a == this ? arb.thread_a_next : arb.thread_b_next) == null,
			"Internal Error: Dangling contact graph pointers detected. (A)");
			cp.assertSoft((arb.body_a == this ? arb.thread_a_prev : arb.thread_b_prev) == null,
				"Internal Error: Dangling contact graph pointers detected. (B)");

			var next = this.arbiterList;

			cp.assertSoft(next == null || (next.body_a == this ? next.thread_a_prev : next.thread_b_prev) == null,
				"Internal Error: Dangling contact graph pointers detected. (C)");

			if (arb.body_a == this)
			{
				arb.thread_a_next = next;
			}
			else
			{
				arb.thread_b_next = next;
			}

			if (next != null)
			{
				if (next.body_a == this)
				{
					next.thread_a_prev = arb;
				}
				else
				{
					next.thread_b_prev = arb;
				}
			}
			this.arbiterList = arb;
		}



		//        /// Force a body to fall asleep immediately.
		public void Sleep()
		{
			this.SleepWithGroup(null);
		}


		//        /// Force a body to fall asleep immediately along with other bodies in a group.
		public void SleepWithGroup(cpBody group)
		{
			cp.assertSoft(!this.IsStatic() && !this.IsRogue(), "Rogue and static bodies cannot be put to sleep.");

			var space = this.space;
			cp.assertSoft(space != null, "Cannot put a rogue body to sleep.");
			cp.assertSoft(space.isLocked, "Bodies cannot be put to sleep during a query or a call to cpSpaceStep(). Put these calls into a post-step callback.");
			cp.assertSoft(group == null || group.IsSleeping(), "Cannot use a non-sleeping body as a group identifier.");

			if (this.IsSleeping())
			{
				cp.assertSoft(cp.componentRoot(this) == cp.componentRoot(group), "The body is already sleeping and it's group cannot be reassigned.");
				return;
			}

			for (var i = 0; i < this.shapeList.Count; i++)
			{
				this.shapeList[i].Update(this.p, this.rot);
			}
			space.deactivateBody(this);

			if (group != null)
			{
				var root = cp.componentRoot(group);

				this.nodeRoot = root;
				this.nodeNext = root.nodeNext;
				this.nodeIdleTime = 0;

				root.nodeNext = this;
			}
			else
			{
				this.nodeRoot = this;
				this.nodeNext = null;
				this.nodeIdleTime = 0;

				space.sleepingComponents.Add(this);
			}
			space.bodies.Remove(this);

		}


	}
		#endregion


}
