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

	public enum cpBodyType
	{
		DYNAMIC,
		KINEMATIC,
		STATIC,
	}

	/// Chipmunk's rigid body struct.
	public class cpBody
	{

		#region PROPS

		/// Function that is called to integrate the body's velocity. (Defaults to cpBodyUpdateVelocity)
		internal Action<cpVect, float, float> velocity_func;

		/// Function that is called to integrate the body's position. (Defaults to cpBodyUpdatePosition)
		internal Action<float> position_func;

		/// Mass of the body.
		/// Must agree with cpBody.m_inv! Use cpBodySetMass() when changing the mass for this reason.
		internal float m;
		/// Mass inverse.
		internal float m_inv;

		/// Moment of inertia of the body.
		/// Must agree with cpBody.i_inv! Use cpBodySetMoment() when changing the moment for this reason.
		internal float i;
		/// Moment of inertia inverse.
		internal float i_inv;

		/// Cached unit length vector representing the angle of the body.
		/// Used for fast rotations using cpvrotate().
		public cpVect cog;


		/// Position of the rigid body's center of gravity.
		public cpVect p;
		/// Velocity of the rigid body's center of gravity.
		public cpVect v;
		/// Force acting on the rigid body's center of gravity.
		internal cpVect f;

		/// Rotation of the body around it's center of gravity in radians.
		/// Must agree with cpBody.rot! Use cpBodySetAngle() when changing the angle for this reason.
		internal float a;
		/// Angular velocity of the body around it's center of gravity in radians/second.
		public float w;
		/// Torque applied to the body around it's center of gravity.
		internal float t;


		public cpTransform transform;

		/// User definable data pointer.
		/// Generally this points to your the game object class so you can access it
		/// when given a cpBody reference in a callback.
		internal object userData;

		/// Maximum velocity allowed when updating the velocity.
		internal float v_limit;
		/// Maximum rotational rate (in radians/second) allowed when updating the angular velocity.
		internal float w_limit;

		public cpVect v_bias;
		public float w_bias;

		public cpSpace space;

		public cpShape shapeList;
		public cpArbiter arbiterList;
		public cpConstraint constraintList;


		public cpBody nodeRoot { get; set; }
		public cpBody nodeNext { get; set; }
		public float nodeIdleTime { get; set; }

		#endregion

		public cpBodyType bodyType
		{
			get
			{

				if (this.nodeIdleTime == cp.Infinity)
				{
					return cpBodyType.STATIC;
				}
				else if (this.m == cp.Infinity)
				{
					return cpBodyType.KINEMATIC;
				}
				else
				{
					return cpBodyType.DYNAMIC;
				}

			}
			set
			{
				SetBodyType(value);

			}
		}

		#region CONSTRUCTORS

		/// <summary>
		/// CREATES A BODY WITH MASS AND INERTIA
		/// </summary>
		/// <param name="mass"></param>
		/// <param name="moment"></param>
		public cpBody(float mass, float moment)
		{

			transform = new cpTransform();

			this.cog = cpVect.Zero;
			this.space = null;

			this.shapeList = null;
			this.arbiterList = null; // These are both wacky linked lists.
			this.constraintList = null;

			velocity_func = UpdateVelocity;
			position_func = UpdatePosition;

			// This stuff is used to track information on the collision graph.
			this.nodeRoot = null;
			this.nodeNext = null;
			this.nodeIdleTime = 0;

			/// Position of the rigid body's center of gravity.
			this.p = cpVect.Zero;
			/// Velocity of the rigid body's center of gravity.
			this.v = cpVect.Zero;
			/// Force acting on the rigid body's center of gravity.
			this.f = cpVect.Zero;


			/// Angular velocity of the body around it's center of gravity in radians/second.
			this.w = 0;
			/// Torque applied to the body around it's center of gravity.
			this.t = 0;

			// This stuff is all private.
			this.v_bias = cpVect.Zero; //x = this.v_biasy = 0;
			this.w_bias = 0;

			this.userData = null;

			this.SetMass(mass);
			this.SetMoment(moment);
			this.SetAngle(0.0f);

		}

		#endregion

		#region PUBLIC METHODS

		/// Returns true if the body is sleeping.
		public bool IsSleeping()
		{
			return this.nodeRoot != null;
		}

		public void SetBodyType(cpBodyType type)
		{
			cpBodyType oldType = bodyType;

			if (oldType == type) return;

			// Static bodies have their idle timers set to infinity.
			// Non-static bodies should have their idle timer reset.
			nodeIdleTime = (type == cpBodyType.STATIC ? cp.Infinity : 0.0f);


			if (type == cpBodyType.DYNAMIC)
			{
				this.m = this.i = 0.0f;
				this.m_inv = this.i_inv = cp.Infinity;

				AccumulateMassFromShapes();

			}
			else
			{
				this.m = this.i = cp.Infinity;
				this.m_inv = this.i_inv = 0.0f;

				this.v = cpVect.Zero;
				this.w = 0.0f;
			}

			// If the body is added to a space already, we'll need to update some space data structures.

			if (space != null)
			{

				cp.AssertSpaceUnlocked(space);


				if (oldType == cpBodyType.STATIC)
				{
					// TODO This is probably not necessary
					//			cpBodyActivateStatic(body, NULL);
				}
				else
				{
					Activate();
				}

				// Move the bodies to the correct array.
				List<cpBody> fromArray = space.ArrayForBodyType(oldType);
				List<cpBody> toArray = space.ArrayForBodyType(type);

				if (fromArray != toArray)
				{
					fromArray.Remove(this);
					toArray.Add(this);
				}

				// Move the body's shapes to the correct spatial index.
				cpBBTree fromIndex = (oldType == cpBodyType.STATIC ? space.staticShapes : space.dynamicShapes);
				cpBBTree toIndex = (type == cpBodyType.STATIC ? space.staticShapes : space.dynamicShapes);

				if (fromIndex != toIndex)
				{
					eachShape((s, o) =>
					{
						fromIndex.Remove(s.hashid);
						toIndex.Insert(s.hashid, s);

					}, null);

				}
			}

		}


		// Should *only* be called when shapes with mass info are modified, added or removed.
		public void AccumulateMassFromShapes()
		{
			if (bodyType != cpBodyType.DYNAMIC) return;

			// Reset the body's mass data.
			this.m = this.i = 0.0f;

			this.cog = cpVect.Zero;

			// Cache the position to realign it at the end.
			cpVect pos = GetPosition();

			// Accumulate mass from shapes.

			eachShape((shape, o) =>
			{
				cpShapeMassInfo info = shape.massInfo;
				float m = info.m;

				if (m > 0.0f)
				{
					float msum = this.m + m;

					this.i += m * info.i + cpVect.cpvdistsq(this.cog, info.cog) * (m * this.m) / msum;
					this.cog = cpVect.cpvlerp(this.cog, info.cog, m / msum);
					this.m = msum;
				}
			}, null);


			// Recalculate the inverses.
			this.m_inv = 1.0f / this.m;
			this.i_inv = 1.0f / this.i;

			// Realign the body since the CoG has probably moved.
			SetPosition(pos);

			AssertSaneBody();

		}

		public cpSpace GetSpace()
		{
			return this.space;
		}

		public float GetMass()
		{
			return this.m;
		}

		/// Set the mass of a body.
		public void SetMass(float mass)
		{
			cp.AssertHard(this.bodyType == cpBodyType.DYNAMIC, "You cannot set the mass of kinematic or static bodies.");
			cp.AssertHard(0.0f <= mass && mass < cp.Infinity, "Mass must be positive and finite.");

			Activate();
			m = mass;
			m_inv = 1.0f / mass;
			AssertSaneBody();
		}


		public float GetMoment()
		{
			return this.i;
		}

		/// Set the moment of a body.
		public void SetMoment(float moment)
		{
			cp.AssertHard(moment >= 0.0f, "Moment of Inertia must be positive and non-zero.");

			Activate();
			this.i = moment;
			this.i_inv = 1.0f / moment;
			AssertSaneBody();
		}

		public cpVect GetRotation()
		{
			return new cpVect(transform.a, transform.b);
		}

		public void AddShape(cpShape shape)
		{
			//this.shapeList.Add(shape);

			cpShape next = this.shapeList;
			if (next != null)
				next.prev = shape;

			shape.next = next;
			this.shapeList = shape;

			if (shape.massInfo.m > 0.0f)
			{
				AccumulateMassFromShapes();
			}
		}

		public void RemoveShape(cpShape shape)
		{
			cpShape prev = shape.prev;
			cpShape next = shape.next;

			if (prev != null)
			{
				prev.next = next;
			}
			else
			{
				this.shapeList = next;
			}

			if (next != null)
			{
				next.prev = prev;
			}

			shape.prev = null;
			shape.next = null;

			if (bodyType == cpBodyType.DYNAMIC && shape.massInfo.m > 0.0f)
			{
				AccumulateMassFromShapes();
			}

		}

		public void RemoveConstraint(cpConstraint constraint)
		{
			this.constraintList = cp.filterConstraints(constraintList, this, constraint);
		}


		// 'p' is the position of the CoG
		public void SetTransform(cpVect p, float a)
		{
			cpVect rot = cpVect.cpvforangle(a);
			cpVect c = this.cog;

			this.transform = cpTransform.NewTranspose(
				rot.x, -rot.y, p.x - (c.x * rot.x - c.y * rot.y),
				rot.y, rot.x, p.y - (c.x * rot.y + c.y * rot.x)
			);
		}

		public static float SetAngle(cpBody body, float angle)
		{
			body.a = angle;
			body.AssertSaneBody();
			return angle;
		}

		public cpVect GetPosition()
		{
			return cpTransform.Point(transform, cpVect.Zero);
		}

		/// Set the position of a body.
		public void SetPosition(cpVect position)
		{
			this.Activate();

			cpVect p = this.p = cpVect.cpvadd(
				cpTransform.Vect(this.transform, this.cog)
				, position);

			AssertSaneBody();

			SetTransform(p, this.a);
		}

		public cpVect GetCenterOfGravity()
		{
			return cog;
		}

		public void SetCenterOfGravity(cpVect cog)
		{
			Activate();
			this.cog = cog;
			AssertSaneBody();
		}

		public cpVect GetVelocity()
		{
			return v;
		}

		public void SetVelocity(cpVect velocity)
		{
			this.Activate();
			this.v = velocity;
			AssertSaneBody();
		}

		public cpVect GetForce()
		{
			return this.f;
		}

		public void SetForce(cpVect force)
		{
			Activate();
			this.f = force;
			AssertSaneBody();
		}

		public float GetAngle()
		{
			return this.a;
		}

		public void SetAngle(float angle)
		{
			Activate();
			SetAngle(this, angle);
			SetTransform(this.p, angle);
		}

		public float GetAngularVelocity()
		{
			return this.w;
		}
		public void SetAngularVelocity(float angularVelocity)
		{
			this.Activate();
			this.w = angularVelocity;
			AssertSaneBody();
		}

		public float GetTorque()
		{
			return this.t;
		}

		public void SetTorque(float torque)
		{
			Activate();
			this.t = torque;
			AssertSaneBody();
		}

		public object GetUserData()
		{
			return this.userData;
		}

		public void SetUserData(object userData)
		{
			this.userData = userData;
		}


		public void SetVelocityUpdateFunc(Action<cpVect, float, float> velocityFunc)
		{
			this.velocity_func = velocityFunc;
		}

		public void SetPositionUpdateFunc(Action<float> positionFunc)
		{
			this.position_func = positionFunc;
		}

		public void UpdateVelocity(cpVect gravity, float damping, float dt)
		{
			// Skip kinematic bodies.
			if (bodyType == cpBodyType.KINEMATIC) return;

			cp.AssertSoft(this.m > 0.0f && this.i > 0.0f, string.Format("Body's mass and moment must be positive to simulate. (Mass: {0} Moment: {1})", this.m, this.i));

			this.v = cpVect.cpvadd(cpVect.cpvmult(this.v, damping), cpVect.cpvmult(cpVect.cpvadd(gravity, cpVect.cpvmult(this.f, this.m_inv)), dt));
			this.w = this.w * damping + this.t * this.i_inv * dt;

			// Reset forces.
			this.f = cpVect.Zero;
			this.t = 0.0f;

			AssertSaneBody();
		}

		public void UpdatePosition(float dt)
		{
			cpVect p = this.p = cpVect.cpvadd(
				this.p,
				cpVect.cpvmult(cpVect.cpvadd(this.v, this.v_bias), dt)
				);

			float a = SetAngle(this, this.a + (this.w + this.w_bias) * dt);

			SetTransform(p, a);

			this.v_bias = cpVect.Zero;
			this.w_bias = 0.0f;

			AssertSaneBody();
		}

		// Convert body relative/local coordinates to absolute/world coordinates.
		public cpVect LocalToWorld(cpVect point)
		{
			return cpTransform.Point(this.transform, point);
		}

		public cpVect WorldToLocal(cpVect point)
		{
			return cpTransform.Point(
				cpTransform.RigidInverse(this.transform),
				point);
		}


		public void ApplyForceAtWorldPoint(cpVect force, cpVect point)
		{
			Activate();
			this.f = cpVect.cpvadd(this.f, force);

			cpVect r = cpVect.cpvsub(point, cpTransform.Point(this.transform, this.cog));
			this.t += cpVect.cpvcross(r, force);
		}


		public void ApplyForceAtLocalPoint(cpVect force, cpVect point)
		{
			ApplyForceAtWorldPoint(cpTransform.Vect(this.transform, force),
			cpTransform.Point(this.transform, point));
		}

		public void ApplyImpulseAtWorldPoint(cpVect impulse, cpVect point)
		{
			Activate();
			cpVect r = cpVect.cpvsub(point,
				cpTransform.Point(this.transform, this.cog));

			cp.apply_impulse(this, impulse, r);
		}

		public void ApplyImpulseAtLocalPoint(cpVect impulse, cpVect point)
		{
			ApplyImpulseAtWorldPoint(
			 cpTransform.Vect(this.transform, impulse),
				cpTransform.Point(this.transform, point));
		}

		public cpVect GetVelocityAtLocalPoint(cpVect point)
		{
			cpVect r = cpTransform.Vect(
				this.transform, cpVect.cpvsub(point, this.cog));
			return cpVect.cpvadd(this.v, cpVect.cpvmult(cpVect.cpvperp(r), this.w));
		}

		public cpVect GetVelocityAtWorldPoint(cpVect point)
		{
			cpVect r = cpVect.cpvsub(point, cpTransform.Point(this.transform, this.cog));
			return cpVect.cpvadd(this.v, cpVect.cpvmult(cpVect.cpvperp(r), this.w));
		}

		// Get the kinetic energy of a body.
		public float KineticEnergy()
		{
			// Need to do some fudging to avoid NaNs
			float vsq = cpVect.cpvdot(this.v, this.v);
			float wsq = this.w * this.w;
			return (vsq != 0 ? vsq * this.m : 0.0f) + (wsq != 0 ? wsq * this.i : 0.0f);
		}

		///// Body/shape iterator callback function type. 
		//public delegate void cpBodyShapeIteratorFunc(cpBody body, cpShape shape, object data);
		public void EachShape(Action<cpShape, object> func, object data)
		{
			var shape = this.shapeList;
			while (shape != null)
			{
				var next = shape.next;
				func(shape, data);
				shape = next;
			}
		}

		public void eachShape(Action<cpShape, object> func, object data)
		{
			for (cpShape var = this.shapeList; var != null; var = var.next)
			{
				func(var, data);
			}
		}


		///// Body/raint iterator callback function type. 
		//public delegate void cpBodyConstraintIteratorFunc(cpBody body, cpConstraint raint, object data);
		public void EachConstraint(Action<cpConstraint, object> func, object data)
		{
			var constraint = this.constraintList;
			while (constraint != null)
			{
				var next = constraint.Next(this);
				func(constraint, data);
				constraint = next;
			}
		}

		public void eachConstraint(Action<cpConstraint, object> func, object data)
		{
			for (cpConstraint var = this.constraintList; var != null; var = var.Next(this))
				func(var, data);
		}


		/// Body/arbiter iterator callback function type. 
		//public delegate void cpBodyArbiterIteratorFunc(cpBody body, cpArbiter arbiter, object data);
		public void EachArbiter(Action<cpArbiter, object> func, object data)
		{
			var arb = this.arbiterList;
			while (arb != null)
			{
				var next = arb.Next(this);

				arb.swapped = (this == arb.body_b);
				func(arb, data);

				arb = next;
			}
		}

		public void eachArbiter(Action<cpArbiter, object> func, object data)
		{
			for (cpArbiter var = this.arbiterList; var != null; var = var.Next(this))
			{
				func(var, data);
			}

		}

		public void EachComponent(Func<cpBody, object, bool> func, object data)
		{
			for (cpBody body = nodeRoot; body != null; body = body.nodeNext)
			{
				func(this, data);
			}
		}

		// Defined in cpSpace.c
		// Wake up a sleeping or idle body.
		public void Activate()
		{

			if (bodyType == cpBodyType.DYNAMIC)
			{
				nodeIdleTime = 0.0f;

				cpBody root = nodeRoot;
				if (root != null && root.IsSleeping())
				{
					// TODO should cpBodyIsSleeping(root) be an assertion?
					cp.AssertSoft(root.bodyType == cpBodyType.DYNAMIC, "Internal Error: Non-dynamic body component root detected.");

					cpSpace space = root.space;
					cpBody body = root;
					while (body != null)
					{
						cpBody next = body.nodeNext;

						body.nodeIdleTime = 0.0f;
						body.nodeRoot = null;
						body.nodeNext = null;
						space.ActivateBody(body);

						body = next;
					}

					space.sleepingComponents.Remove(root);


				}

				eachArbiter((arb, o) =>
				{

					// Reset the idle timer of things the body is touching as well.
					// That way things don't get left hanging in the air.
					cpBody other = (arb.body_a == this ? arb.body_b : arb.body_a);
					if (other.bodyType != cpBodyType.STATIC) other.nodeIdleTime = 0.0f;

				}, null);


			}

		}

		// Wake up any sleeping or idle bodies touching a static body.
		public void ActivateStatic(cpShape filter)
		{
			cp.AssertHard(bodyType == cpBodyType.STATIC, "Body.activateStatic() called on a non-static body.");

			eachArbiter((arb, o) =>
			{
				if (filter == null || filter == arb.a || filter == arb.b)
				{
					(arb.body_a == this ? arb.body_b : arb.body_a).Activate();
				}

			}, null);
			// TODO should also activate joints!
		}

		//public void PushArbiter(cpArbiter arb)
		//{

		//	cp.assertSoft(cpArbiter.ThreadForBody(arb, this).next == null, "Internal Error: Dangling contact graph pointers detected. (A)");
		//	cp.assertSoft(cpArbiter.ThreadForBody(arb, this).prev == null, "Internal Error: Dangling contact graph pointers detected. (B)");

		//	cpArbiter next = this.arbiterList;

		//	cp.assertSoft(next == null || cpArbiter.ThreadForBody(next, this).prev == null, "Internal Error: Dangling contact graph pointers detected. (C)");

		//	cpArbiterThread thread = cpArbiter.ThreadForBody(arb, this);
		//	thread.next = next;

		//	if (next != null)
		//	{
		//		var threadNext = cpArbiter.ThreadForBody(next, this);
		//		threadNext.prev = arb;
		//	}

		//	this.arbiterList = arb;
		//}

		public void PushArbiter(cpArbiter arb)
		{

			cp.AssertSoft((arb.body_a == this ? arb.thread_a.next : arb.thread_b.next) == null,
			"Internal Error: Dangling contact graph pointers detected. (A)");
			cp.AssertSoft((arb.body_a == this ? arb.thread_a.prev : arb.thread_b.prev) == null,
				"Internal Error: Dangling contact graph pointers detected. (B)");

			var next = this.arbiterList;

			cp.AssertSoft(next == null || (next.body_a == this ? next.thread_a.prev : next.thread_b.prev) == null,
				"Internal Error: Dangling contact graph pointers detected. (C)");

			if (arb.body_a == this)
			{
				arb.thread_a.next = next;
			}
			else
			{
				arb.thread_b.next = next;
			}

			if (next != null)
			{
				if (next.body_a == this)
				{
					next.thread_a.prev = arb;
				}
				else
				{
					next.thread_b.prev = arb;
				}
			}
			this.arbiterList = arb;
		}






		////////////////////////////////////////////////////////



		/// Returns true if the body is static.
		//public bool IsStatic()
		//{
		//	return nodeIdleTime == cp.Infinity;
		//}

		/// Returns true if the body has not been added to a space.
		/// Note: Static bodies are a subtype of rogue bodies.
		public bool IsRogue()
		{
			return space == null;  //(cpSpace)0));
		}

		//public cpVect GetVel() { return v; }

		public void SetAngleInternal(float angle)
		{
			cp.AssertHard(!float.IsNaN(angle), "Internal Error: Attempting to set body's angle to NaN");
			this.a = angle;//fmod(a, (cpfloat)M_PI*2.0f);

			//this.rot = vforangle(angle);
			this.cog.x = cp.cpfcos(angle);
			this.cog.y = cp.cpfsin(angle);
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



		// Force a body to fall asleep immediately.
		public void Sleep()
		{
			this.SleepWithGroup(null);
		}


		// Force a body to fall asleep immediately along with other bodies in a group.
		public void SleepWithGroup(cpBody group)
		{
			cp.AssertHard(bodyType == cpBodyType.DYNAMIC, "Non-dynamic bodies cannot be put to sleep.");

			var space = this.space;
			cp.AssertHard(!space.IsLocked, "Bodies cannot be put to sleep during a query or a call to cpSpaceStep(). Put these calls into a post-step callback.");
			cp.AssertHard(space.GetSleepTimeThreshold() < cp.Infinity, "Sleeping is not enabled on the space. You cannot sleep a body without setting a sleep time threshold on the space.");
			cp.AssertHard(group == null || group.IsSleeping(), "Cannot use a non-sleeping body as a group identifier.");

			if (this.IsSleeping())
			{
				cp.AssertSoft(cp.ComponentRoot(this) == cp.ComponentRoot(group), "The body is already sleeping and it's group cannot be reassigned.");
				return;
			}

			eachShape((shape, o) => shape.CacheBB(), null);


			space.DeactivateBody(this);

			if (group != null)
			{
				var root = cp.ComponentRoot(group);

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
			space.dynamicBodies.Remove(this);

		}

		public void ActivateWrap(object unused)
		{
			Activate();
		}


		/// Allocate and initialize a cpBody.
		public static cpBody New(float mass, float moment)
		{
			cpBody tmp = new cpBody(mass, moment);
			return tmp;
		}

		/// Allocate and initialize a cpBody, and set it as a static body.
		public static cpBody NewStatic()
		{
			cpBody body = new cpBody(0.0f, 0.0f);
			body.bodyType = cpBodyType.STATIC;
			return body;
		}

		/// Allocate and initialize a cpBody, and set it as a kinematic body.
		public static cpBody NewKinematic()
		{
			cpBody body = new cpBody(0.0f, 0.0f);
			body.bodyType = cpBodyType.KINEMATIC;
			return body;
		}

		static void cpv_assert_nan(cpVect v, string message) { cp.AssertHard(v.x == v.x && v.y == v.y, message); }
		static void cpv_assert_infinite(cpVect v, string message) { cp.AssertHard(cp.cpfabs(v.x) != cp.Infinity && cp.cpfabs(v.y) != cp.Infinity, message); }
		static void cpv_assert_sane(cpVect v, string message) { cpv_assert_nan(v, message); cpv_assert_infinite(v, message); }
		public void SanityCheck()
		{
			//cp.assertHard(m >= 0.0f, "Body's mass is negative.");
			//cp.assertHard(i >= 0.0f, "Body's moment is negative.");

			//cpv_assert_sane(p, "Body's position is invalid.");
			//cpv_assert_sane(v, "Body's velocity is invalid.");
			//cpv_assert_sane(f, "Body's force is invalid.");

			//cp.assertHard(cp.cpfabs(a) != cp.Infinity, "Body's angle is invalid.");
			//cp.assertHard(cp.cpfabs(w) != cp.Infinity, "Body's angular velocity is invalid.");
			//cp.assertHard(cp.cpfabs(t) != cp.Infinity, "Body's torque is invalid.");

		}
		private void AssertSaneBody()
		{
			SanityCheck();
		}

	
	}
		#endregion


}

//public void eachShape(Action<cpShape> func)
//{
//	for (int i = 0, len = this.shapeList.Count; i < len; i++)
//	{
//		func(this.shapeList[i]);
//	}
//}

//public void eachConstraint(Action<cpConstraint> func)
//{
//	var constraint = this.constraintList;
//	while (constraint != null)
//	{
//		var next = constraint.Next(this);
//		func(constraint);
//		constraint = next;
//	}
//}

//public void eachArbiter(Action<cpArbiter> func)
//{
//	var arb = this.arbiterList;
//	while (arb != null)
//	{
//		var next = arb.Next(this);

//		arb.swapped = (this == arb.body_b);
//		func(arb);

//		arb = next;
//	}
//}
