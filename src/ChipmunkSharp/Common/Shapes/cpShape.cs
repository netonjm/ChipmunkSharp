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

using System.Collections.Generic;
using System;
using System.Linq;
namespace ChipmunkSharp
{


	public enum cpShapeType
	{
		Circle,
		Segment,
		Polygon,
		NumShapes
	};

	public class cpCollisionInfo
	{
		public cpShape a, b;
		public ulong id;
		public cpVect n;
		public int count;

		// TODO Should this be a unique struct type?

		public List<cpContact> arr;

		public cpCollisionInfo(cpShape a, cpShape b, ulong id, cpVect n, List<cpContact> contacts)
		{
			// TODO: Complete member initialization
			this.a = a;
			this.b = b;
			this.id = id;
			this.n = n;

			this.arr = contacts;
		}
	};


	public class cpShapeFilter
	{
		public int group;
		public int categories;
		public int mask;

		public cpShapeFilter()
		{

		}

		public cpShapeFilter(int group, int categories, int mask)
		{
			this.group = group;
			this.categories = categories;
			this.mask = mask;
		}

		public bool Reject(cpShapeFilter b)
		{
			return Reject(this, b);
		}

		public static bool Reject(cpShapeFilter a, cpShapeFilter b)
		{
			// Reject the collision if:
			return (
				// They are in the same non-zero group.
				(a.group != 0 && a.group == b.group) ||
				// One of the category/mask combinations fails.
				(a.categories & b.mask) == 0 ||
				(b.categories & a.mask) == 0
			);
		}


	}

	public struct cpShapeMassInfo
	{
		public float m;
		public float i;
		public cpVect cog;
		public float area;

		public cpShapeMassInfo(float m, float i, cpVect cog, float area)
		{
			this.m = m;
			this.i = i;
			this.cog = cog;
			this.area = area;
		}

		public static cpShapeMassInfo cpSegmentShapeMassInfo(float mass, cpVect a, cpVect b, float r)
		{
			var info = new cpShapeMassInfo(
	mass, cp.MomentForBox(1.0f, cpVect.cpvdist(a, b) + 2.0f * r, 2.0f * r), // TODO is an approximation.
	cpVect.cpvlerp(a, b, 0.5f),
	cp.AreaForSegment(a, b, r)
);
			return info;
		}

		public static cpShapeMassInfo cpCircleShapeMassInfo(float mass, float radius, cpVect center)
		{
			var info = new cpShapeMassInfo(
					mass, cp.MomentForCircle(1.0f, 0.0f, radius, cpVect.Zero),
		center,
		cp.AreaForCircle(0.0f, radius)
				);
			return info;

		}

		public static cpShapeMassInfo cpPolyShapeMassInfo(float mass, int count, cpVect[] verts, float radius)
		{

			cpVect centroid = cp.CentroidForPoly(count, verts);

			var info = new cpShapeMassInfo(
				mass,
				cp.MomentForPoly(1.0f, count, verts, cpVect.cpvneg(centroid), radius),
				centroid,
				cp.AreaForCircle(0.0f, radius)
			);
			return info;
		}
	};

	public class cpCircleShape : cpShape
	{

		public cpVect c, tc;

		public float r;


		public cpCircleShape(cpBody body, float radius, cpVect offset)
			: base(body, cpShapeMassInfo.cpCircleShapeMassInfo(0.0f, radius, offset))
		{
			this.c = offset;
			this.r = radius;

			this.shapeType = cpShapeType.Circle;
		}

		protected override void segmentQuery(cpVect a, cpVect b, float radius, ref cpSegmentQueryInfo info)
		{
			cp.CircleSegmentQuery(this, this.tc, this.r, a, b, radius, ref info);
		}

		protected override void pointQuery(cpVect p, ref cpPointQueryInfo info)
		{
			//	base.pointQuery(p, ref info);
			cpVect delta = cpVect.cpvsub(p, this.tc);
			float d = cpVect.cpvlength(delta);
			float r = this.r;

			info.shape = this;
			info.point = cpVect.cpvadd(this.tc, cpVect.cpvmult(delta, r / d)); // TODO: div/0
			info.distance = d - r;

			// Use up for the gradient if the distance is very small.
			info.gradient = (d > cp.MAGIC_EPSILON ? cpVect.cpvmult(delta, 1.0f / d) : new cpVect(0.0f, 1.0f));

		}

		public override cpBB CacheData(cpTransform transform)
		{
			cpVect c = this.tc = cpTransform.Point(transform, this.c);
			return new cpBB(c, this.r);
		}

		/// <summary>
		/// Gets the circle radius. Parameter name: .r
		/// </summary>
		/// <returns></returns>
		public float GetRadius()
		{
			return this.r;
		}

		public void SetRadius(float radius)
		{
			this.r = radius;

			float mass = this.massInfo.m;
			this.massInfo = cpShapeMassInfo.cpCircleShapeMassInfo(mass, this.r, this.c);
			if (mass > 0.0f)
				this.body.AccumulateMassFromShapes();
		}


		/// <summary>
		/// Gets the circle offset. Parameter name: .c
		/// </summary>
		/// <returns></returns>
		public cpVect GetOffset()
		{
			return this.c;
		}

		public void SetOffset(cpVect offset)
		{
			this.c = offset;

			float mass = this.massInfo.m;
			this.massInfo = cpShapeMassInfo.cpCircleShapeMassInfo(mass, this.r, this.c);
			if (mass > 0.0f)
				this.body.AccumulateMassFromShapes();
		}




		//public override void Draw(cpDebugDraw m_debugDraw)
		//{

		//	if (m_debugDraw.Flags.HasFlag(cpDrawFlags.Shapes) || m_debugDraw.Flags.HasFlag(cpDrawFlags.All))
		//	{
		//		cpColor color = cp.GetShapeColor(this);
		//		m_debugDraw.DrawSolidCircle(new cpVect(tc.x, tc.y), r, cpVect.Zero, color);
		//		m_debugDraw.DrawSegment(tc, cpVect.Multiply(body.GetRotation(), this.r).Add(this.tc), color);
		//	}

		//	if (m_debugDraw.Flags.HasFlag(cpDrawFlags.BB) || m_debugDraw.Flags.HasFlag(cpDrawFlags.All))
		//		bb.Draw(m_debugDraw);

		//}

	}

	public class cpSegmentShape : cpShape
	{

		public cpVect a, b, n;
		public cpVect ta, tb, tn;
		public float r;

		public cpVect a_tangent, b_tangent;

		public override cpBB CacheData(cpTransform transform)
		{
			this.ta = cpTransform.Point(transform, this.a);
			this.tb = cpTransform.Point(transform, this.b);
			this.tn = cpTransform.Vect(transform, this.n);

			float l, r, b, t;

			if (this.ta.x < this.tb.x)
			{
				l = this.ta.x;
				r = this.tb.x;
			}
			else
			{
				l = this.tb.x;
				r = this.ta.x;
			}

			if (this.ta.y < this.tb.y)
			{
				b = this.ta.y;
				t = this.tb.y;
			}
			else
			{
				b = this.tb.y;
				t = this.ta.y;
			}

			float rad = this.r;
			return new cpBB(l - rad, b - rad, r + rad, t + rad);
		}

		protected override void pointQuery(cpVect p, ref cpPointQueryInfo info)
		{
			cpVect closest = cp.closestPointOnSegment(p, this.ta, this.tb);

			cpVect delta = cpVect.cpvsub(p, closest);
			float d = cpVect.cpvlength(delta);
			float r = this.r;
			cpVect g = cpVect.cpvmult(delta, 1.0f / d);

			info.shape = (cpShape)this;
			info.point = (d != 0 ? cpVect.cpvadd(closest, cpVect.cpvmult(g, r)) : closest);
			info.distance = d - r;

			// Use the segment's normal if the distance is very small.
			info.gradient = (d > cp.MAGIC_EPSILON ? g : this.n);
		}

		protected override void segmentQuery(cpVect a, cpVect b, float r2, ref cpSegmentQueryInfo info)
		{
			cpVect n = this.tn;
			float d = cpVect.cpvdot(cpVect.cpvsub(this.ta, a), n);
			float r = this.r + r2;

			cpVect flipped_n = (d > 0.0f ? cpVect.cpvneg(n) : n);
			cpVect seg_offset = cpVect.cpvsub(cpVect.cpvmult(flipped_n, r), a);

			// Make the endpoints relative to 'a' and move them by the thickness of the segment.
			cpVect seg_a = cpVect.cpvadd(this.ta, seg_offset);
			cpVect seg_b = cpVect.cpvadd(this.tb, seg_offset);
			cpVect delta = cpVect.cpvsub(b, a);

			if (cpVect.cpvcross(delta, seg_a) * cpVect.cpvcross(delta, seg_b) <= 0.0f)
			{
				float d_offset = d + (d > 0.0f ? -r : r);
				float ad = -d_offset;
				float bd = cpVect.cpvdot(delta, n) - d_offset;

				if (ad * bd < 0.0f)
				{
					float t = ad / (ad - bd);

					info.shape = (cpShape)this;
					info.point = cpVect.cpvsub(cpVect.cpvlerp(a, b, t), cpVect.cpvmult(flipped_n, r2));
					info.normal = flipped_n;
					info.alpha = t;
				}
			}
			else if (r != 0.0f)
			{
				cpSegmentQueryInfo info1 = new cpSegmentQueryInfo(null, b, cpVect.Zero, 1.0f);
				cpSegmentQueryInfo info2 = new cpSegmentQueryInfo(null, b, cpVect.Zero, 1.0f);

				cp.CircleSegmentQuery(this, this.ta, this.r, a, b, r2, ref info1);
				cp.CircleSegmentQuery(this, this.tb, this.r, a, b, r2, ref info2);

				if (info1.alpha < info2.alpha)
				{
					info = info1;
				}
				else
				{
					info = info2;
				}
			}
		}

		public cpSegmentShape(cpBody body, cpVect a, cpVect b, float r)
			: base(body, cpShapeMassInfo.cpSegmentShapeMassInfo(0.0f, a, b, r))
		{


			this.a = a;
			this.b = b;

			this.n = cpVect.cpvrperp(cpVect.vnormalize(cpVect.cpvsub(b, a)));

			this.r = r;

			this.a_tangent = cpVect.Zero;
			this.b_tangent = cpVect.Zero;

			this.shapeType = cpShapeType.Segment;

		}


		public cpVect GetA()
		{
			return this.a;
		}

		public cpVect GetB()
		{
			return this.b;
		}

		public cpVect GetNormal()
		{
			return this.n;
		}

		public float GetRadius()
		{
			return this.r;
		}

		public void SetRadius(float radius)
		{

			this.r = radius;

			float mass = this.massInfo.m;
			this.massInfo = cpShapeMassInfo.cpSegmentShapeMassInfo(this.massInfo.m, this.a, this.b, this.r);
			if (mass > 0.0f)
				this.body.AccumulateMassFromShapes();
		}


		public void SetNeighbors(cpVect prev, cpVect next)
		{
			this.a_tangent = cpVect.cpvsub(prev, this.a);
			this.b_tangent = cpVect.cpvsub(next, this.b);
		}

		public void SetEndpoints(cpVect a, cpVect b)
		{
			this.a = a;
			this.b = b;
			this.n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(b, a)));

			float mass = this.massInfo.m;
			this.massInfo = cpShapeMassInfo.cpSegmentShapeMassInfo(massInfo.m, this.a, this.b, this.r);
			if (mass > 0.0f)
				this.body.AccumulateMassFromShapes();

		}


		/// ///////////////////////////////////////////////////////////


		//public override void Draw(cpDebugDraw m_debugDraw)
		//{


		//	if (m_debugDraw.Flags.HasFlag(cpDrawFlags.Shapes) || m_debugDraw.Flags.HasFlag(cpDrawFlags.All))
		//	{
		//		cpColor color = cp.GetShapeColor(this);
		//		var lineWidth = cp.cpfmax(1, this.r);  // take a look if we need to apply scale for radius
		//		m_debugDraw.DrawSegment(ta, tb, lineWidth, color);
		//	}

		//	if (m_debugDraw.Flags.HasFlag(cpDrawFlags.BB) || m_debugDraw.Flags.HasFlag(cpDrawFlags.All))
		//		bb.Draw(m_debugDraw);

		//}

	}

	/// Opaque collision shape struct.
	public class cpShape : IObjectBox
	{

		#region PROPS

		public cpShapeType shapeType;

		public cpSpace space;

		/// The rigid body this collision shape is attached to.
		public cpBody body;

		/// The current bounding box of the shape.
		public cpBB bb { get; set; }

		public cpShapeMassInfo massInfo;


		/// Sensor flag.
		/// Sensor shapes call collision callbacks but don't produce collisions.
		public bool sensor;

		/// Coefficient of restitution. (elasticity)
		public float e;
		/// Coefficient of friction.
		public float u;
		/// Surface velocity used when solving for friction.
		public cpVect surfaceV;

		/// User definable data pointer.
		/// Generally this points to your the game object class so you can access it
		/// when given a cpShape reference in a callback.
		public object userData;


		/// Collision type of this shape used when picking collision handlers.
		public ulong type;

		public cpShapeFilter filter { get; set; }


		public cpShape next;

		public cpShape prev;

		public ulong hashid;

		#endregion

		public static cpShapeFilter FILTER_ALL = new cpShapeFilter(cp.NO_GROUP, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES);
		public static cpShapeFilter FILTER_NONE = new cpShapeFilter(cp.NO_GROUP, ~cp.ALL_CATEGORIES, ~cp.ALL_CATEGORIES);


		public cpShape(cpBody body, cpShapeMassInfo massInfo)
		{




			/// The rigid body this collision shape is attached to.
			this.body = body;

			this.massInfo = massInfo;

			/// The current bounding box of the shape.
			/// The current bounding box of the shape.
			/// 
			//this.bb_l = this.bb_b = this.bb_r = this.bb_t = 0;
			this.bb = new cpBB(0, 0, 0, 0);

			//this.hashid = (cp.shapeIDCounter++).ToString();

			/// Sensor flag.
			/// Sensor shapes call collision callbacks but don't produce collisions.
			this.sensor = false;

			filter = new cpShapeFilter(cp.NO_GROUP, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES);


			/// Coefficient of restitution. (elasticity)
			this.e = 0;
			/// Coefficient of friction.
			this.u = 0;
			/// Surface velocity used when solving for friction.
			this.surfaceV = cpVect.Zero;

			/// Collision type of this shape used when picking collision handlers.
			this.type = 0;

			this.space = null;
		}


		public cpSpace GetSpace()
		{
			return this.space;
		}

		public cpBody GetBody()
		{
			return this.body;
		}

		public void SetBody(cpBody body)
		{
			cp.AssertHard(!Active(), "You cannot change the body on an active shape. You must remove the shape from the space before changing the body.");
			this.body = body;
		}

		public float GetMass()
		{
			return this.massInfo.m;
		}

		public void SetMass(float mass)
		{
			this.body.Activate();

			this.massInfo.m = mass;
			this.body.AccumulateMassFromShapes();
		}

		public float GetMoment()
		{
			return this.massInfo.m / this.massInfo.area;
		}

		public float GetArea() { return this.massInfo.area; }
		public cpVect GetCenterOfGravity() { return this.massInfo.cog; }

		public cpBB GetBB()
		{
			return this.bb;
		}

		public bool GetSensor()
		{
			return this.sensor;
		}

		public void SetSensor(bool sensor)
		{
			this.body.Activate();
			this.sensor = sensor;
		}

		public float GetElasticity()
		{
			return this.e;
		}

		public void SetElasticity(float elasticity)
		{
			cp.AssertHard(elasticity >= 0.0f, "Elasticity must be positive and non-zero.");
			this.body.Activate();
			// throw new NotImplementedException();
			this.e = elasticity;
		}

		public object GetUserData()
		{
			return this.userData;
		}

		public void SetUserData(object userData)
		{
			this.userData = userData;
		}

		public ulong GetCollisionType()
		{
			return this.type;
		}

		public void SetCollisionType(ulong collisionType)
		{
			this.body.Activate();
			this.type = collisionType;
		}

		public cpShapeFilter GetFilter()
		{
			return this.filter;
		}

		public void SetFilter(cpShapeFilter filter)
		{
			body.Activate();
			this.filter = filter;
		}

		public virtual cpBB CacheBB()
		{
			return Update(body.transform);

		}
		public cpBB Update(cpTransform transform)
		{
			return (this.bb = CacheData(transform));
		}

		public float PointQuery(cpVect p, ref cpPointQueryInfo info)
		{
			if (info == null)
				info = new cpPointQueryInfo(null, cpVect.Zero, cp.Infinity, cpVect.Zero);

			pointQuery(p, ref info);

			return info.distance;
		}

		public bool SegmentQuery(cpVect a, cpVect b, float radius, ref cpSegmentQueryInfo info)
		{
			if (info == null)
				info = new cpSegmentQueryInfo(null, b, cpVect.Zero, 1.0f);

			cpPointQueryInfo nearest = null;
			PointQuery(a, ref nearest);

			if (nearest.distance <= radius)
			{
				info.shape = this;
				info.alpha = 0.0f;
				info.normal = cpVect.cpvnormalize(cpVect.cpvsub(a, nearest.point));
			}
			else
			{
				segmentQuery(a, b, radius, ref info);
			}

			return info.shape != null;

		}

		protected virtual void pointQuery(cpVect p, ref cpPointQueryInfo info)
		{
			throw new NotImplementedException();

		}
		protected virtual void segmentQuery(cpVect a, cpVect b, float radius, ref cpSegmentQueryInfo info)
		{
			throw new NotImplementedException();
		}
		public cpContactPointSet Collide(cpShape b, ref List<cpContact> contacts)
		{
			return Collide(this, b, ref contacts);
		}
		/// Test if a point lies within a shape.
		public static cpContactPointSet Collide(cpShape a, cpShape b, ref List<cpContact> contacts)
		{
			//cpContact[] contacts = new cpContact[cpArbiter.CP_MAX_CONTACTS_PER_ARBITER];
			cpCollisionInfo info = cpCollision.cpCollide(a, b, 0, ref contacts);

			cpContactPointSet set = new cpContactPointSet();
			set.count = info.count;

			set.points = new PointsDistance[set.count];

			// cpCollideShapes() may have swapped the contact order. Flip the normal.
			bool swapped = (a != info.a);
			set.normal = (swapped ? cpVect.cpvneg(info.n) : info.n);

			for (int i = 0; i < info.count; i++)
			{
				cpVect p1 = contacts[i].r1;
				cpVect p2 = contacts[i].r2;

				set.points[i] = new PointsDistance();
				set.points[i].pointA = (swapped ? p2 : p1);
				set.points[i].pointB = (swapped ? p1 : p2);
				set.points[i].distance = cpVect.cpvdot(cpVect.cpvsub(p2, p1), set.normal);
			}

			return set;
		}

		public void SetDensity(float density)
		{
			SetMass(density * this.massInfo.area);
		}

		public float GetFriction()
		{
			return this.u;
		}
		public void SetFriction(float friction)
		{
			cp.AssertHard(friction >= 0.0f, "Friction must be postive and non-zero.");
			this.body.Activate();
			this.u = friction;
		}

		public cpVect GetSurfaceVelocity()
		{
			return this.surfaceV;
		}

		public void SetSurfaceVelocity(cpVect surfaceVelocity)
		{
			this.body.Activate();
			this.surfaceV = surfaceVelocity;
		}

		public bool Active()
		{
			return (this.prev != null && this.body != null && this.body.shapeList == this);
			//return this.body != null && this.body.shapeList.IndexOf(this) != -1;
		}

		public static void UpdateFunc(cpShape shape, object unused)
		{
			shape.CacheBB();
		}

		public virtual cpBB CacheData(cpTransform transform)
		{
			throw new NotImplementedException();
		}

		//public virtual void Draw(cpDebugDraw m_debugDraw)
		//{
		//	throw new NotImplementedException();
		//}
		/// /////////////////////////////////////////////////////////////////////////



	};

}
