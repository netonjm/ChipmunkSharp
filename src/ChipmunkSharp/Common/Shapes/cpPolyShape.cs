//
// cpPolyShape.cs
//
// Author:
//       Jose Medrano <josmed@microsoft.com>
//
// Copyright (c) 2015
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{

	public class cpSplittingPlane
	{

		public cpVect n, v0;

		public cpSplittingPlane(cpVect n, cpVect v0)
		{
			// TODO: Complete member initialization
			this.n = n;
			this.v0 = v0;
		}

	}

	/// @private
	public class cpPolyShape : cpShape
	{

		public const int CP_POLY_SHAPE_INLINE_ALLOC = 6;

		public float r;
		public cpSplittingPlane[] planes;

		public int Count;

		public override cpBB CacheData(cpTransform transform)
		{
			int count = this.Count;
			int dst = 0;
			int src = dst + count;

			float l = cp.Infinity, r = -cp.Infinity;
			float b = cp.Infinity, t = -cp.Infinity;

			for (int i = 0; i < count; i++)
			{
				cpVect v = cpTransform.Point(transform, this.planes[src + i].v0);
				cpVect n = cpTransform.Vect(transform, this.planes[src + i].n);

				this.planes[i + dst] = new cpSplittingPlane(n, v);

				l = cp.cpfmin(l, v.x);
				r = cp.cpfmax(r, v.x);
				b = cp.cpfmin(b, v.y);
				t = cp.cpfmax(t, v.y);
			}

			float radius = this.r;
			return (this.bb = new cpBB(l - radius, b - radius, r + radius, t + radius));
		}

		protected override void pointQuery(cpVect p, ref cpPointQueryInfo info)
		{
			int count = Count;
			cpSplittingPlane[] planes = this.planes;
			float r = this.r;

			cpVect v0 = planes[count - 1].v0;
			float minDist = cp.Infinity;
			cpVect closestPoint = cpVect.Zero;
			cpVect closestNormal = cpVect.Zero;
			bool outside = false;

			for (int i = 0; i < count; i++)
			{
				cpVect v1 = planes[i].v0;
				if (cpVect.cpvdot(planes[i].n, cpVect.cpvsub(p, v1)) > 0.0f)
					outside = true;

				cpVect closest = cp.closestPointOnSegment(p, v0, v1);

				{
					float dista = cpVect.cpvdist(p, closest);
					if (dista < minDist)
					{
						minDist = dista;
						closestPoint = closest;
						closestNormal = planes[i].n;
					}
				}

				v0 = v1;
			}

			float dist = (outside ? minDist : -minDist);
			cpVect g = cpVect.cpvmult(cpVect.cpvsub(p, closestPoint), 1.0f / dist);

			info.shape = this;
			info.point = cpVect.cpvadd(closestPoint, cpVect.cpvmult(g, r));
			info.distance = dist - r;

			// Use the normal of the closest segment if the distance is small.
			info.gradient = (minDist > cp.MAGIC_EPSILON ? g : closestNormal);
		}

		protected override void segmentQuery(cpVect a, cpVect b, float r2, ref cpSegmentQueryInfo info)
		{

			cpSplittingPlane[] planes = this.planes;
			int count = this.Count;
			float r = this.r;
			float rsum = r + r2;

			for (int i = 0; i < count; i++)
			{
				cpVect n = planes[i].n;
				float an = cpVect.cpvdot(a, n);
				float d = an - cpVect.cpvdot(planes[i].v0, n) - rsum;
				if (d < 0.0f) continue;

				float bn = cpVect.cpvdot(b, n);
				float t = d / (an - bn);
				if (t < 0.0f || 1.0f < t) continue;

				cpVect point = cpVect.cpvlerp(a, b, t);
				float dt = cpVect.cpvcross(n, point);
				float dtMin = cpVect.cpvcross(n, planes[(i - 1 + count) % count].v0);
				float dtMax = cpVect.cpvcross(n, planes[i].v0);

				if (dtMin <= dt && dt <= dtMax)
				{
					info.shape = this;
					info.point = cpVect.cpvsub(cpVect.cpvlerp(a, b, t), cpVect.cpvmult(n, r2));
					info.normal = n;
					info.alpha = t;
				}
			}

			// Also check against the beveled vertexes.
			if (rsum > 0.0f)
			{
				for (int i = 0; i < count; i++)
				{
					cpSegmentQueryInfo circle_info = new cpSegmentQueryInfo(null, b, cpVect.Zero, 1.0f);

					cp.CircleSegmentQuery(this, planes[i].v0, r, a, b, r2, ref circle_info);

					if (circle_info.alpha < info.alpha)
						info = circle_info;
				}
			}

		}

		public static cpShapeMassInfo MassInfo(float mass, int count, cpVect[] verts, float radius)
		{
			// TODO moment is approximate due to radius.
			cpVect centroid = cp.CentroidForPoly(count, verts);
			cpShapeMassInfo info = new cpShapeMassInfo(
				mass, cp.MomentForPoly(1.0f, count, verts, cpVect.cpvneg(centroid), radius),
				centroid,
				cp.AreaForPoly(count, verts, radius)
				);

			return info;
		}

		public cpPolyShape(cpBody body, int count, cpVect[] verts,
			cpTransform transform, float radius)
			: base(body, new cpShapeMassInfo())
		{

			cpVect[] hullVerts = new cpVect[count];

			// Transform the verts before building the hull in case of a negative scale.
			for (int i = 0; i < count; i++)
				hullVerts[i] = cpTransform.Point(transform, verts[i]);

			int hullCount = cp.ConvexHull(count, hullVerts, ref hullVerts, null, 0.0f);

			InitRaw(hullCount, hullVerts, radius);
		}

		public void InitRaw(int count, cpVect[] verts, float radius)
		{
			cpShapeMassInfo.cpPolyShapeMassInfo(0.0f, count, verts, radius);
			this.SetVerts(count, verts);
			this.shapeType = cpShapeType.Polygon;
			this.r = radius;
		}

		public cpPolyShape(cpBody body, int count, cpVect[] verts, float radius)
			: base(body, new cpShapeMassInfo())
		{
			InitRaw(count, verts, radius);
		}

		public cpVect GetVert(int i)
		{
			cp.AssertHard(0 <= i && i < Count, "Index out of range.");

			return this.planes[i + Count].v0;
		}

		public float GetRadius()
		{
			return this.r;
		}

		public void SetVerts(int count, cpVect[] verts)
		{
			Count = count;
			if (Count <= CP_POLY_SHAPE_INLINE_ALLOC)
				this.planes = new cpSplittingPlane[2 * CP_POLY_SHAPE_INLINE_ALLOC];
			else
				this.planes = new cpSplittingPlane[2 * Count];

			// This a pretty bad way to do this in javascript. As a first pass, I want to keep
			// the code similar to the C.

			for (int i = 0; i < Count; i++)
			{
				cpVect a = verts[(i - 1 + Count) % Count];
				cpVect b = verts[i];
				cpVect n = cpVect.cpvnormalize(cpVect.cpvrperp(cpVect.cpvsub(b, a)));

				this.planes[i + Count] = new cpSplittingPlane(n, b);
			}
		}

		public void SetVerts(int count, cpVect[] verts, cpTransform transform)
		{
			for (int i = 0; i < verts.Length; i++)
				verts[i] = cpTransform.Point(transform, verts[i]);
			SetVertsRaw(count, verts);
		}

		public void SetVertsRaw(int count, cpVect[] verts)
		{
			SetVerts(count, verts);
			float mass = this.massInfo.m;

			this.massInfo = MassInfo(this.massInfo.m, count, verts, this.r);

			if (mass > 0.0f)
				body.AccumulateMassFromShapes();
		}


		public void SetRadius(float radius)
		{
			this.r = radius;
		}
		public static cpVect[] GetVertices(cpTransform transform, int count, cpVect[] verts)
		{
			cpVect[] hullVerts = new cpVect[count];

			for (int i = 0; i < count; i++)
			{
				hullVerts[i] = cpTransform.Point(transform, verts[i]);
			}
			return hullVerts;
		}


		//public override void Draw(cpDebugDraw m_debugDraw)
		//{

		//	if (m_debugDraw.Flags.HasFlag(cpDrawFlags.Shapes) || m_debugDraw.Flags.HasFlag(cpDrawFlags.All))
		//	{
		//		cpColor color = cp.GetShapeColor(this);
		//		int count = Count;
		//		m_debugDraw.DrawPolygon(GetVertices(), count, color);
		//	}

		//	if (m_debugDraw.Flags.HasFlag(cpDrawFlags.BB) || m_debugDraw.Flags.HasFlag(cpDrawFlags.All))
		//		bb.Draw(m_debugDraw);
		//}

		public static cpPolyShape BoxShape(cpBody body, float width, float height, float radius)
		{
			var hw = width / 2f;
			var hh = height / 2f;

			return BoxShape2(body, new cpBB(-hw, -hh, hw, hh), radius);


		}

		public static cpPolyShape BoxShape2(cpBody body, cpBB box, float radius)
		{

			cpVect[] verts = new cpVect[] {
		new cpVect(box.r, box.b),
		new cpVect(box.r, box.t),
		new cpVect(box.l, box.t),
		new cpVect(box.l, box.b),
	};

			return new cpPolyShape(body, 4, verts, radius);
		}



		/// /////////////////////////////////////////////////////////////

		public cpVect[] GetVertices()
		{
			int count = Count;
			cpVect[] verts = new cpVect[count];
			for (int i = 0; i < count; i++)
					verts[i] = new cpVect(this.planes[i].v0);

			return verts;


		}

	}

}
