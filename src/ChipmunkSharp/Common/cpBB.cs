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
namespace ChipmunkSharp
{

	public class cpBB
	{

		public int numBB = 0;
		public float l, b, r, t;

		public cpBB(float l1, float b1, float r1, float t1)
		{
			l = l1;
			b = b1;
			r = r1;
			t = t1;
			numBB++;
		}

		public cpBB(cpVect p, float r)
			: this(p.x - r, p.y - r, p.x + r, p.y + r)
		{

		}

		public override string ToString()
		{
			return string.Format("l:{0},b:{1},r:{2},t:{3}", l, b, r, t);
		}

		/// ructs a cpBB for a circle with the given position and radius.
		public static cpBB NewForCircle(cpVect p, float r)
		{
			return new cpBB(p.x - r, p.y - r, p.x + r, p.y + r);
		}

		public float Proximity(cpBB b)
		{
			return Proximity(this, b);
		}

		public static float Proximity(Node a, Leaf b)
		{
			return Proximity(a.bb, b.bb);
		}

		public static float Proximity(cpBB a, cpBB b)
		{
			return cp.cpfabs(a.l + a.r - b.l - b.r) + cp.cpfabs(a.b + a.t - b.b - b.t);
		}

		/// Returns true if @c a and @c b intersect.
		public bool Intersects(cpBB a)
		{
			return Intersects(this, a);
		}
		public static bool Intersects(cpBB a, cpBB b)
		{
			return (a.l <= b.r && b.l <= a.r && a.b <= b.t && b.b <= a.t);
		}

		/// Returns true if @c other lies completely within @c bb.
		public bool ContainsBB(cpBB other)
		{
			return ContainsBB(this, other);
		}
		public static bool ContainsBB(cpBB bb, cpBB other)
		{
			return (bb.l <= other.l && bb.r >= other.r && bb.b <= other.b && bb.t >= other.t);
		}

		/// Returns true if @c bb contains @c v.
		public bool ContainsVect(cpVect v)
		{
			return ContainsVect(this, v);
		}
		public static bool ContainsVect(cpBB bb, cpVect v)
		{
			return (bb.l <= v.x && bb.r >= v.x && bb.b <= v.y && bb.t >= v.y);
		}

		/// Returns a bounding box that holds both bounding boxes.
		public cpBB Merge(cpBB a)
		{
			return Merge(this, a);
		}
		public static cpBB Merge(cpBB a, cpBB b)
		{
			return new cpBB(
				cp.cpfmin(a.l, b.l),
				cp.cpfmin(a.b, b.b),
				cp.cpfmax(a.r, b.r),
				cp.cpfmax(a.t, b.t)
			);
		}

		/// Returns a bounding box that holds both @c bb and @c v.
		public cpBB Expand(cpVect v)
		{
			return Expand(this, v);
		}
		public static cpBB Expand(cpBB bb, cpVect v)
		{
			return new cpBB(
				cp.cpfmin(bb.l, v.x),
				cp.cpfmin(bb.b, v.y),
				cp.cpfmax(bb.r, v.x),
				cp.cpfmax(bb.t, v.y)
			);
		}

		/// Returns the center of a bounding box.
		public static cpVect Center(cpBB bb)
		{
			return cpVect.cpvlerp(new cpVect(bb.l, bb.b), new cpVect(bb.r, bb.t), 0.5f);
		}
		public cpVect Center()
		{
			return Center(this);
		}


		/// Returns the area of the bounding box.
		public static float Area(cpBB bb)
		{
			return (bb.r - bb.l) * (bb.t - bb.b);
		}
		public float Area()
		{
			return Area(this);
		}


		/// Merges @c a and @c b and returns the area of the merged bounding box.
		public static float MergedArea(cpBB a, cpBB b)
		{
			return (cp.cpfmax(a.r, b.r) - cp.cpfmin(a.l, b.l)) * (cp.cpfmax(a.t, b.t) - cp.cpfmin(a.b, b.b));
		}

		public float MergedArea(cpBB a)
		{
			return MergedArea(this, a);
		}


		/// Returns the fraction along the segment query the cpBB is hit. Returns INFINITY if it doesn't hit.
		public static float SegmentQuery(cpBB bb, cpVect a, cpVect b)
		{
			float idx = 1.0f / (b.x - a.x);
			float tx1 = (bb.l == a.x ? -cp.Infinity : (bb.l - a.x) * idx);
			float tx2 = (bb.r == a.x ? cp.Infinity : (bb.r - a.x) * idx);
			float txmin = cp.cpfmin(tx1, tx2);
			float txmax = cp.cpfmax(tx1, tx2);

			float idy = 1.0f / (b.y - a.y);
			float ty1 = (bb.b == a.y ? -cp.Infinity : (bb.b - a.y) * idy);
			float ty2 = (bb.t == a.y ? cp.Infinity : (bb.t - a.y) * idy);
			float tymin = cp.cpfmin(ty1, ty2);
			float tymax = cp.cpfmax(ty1, ty2);

			if (tymin <= txmax && txmin <= tymax)
			{
				float min = cp.cpfmax(txmin, tymin);
				float max = cp.cpfmin(txmax, tymax);

				if (0.0 <= max && min <= 1.0) return cp.cpfmax(min, 0.0f);
			}

			return cp.Infinity;
		}

		public float SegmentQuery(cpVect a, cpVect b)
		{
			return SegmentQuery(this, a, b);
		}


		/// Return true if the bounding box intersects the line segment with ends @c a and @c b.
		public static bool IntersectsSegment(cpBB bb, cpVect a, cpVect b)
		{
			return (SegmentQuery(bb, a, b) != cp.Infinity);
		}
		public bool IntersectsSegment(cpVect a, cpVect b)
		{
			return IntersectsSegment(this, a, b);
		}


		/// Clamp a vector to a bounding box.
		public static cpVect ClampVect(cpBB bb, cpVect v)
		{
			return cpVect.cpv(cp.cpfclamp(v.x, bb.l, bb.r), cp.cpfclamp(v.y, bb.b, bb.t));
		}
		public cpVect ClampVect(cpVect v)
		{
			return ClampVect(this, v);
		}


		/// Wrap a vector to a bounding box.
		public static cpVect WrapVect(cpBB bb, cpVect v)
		{
			// wrap a vector to a bbox
			float ix = cp.cpfabs(bb.r - bb.l);
			float modx = (v.x - bb.l) % ix;
			float x = (modx > 0) ? modx : modx + ix;

			float iy = cp.cpfabs(bb.t - bb.b);
			float mody = (v.y - bb.b) % iy;
			float y = (mody > 0) ? mody : mody + iy;

			return new cpVect(x + bb.l, y + bb.b);
		}

		/// Constructs a cpBB for a circle with the given position and radius.
		public static cpBB cpBBNewForCircle(cpVect p, float r)
		{
			return cpBBNew(p.x - r, p.y - r, p.x + r, p.y + r);
		}

		/// Convenience constructor for cpBB structs.
		public static cpBB cpBBNew(float l, float b, float r, float t)
		{
			cpBB bb = new cpBB(l, b, r, t);
			return bb;
		}

		/// Constructs a cpBB centered on a point with the given extents (half sizes).
		public static cpBB NewForExtents(cpVect c, float hw_max, float hh_max)
		{
			return cpBBNew(c.x - hw_max, c.y - hh_max, c.x + hw_max, c.y + hh_max);
		}

		//public void Draw(cpDebugDraw m_debugDraw)
		//{
		//	Draw(m_debugDraw, cpColor.Green);
		//}

		//public void Draw(cpDebugDraw m_debugDraw,cpColor color)
		//{
		//	m_debugDraw.DrawBB(this, color);
		//}

	} ;

}


