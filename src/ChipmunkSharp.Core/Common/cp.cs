//
// cp.cs
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
using System.Diagnostics;

namespace ChipmunkSharp
{

	public class hash
	{

		public int[] values;
		public int count;

		public hash(int value)
		{
			values = new int[50];
			SetValue(value);
		}

		public hash(hash value)
		{
			values = new int[50];
			for (int i = 0; i < value.count; i++)
			{

			}
		}

		public void SetValue(int value)
		{

		}

	}

	public struct cpMat2x2
	{

		public float a, b, c, d;

		public cpMat2x2(float a, float b, float c, float d)
		{
			this.a = a;
			this.b = b;
			this.c = c;
			this.d = d;
		}

		public static cpVect Transform(cpMat2x2 m, cpVect v)
		{
			return new cpVect(v.x * m.a + v.y * m.b, v.x * m.c + v.y * m.d);
		}

	} ;

	public class cp
	{

		public const float M_E = 2.71828182845904523536f;
		public const float M_LOG2E = 1.44269504088896340736f;
		public const float M_LOG10E = 0.434294481903251827651f;
		public const float M_LN2 = 0.693147180559945309417f;
		public const float M_LN10 = 2.30258509299404568402f;
		public const float M_PI = 3.14159265358979323846f;
		public const float M_PI_2 = 1.57079632679489661923f;
		public const float M_PI_4 = 0.785398163397448309616f;
		public const float M_1_PI = 0.318309886183790671538f;
		public const float M_2_PI = 0.636619772367581343076f;
		public const float M_2_SQRTPI = 1.12837916709551257390f;
		public const float M_SQRT2 = 1.41421356237309504880f;
		public const float M_SQRT1_2 = 0.707106781186547524401f;

		public static ulong COLLISION_TYPE_STICKY = 1;
		public static ulong WILDCARD_COLLISION_TYPE
		{
			get
			{
				int parse = ~0;
				return (ulong)parse;
			}
		}

		// Chipmunk 6.2.1
		public static string CP_VERSION_MAJOR = "7";
		public static string CP_VERSION_MINOR = "0";
		public static string CP_VERSION_RELEASE = "0";

		public static string cpVersionString = string.Format("{0}.{1}.{2}", CP_VERSION_MAJOR, CP_VERSION_MINOR, CP_VERSION_RELEASE);

		public static int numLeaves { get; set; }
		public static int numNodes { get; set; }
		public static int numPairs { get; set; }
		public static int numContacts { get; set; }


		public static ulong shapeIDCounter;
		public static int CP_USE_CGPOINTS = 1;

		public const int ALL_CATEGORIES = ~0;
		//public const int WILDCARD_COLLISION_TYPE = ~0;

		public static int NO_GROUP = 0;
		public static int ALL_LAYERS = ~0;

		public static int numApplyImpulse = 0;
		public static int numApplyContact = 0;

		public static float scale;

		public static byte[] INFINITY = { 0x00, 0x00, 0x80, 0x7F };

		public static float MAGIC_EPSILON = 1e-5F;

		public static float Infinity
		{
			get
			{
				return BitConverter.ToSingle(INFINITY, 0);
			}
		}

		public static ulong CP_HASH_COEF = 3344921057ul;

		public static ulong CP_HASH_PAIR(ulong a, ulong b)
		{
			return a * CP_HASH_COEF ^ b * CP_HASH_COEF;
		}

		public static float PHYSICS_INFINITY { get { return Infinity; } }

		public static void resetShapeIdCounter()
		{
			shapeIDCounter = 0;
		}

		public static void CircleSegmentQuery(cpShape shape, cpVect center, float r1, cpVect a, cpVect b, float r2, ref cpSegmentQueryInfo info)
		{
			// offset the line to be relative to the circle
			cpVect da = cpVect.cpvsub(a, center);
			cpVect db = cpVect.cpvsub(b, center);
			float rsum = r1 + r2;


			float qa = cpVect.cpvdot(da, da) - 2 * cpVect.cpvdot(da, db) + cpVect.cpvdot(db, db);
			float qb = cpVect.cpvdot(da, db) - cpVect.cpvdot(da, da);
			float det = qb * qb - qa * (cpVect.cpvdot(da, da) - rsum * rsum);

			if (det >= 0.0f)
			{
				float t = (-qb - cp.cpfsqrt(det)) / (qa);
				if (0.0f <= t && t <= 1.0f)
				{
					{
						cpVect n = cpVect.cpvnormalize(cpVect.cpvlerp(da, db, t));

						info.shape = shape;
						info.point = cpVect.cpvsub(cpVect.cpvlerp(da, db, t), cpVect.cpvmult(n, r2));
						info.normal = n;
						info.alpha = t;


					}

				}
			}
		}
		public static cpVect relative_velocity(cpBody a, cpBody b, cpVect r1, cpVect r2)
		{

			cpVect v1_sum = cpVect.cpvadd(a.v, cpVect.cpvmult(cpVect.cpvperp(r1), a.w));
			cpVect v2_sum = cpVect.cpvadd(b.v, cpVect.cpvmult(cpVect.cpvperp(r2), b.w));

			return cpVect.cpvsub(v2_sum, v1_sum);
		}

		public static float normal_relative_velocity(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect n)
		{
			return cpVect.cpvdot(relative_velocity(a, b, r1, r2), n);
		}

		public static void apply_impulse(cpBody body, cpVect j, cpVect r)
		{
			body.v = cpVect.cpvadd(body.v, cpVect.cpvmult(j, body.m_inv));
			body.w += body.i_inv * cpVect.cpvcross(r, j);
		}

		public static void apply_impulses(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect j)
		{
			apply_impulse(a, cpVect.cpvneg(j), r1);
			apply_impulse(b, j, r2);
		}

		public static void apply_bias_impulse(cpBody body, cpVect j, cpVect r)
		{
			body.v_bias = cpVect.cpvadd(body.v_bias, cpVect.cpvmult(j, body.m_inv));
			body.w_bias += body.i_inv * cpVect.cpvcross(r, j);
		}


		public static void apply_bias_impulses(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect j)
		{
			apply_bias_impulse(a, cpVect.cpvneg(j), r1);
			apply_bias_impulse(b, j, r2);
		}


		public static float k_scalar_body(cpBody body, cpVect r, cpVect n)
		{
			var rcn = cpVect.cpvcross(r, n);
			return body.m_inv + body.i_inv * rcn * rcn;
		}

		public static float k_scalar(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect n)
		{
			var value = k_scalar_body(a, r1, n) + k_scalar_body(b, r2, n);

			cp.AssertSoft(value != 0, "Unsolvable collision or constraint.");

			return value;
		}

		// k1 and k2 are modified by the function to contain the outputs.

		public static cpMat2x2 k_tensor(cpBody a, cpBody b, cpVect r1, cpVect r2)
		{
			float m_sum = a.m_inv + b.m_inv;

			// start with Identity*m_sum
			float k11 = m_sum, k12 = 0.0f;
			float k21 = 0.0f, k22 = m_sum;

			// add the influence from r1
			float a_i_inv = a.i_inv;
			float r1xsq = r1.x * r1.x * a_i_inv;
			float r1ysq = r1.y * r1.y * a_i_inv;
			float r1nxy = -r1.x * r1.y * a_i_inv;
			k11 += r1ysq; k12 += r1nxy;
			k21 += r1nxy; k22 += r1xsq;

			// add the influnce from r2
			float b_i_inv = b.i_inv;
			float r2xsq = r2.x * r2.x * b_i_inv;
			float r2ysq = r2.y * r2.y * b_i_inv;
			float r2nxy = -r2.x * r2.y * b_i_inv;
			k11 += r2ysq; k12 += r2nxy;
			k21 += r2nxy; k22 += r2xsq;

			// invert
			float det = k11 * k22 - k12 * k21;
			cp.AssertSoft(det != 0.0f, "Unsolvable constraint.");

			float det_inv = 1.0f / det;
			return new cpMat2x2(
				 k22 * det_inv, -k12 * det_inv,
				-k21 * det_inv, k11 * det_inv
			);
		}

		public static float bias_coef(float errorBias, float dt)
		{
			return 1.0f - cpfpow(errorBias, dt);
		}

		//===========================================================

		/** Clamp a value between from and to.
		   @since v0.99.1
	   */
		#region Mathemathical Operations and variables

		public static float cpfceil(float a)
		{
			return (float)Math.Ceiling(a);
		}

		public static float cpffloor(float a)
		{
			return (float)Math.Floor(a);
		}

		public static float cpfpow(float a, float b)
		{
			return (float)Math.Pow(a, b);
		}

		public static float cpfexp(float a)
		{
			return (float)Math.Exp(a);
		}

		public static float cpfmod(float a, float b)
		{
			return a % b;
		}

		public static float cpfatan2(float a, float b)
		{
			return (float)Math.Atan2(a, b);
		}

		public static float cpfacos(float a)
		{
			return (float)Math.Acos(a);
		}

		public static float cpfsqrt(float a)
		{
			return (float)Math.Sqrt(a);
		}

		public static float cpfsin(float a)
		{
			return (float)Math.Sin(a);
		}

		public static float cpfcos(float a)
		{
			return (float)Math.Cos(a);
		}

		/// Return the max of two cpfloats.
		public static float cpfmax(float a, float b)
		{
			return (a > b) ? a : b;
		}

		/// Return the min of two floats.
		public static float cpfmin(float a, float b)
		{
			return (a < b) ? a : b;
		}

		/// Return the absolute value of a float.
		public static float cpfabs(float f)
		{
			return (f < 0) ? -f : f;
		}

		/// Clamp @c f to be between @c min and @c max.
		public static float cpfclamp(float f, float min, float max)
		{
			return cpfmin(cpfmax(f, min), max);
		}

		public static float cpclamp(float value, float min_inclusive, float max_inclusive)
		{
			if (min_inclusive > max_inclusive)
			{
				float ftmp = min_inclusive;
				min_inclusive = max_inclusive;
				max_inclusive = ftmp;
			}

			return value < min_inclusive ? min_inclusive : value < max_inclusive ? value : max_inclusive;
		}

		/// Clamp @c f to be between 0 and 1.
		public static float cpfclamp01(float f)
		{
			return cpfmax(0.0f, cpfmin(f, 1.0f));
		}

		/// Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c t percent.
		public static float cpflerp(float f1, float f2, float t)
		{
			return f1 * (1.0f - t) + f2 * t;
		}

		/// Linearly interpolate from @c f1 to @c f2 by no more than @c d.
		public static float cpflerpconst(float f1, float f2, float d)
		{
			return f1 + cpfclamp(f2 - f1, -d, d);
		}

		#endregion

		#region ASSERTS

		public static void Assert(string p2)
		{
			Error(string.Format("Assert:{0}", p2));
		}

		public static void Assert(bool p1, string p2)
		{
			if (!p1)
				Error(string.Format("Assert:{0} Value:{1}", p2, p1));
		}

		public static void AssertHard(bool p1, string p2)
		{
			if (!p1)
				Error(string.Format("AssertHard:{0} Value:{1}", p2, p1));
		}

		public static void AssertHard(string p)
		{
			Error(string.Format("AssertHard:{0} Value:{1}", p, ""));
		}

		public static void AssertSoft(bool p1, string p2)
		{
			if (!p1)
				Trace(string.Format("cpAssertSoft:{0} Value:{1}", p2, p1));
		}

		public static void AssertSpaceUnlocked(cpSpace space)
		{
			AssertSoft(!space.IsLocked, "This addition/removal cannot be done safely during a call to cpSpaceStep() or during a query. Put these calls into a post-step callback.");
		}

		public static void AssertWarn(bool p1, string p2)
		{
			if (!p1)
				Trace(string.Format("AssertWarn:{0} Value:{1}", p2, p1));
		}

		public static void AssertWarn(string p)
		{
			Trace(string.Format("AssertWarn:{0}", p));
		}

		public static void AssertWarn(bool p)
		{
			if (!p)
				Trace(string.Format("AssertWarn: ERROR DETECTED"));
		}

		public static void Trace(string message)
		{
#if DEBUG
			Debug.WriteLine(message);
#endif
		}

		public static void Error(string message)
		{
#if DEBUG
			Debug.Assert(false, message);
#endif

		}

		#endregion

		#region MOMENTS

		public static float MomentForCircle(float m, float r1, float r2, cpVect offset)
		{

			return m * (0.5f * (r1 * r1 + r2 * r2) + cpVect.cpvlengthsq(offset));
		}

		public static float AreaForCircle(float r1, float r2)
		{
			return cp.M_PI * cp.cpfabs(r1 * r1 - r2 * r2);
		}

		public static float MomentForSegment(float m, cpVect a, cpVect b, float r)
		{
			cpVect offset = cpVect.cpvlerp(a, b, 0.5f);

			// This approximates the shape as a box for rounded segments, but it's quite close.
			float length = cpVect.cpvdist(b, a) + 2.0f * r;
			return m * ((length * length + 4.0f * r * r) / 12.0f + cpVect.cpvlengthsq(offset));
		}

		public static float AreaForSegment(cpVect a, cpVect b, float r)
		{
			return r * (cp.M_PI * r + 2 * cpVect.cpvdist(a, b));
		}

		public static float MomentForPoly(float m, int count, cpVect[] verts, cpVect offset, float r)
		{
			// TODO account for radius.
			if (count == 2) return MomentForSegment(m, verts[0], verts[1], 0.0f);

			float sum1 = 0.0f;
			float sum2 = 0.0f;
			for (int i = 0; i < count; i++)
			{
				cpVect v1 = cpVect.cpvadd(verts[i], offset);
				cpVect v2 = cpVect.cpvadd(verts[(i + 1) % count], offset);

				float a = cpVect.cpvcross(v2, v1);
				float b = cpVect.cpvdot(v1, v1) + cpVect.cpvdot(v1, v2) + cpVect.cpvdot(v2, v2);

				sum1 += a * b;
				sum2 += a;
			}

			return (m * sum1) / (6.0f * sum2);
		}

		//public static float MomentForPoly(float m, cpVect[] verts, cpVect offset, float r)
		//{
		//	return MomentForPoly(m, verts.Length, verts, offset, r);
		//}

		public static float AreaForPolyOld(int count, cpVect[] verts)
		{
			float area = 0.0f;
			for (int i = 0; i < count; i++)
			{
				area += cpVect.cpvcross(verts[i], verts[(i + 1) % count]);
			}

			return -area / 2.0f;
		}

		public static float AreaForPoly(int count, cpVect[] verts, float r)
		{
			float area = 0.0f;
			float perimeter = 0.0f;

			for (int i = 0; i < count; i++)
			{
				cpVect v1 = verts[i];
				cpVect v2 = verts[(i + 1) % count];

				area += cpVect.cpvcross(v1, v2);
				perimeter += cpVect.cpvdist(v1, v2);
			}

			return r * (cp.M_PI * cpfabs(r) + perimeter) + area / 2.0f;
		}

		//public static float AreaForPoly(cpVect[] verts, float r)
		//{
		//	return AreaForPoly(verts.Length, verts, r);
		//}

		public static cpVect CentroidForPoly(int count, cpVect[] verts)
		{
			float sum = 0.0f;
			cpVect vsum = cpVect.Zero;
			for (int i = 0; i < count; i++)
			{
				cpVect v1 = verts[i];
				cpVect v2 = verts[(i + 1) % count];
				float cross = cpVect.cpvcross(v1, v2);

				sum += cross;
				vsum = cpVect.cpvadd(vsum, cpVect.cpvmult(cpVect.cpvadd(v1, v2), cross));
			}

			return cpVect.cpvmult(vsum, 1.0f / (3.0f * sum));
		}

		//public static cpVect CentroidForPoly(cpVect[] verts)
		//{
		//	return CentroidForPoly(verts.Length, verts);
		//}

		public static float MomentForBox2(float m, cpBB box)
		{
			var width = box.r - box.l;
			var height = box.t - box.b;
			var offset = cpVect.cpvmult(new cpVect(box.l + box.r, box.b + box.t), 0.5f);

			// TODO NaN when offset is 0 and m is INFINITY	
			return MomentForBox(m, width, height) + m * cpVect.cpvlengthsq(offset);
		}

		public static float MomentForBox(float m, float width, float height)
		{
			return m * (width * width + height * height) / 12f;
		}


		#endregion

		//MARK: Quick Hull

		public static void LoopIndexes(ref cpVect[] verts, int count, out int start, out int end)
		{
			start = 0;
			end = 0;

			cpVect min = verts[0];
			cpVect max = min;

			for (int i = 1; i < count; i++)
			{
				cpVect v = verts[i];

				if (v.x < min.x || (v.x == min.x && v.y < min.y))
				{
					min = verts[i];
					start = i;
				}
				else if (v.x > max.x || (v.x == max.x && v.y > max.y))
				{
					max = verts[i];
					end = i;
				}
			}
		}

		private static int QHullPartition(ref cpVect[] verts, int verts_index, int count, cpVect a, cpVect b, float tol)
		{

			if (count == 0) return 0;

			float max = 0;
			int pivot = 0;

			cpVect delta = cpVect.cpvsub(b, a);
			float valueTol = tol * cpVect.cpvlength(delta);

			int head = 0;
			for (int tail = count - 1; head <= tail; )
			{
				float value = cpVect.cpvcross(cpVect.cpvsub(verts[head + verts_index], a), delta);
				if (value > valueTol)
				{
					if (value > max)
					{
						max = value;
						pivot = head;
					}

					head++;
				}
				else
				{
					SWAP(ref verts[verts_index + head], ref verts[verts_index + tail]);
					tail--;
				}
			}

			// move the new pivot to the front if it's not already there.
			if (pivot != 0) SWAP(ref verts[verts_index], ref verts[verts_index + pivot]);
			return head;
		}

		public static int QHullReduce(float tol, cpVect[] verts, int verts_index, int count, cpVect a, cpVect pivot, cpVect b, ref cpVect[] result, int result_index)
		{

			if (count < 0)
			{
				return 0;
			}
			else if (count == 0)
			{
				result[result_index] = pivot;
				return 1;
			}
			else
			{
				int left_count = QHullPartition(ref verts, verts_index, count, a, pivot, tol);
				int index = QHullReduce(tol, verts, verts_index + 1, left_count - 1, a, verts[verts_index], pivot, ref result, result_index);

				result[(index++) + result_index] = pivot;



				int right_count = QHullPartition(ref verts, verts_index + left_count, count - left_count, pivot, b, tol);


				int new_index = verts_index + left_count;

				if (new_index > verts.Length - 1)
					new_index = verts.Length - 1;


				return index + QHullReduce(tol, verts, verts_index + left_count + 1, right_count - 1, pivot, verts[new_index], b, ref result, result_index + index);
			}
		}


		// QuickHull seemed like a neat algorithm, and efficient-ish for large input sets.
		// My implementation performs an in place reduction using the result array as scratch space.
		public static int ConvexHull(int count, cpVect[] verts, ref cpVect[] result, int? first, float tol)
		{

			if (verts != result)
			{

				for (int i = 0; i < verts.Length; i++)
				{
					result[i] = new cpVect(verts[i]);
				}


				// Copy the line vertexes into the empty part of the result polyline to use as a scratch buffer.
				//memcpy(result, verts, count * sizeof(cpVect));
				//TODO: NOT IMPLEMENTED
			}

			// Degenerate case, all points are the same.
			int start, end;

			LoopIndexes(ref verts, count, out start, out end);
			if (start == end)
			{
				if (first.HasValue)
					first = 0;

				return 1;
			}

			SWAP(ref result[0], ref result[start]);
			SWAP(ref result[1], ref result[end == 0 ? start : end]);

			cpVect a = result[0];
			cpVect b = result[1];

			if (first.HasValue)
				first = start;



			//Array.Resize(ref result, result.Length + 1);

			return QHullReduce(tol, result, 0 + 2, count - 2, a, b, a, ref result, 0 + 1) + 1;

		}

		public static void SWAP(ref cpVect first, ref cpVect second)
		{
			var tmp = first;
			first = second;
			second = tmp;
		}





		/// ///////////////////////////////////////////////////////////////////



		public static float bbTreeMergedArea2(Node node, float l, float b, float r, float t)
		{
			return (cp.cpfmax(node.bb.r, r) - cp.cpfmin(node.bb.l, l)) * (cp.cpfmax(node.bb.t, t) - cp.cpfmin(node.bb.b, b));
		}

		public static void nodeRender(Node node, int depth)
		{
			if (!node.isLeaf && depth <= 10)
			{
				nodeRender(node.A, depth + 1);
				nodeRender(node.B, depth + 1);
			}

			var str = "";
			for (var i = 0; i < depth; i++)
			{
				str += " ";
			}

			Trace(str + node.bb.b + " " + node.bb.t);
		}



		public static cpConstraint filterConstraints(cpConstraint node, cpBody body, cpConstraint filter)
		{
			if (node == filter)
			{
				return node.Next(body);
			}
			else if (node.a == body)
			{
				node.next_a = filterConstraints(node.next_a, body, filter);
			}
			else
			{
				node.next_b = filterConstraints(node.next_b, body, filter);
			}

			return node;
		}

		public static cpBody ComponentRoot(cpBody body)
		{
			return (body != null ? body.nodeRoot : null);
		}

		public static void ComponentActivate(cpBody root)
		{
			if (root == null || !root.IsSleeping()) return;
			cp.AssertHard(!root.IsRogue(), "Internal Error: componentActivate() called on a rogue body.");

			var space = root.space;
			cpBody body = root;
			while (body != null)
			{
				var next = body.nodeNext;

				body.nodeIdleTime = 0;
				body.nodeRoot = null;
				body.nodeNext = null;

				space.ActivateBody(body);

				body = next;
			}

			space.sleepingComponents.Remove(root);
		}



		public static void componentAdd(cpBody root, cpBody body)
		{
			body.nodeRoot = root;

			if (body != root)
			{
				body.nodeNext = root.nodeNext;
				root.nodeNext = body;
			}
		}



		public static cpCollisionHandler defaultCollisionHandler = new cpCollisionHandler();


		//// **** All Important cpSpaceStep() Function
		/// Returns true if @c a and @c b intersect.

		public static bool bbIntersects(cpBB a, cpBB b)
		{
			return (a.l <= b.r && b.l <= a.r && a.b <= b.t && b.b <= a.t); ;
		}

		public static bool bbIntersects2(cpBB bb, float l, float b, float r, float t)
		{
			return (bb.l <= r && l <= bb.r && bb.b <= t && b <= bb.t);
		}

		public static float bbProximity(Node a, Node b)
		{
			return cp.cpfabs(a.bb.l + a.bb.r - b.bb.l - b.bb.r) + cp.cpfabs(a.bb.b + a.bb.t - b.bb.b - b.bb.t);
		}

		public static float bbTreeMergedArea(Node a, Node b)
		{
			return (cp.cpfmax(a.bb.r, b.bb.r) - cp.cpfmin(a.bb.l, b.bb.l)) * (cp.cpfmax(a.bb.t, b.bb.t) - cp.cpfmin(a.bb.b, b.bb.b));
		}

		public static bool bbTreeIntersectsNode(Node a, Node b)
		{
			return (a.bb.l <= b.bb.r && b.bb.l <= a.bb.r && a.bb.b <= b.bb.t && b.bb.b <= a.bb.t);
		}

		/// Check that a set of vertexes is convex and has a clockwise winding.
		public static bool polyValidate(float[] verts)
		{
			var len = verts.Length;
			for (var i = 0; i < len; i += 2)
			{
				var ax = verts[i];
				var ay = verts[i + 1];
				var bx = verts[(i + 2) % len];
				var by = verts[(i + 3) % len];
				var cx = verts[(i + 4) % len];
				var cy = verts[(i + 5) % len];

				//if(vcross(vsub(b, a), vsub(c, b)) > 0){
				if (cpVect.cpvcross2(bx - ax, by - ay, cx - bx, cy - by) > 0)
				{
					return false;
				}
			}

			return true;
		}

		internal static cpVect closestPointOnSegment2(float px, float py, float ax, float ay, float bx, float by)
		{
			var deltax = ax - bx;
			var deltay = ay - by;
			var t = cpfclamp01(cpVect.cpvdot2(deltax, deltay, px - bx, py - by) / cpVect.vlengthsq2(deltax, deltay));
			return new cpVect(bx + deltax * t, by + deltay * t);
		}

		public static cpVect closestPointOnSegment(cpVect p, cpVect a, cpVect b)
		{
			var delta = cpVect.cpvsub(a, b);
			var t = cp.cpfclamp01(cpVect.cpvdot(delta, cpVect.cpvsub(p, b)) / cpVect.cpvlengthsq(delta));
			return cpVect.cpvadd(b, cpVect.cpvmult(delta, t));
		}


		public static int GRABABLE_MASK_BIT { get { return (1 << 31); } }

		public static int NOT_GRABABLE_MASK { get { return ~GRABABLE_MASK_BIT; } }

		public static void SWAP(float[] arr, int idx1, int idx2)
		{
			var tmp = arr[idx1 * 2];
			arr[idx1 * 2] = arr[idx2 * 2];
			arr[idx2 * 2] = tmp;

			tmp = arr[idx1 * 2 + 1];
			arr[idx1 * 2 + 1] = arr[idx2 * 2 + 1];
			arr[idx2 * 2 + 1] = tmp;
		}

		public static List<cpColor> _styles;

		public static float[] colorRanges = new float[] {
			//0.2f,0.4f,0.5f,0.7f,0.8f,1f
						178,1,255
		};

		public static List<cpColor> styles
		{
			get
			{

				if (_styles == null)
				{

					var rnd = new Random(DateTime.Now.Millisecond);

					_styles = new List<cpColor>();

					for (var i = 200; i >= 0; i -= 10)
					{
						_styles.Add(new cpColor(rnd.Next(100, 255), rnd.Next(160, 255), rnd.Next(160, 255)));
					}

				}

				return _styles;

			}
			set
			{
				_styles = value;
			}
		}

		public static cpVect canvas2point(float x, float y, float scale)
		{
			return new cpVect(x / scale, 480f - y / scale);
		}

		public static cpColor GetShapeColor(cpShape shape)
		{

			if (shape.sensor)
				return new cpColor(128, 128, 128);
			else
			{

				if (shape.body.IsSleeping())
				{
					return new cpColor(125, 125, 125);
				}
				else if (shape.body.nodeIdleTime > shape.space.sleepTimeThreshold)
				{
					return new cpColor(170, 170, 170);
				}
				else if (shape.body.bodyType == cpBodyType.STATIC)
				{
					return new cpColor(75, 75, 75);
				}
				else
				{
					return styles[(int)shape.hashid % styles.Count];
				}
			}
		}

		public static cpVect point2canvas(cpVect point, float scale)
		{
			return new cpVect(point.x * scale, (480 - point.y) * scale);
		}

		public static float last_MSA_min = 0;

		internal static cpBB bbNewForCircle(cpVect p, float r)
		{
			return new cpBB(
			p.x - r,
			p.y - r,
			p.x + r,
			p.y + r
		);
		}

		public static float[] ConvertTofloatArray(List<cpVect> vec)
		{
			float[] dev = new float[vec.Count];
			int sum = 0;

			for (int i = 0; i < vec.Count; i++)
			{
				dev[sum] = vec[i].x;
				sum++;
				dev[sum] = vec[i].y;
				sum++;
			}
			return dev;
		}

		public static cpVect frand_unit_circle()
		{
			cpVect v = new cpVect(frand() * 2.0f - 1.0f, frand() * 2.0f - 1.0f);
			return (cpVect.cpvlengthsq(v) < 1.0f ? v : frand_unit_circle());
		}

		public static float frand()
		{

			//float tmp = ((rand.Nextfloat() *  f) / ((float) (/*(uint)~0*/ 0xFFFFFFFF /*or is it -1 :P */)));
			//return tmp < 0 ? (-tmp) : tmp;
			return RandomHelper.frand(1);
		}


		public static int numShapes { get; set; }


		public static void RecenterPoly(int numVerts, cpVect[] verts)
		{
			cpVect centroid = cp.CentroidForPoly(numVerts, verts);

			for (int i = 0; i < numVerts; i++)
			{
				verts[i] = cpVect.cpvsub(verts[i], centroid);
			}
		}
	}

}		//public static float[] convexHull(float[] verts, float[] result, float tolerance)
//{
//	if (result != null)
//	{
//		// Copy the line vertexes into the empty part of the result polyline to use as a scratch buffer.
//		for (var i = 0; i < verts.Length; i++)
//		{
//			result[i] = verts[i];
//		}
//	}
//	else
//	{
//		// If a result array was not specified, reduce the input instead.
//		result = verts;
//	}


//	// Degenerate case, all points are the same.
//	int[] indexes = LoopIndexes(verts);
//	int start = indexes[0], end = indexes[1];

//	int position;
//	float[] dev;

//	if (start == end)
//	{
//		//if(first) (*first) = 0;
//		position = 2;
//		dev = new float[position];
//		for (int i = 0; i < position; i++)
//			dev[i] = result[i];
//		return dev;
//	}

//	SWAP(result, 0, start);
//	SWAP(result, 1, end == 0 ? start : end);

//	var a = new cpVect(result[0], result[1]);
//	var b = new cpVect(result[2], result[3]);

//	var count = verts.Length >> 1;
//	//if(first) (*first) = start;
//	var resultCount = QHullReduce(tolerance, result, 2, count - 2, a, b, a, 1) + 1;

//	position = resultCount * 2;

//	dev = new float[position];
//	for (int i = 0; i < position; i++)
//		dev[i] = result[i];

//	assertSoft(polyValidate(result),
//		"Internal error: cpConvexHull() and cpPolyValidate() did not agree." +
//		"Please report this error with as much info as you can.");
//	return dev;
//}}
