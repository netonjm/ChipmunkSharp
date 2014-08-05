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
using ChipmunkSharp;
using System.Linq;
// // typedef int (*collisionFunc)(cpShape , cpShape , cpContact );
using System;
using System.Collections.Generic;

namespace ChipmunkSharp
{

	//public delegate int CollisionFunc(cpShape a, cpShape b, int id, List<ContactPoint> arr);

	public class cpCollision
	{
		public static ulong PolySupportPointIndex(int count, cpSplittingPlane[] planes, cpVect n)
		{
			float max = -cp.Infinity;
			int index = 0;
			for (int i = 0; i < count; i++)
			{
				cpVect v = planes[i].v0;
				float d = cpVect.cpvdot(v, n);
				if (d > max)
				{
					max = d;
					index = i;
				}
			}

			return (ulong)index;
		}

		// Closest points on the surface of two shapes.

		#region CONSTANTS

		public const int DRAW_ALL = 0;
		public const int ENABLE_CACHING = 1;

		public const int DRAW_GJK = (0 | DRAW_ALL);
		public const int DRAW_EPA = (0 | DRAW_ALL);
		public const int DRAW_CLOSEST = (0 | DRAW_ALL);
		public const int DRAW_CLIP = (0 | DRAW_ALL);

		public const int MAX_GJK_ITERATIONS = 30;
		public const int MAX_EPA_ITERATIONS = 30;
		public const int WARN_GJK_ITERATIONS = 20;
		public const int WARN_EPA_ITERATIONS = 20;

		#endregion

		public struct SupportPoint
		{
			public cpVect p;
			public ulong id;

			public SupportPoint(cpVect p, ulong id)
			{
				this.p = p;
				this.id = id;
			}

			public static SupportPoint CircleSupportPoint(cpCircleShape circle, cpVect n)
			{
				return new SupportPoint(circle.tc, 0);
			}

			public static SupportPoint SegmentSupportPoint(cpSegmentShape seg, cpVect n)
			{

				if (cpVect.cpvdot(seg.ta, n) > cpVect.cpvdot(seg.tb, n))
					return new SupportPoint(seg.ta, 0);
				else
					return new SupportPoint(seg.tb, 1);
			}

			public static SupportPoint PolySupportPoint(cpPolyShape poly, cpVect n)
			{
				ulong i = PolySupportPointIndex(poly.Count, poly.planes, n);
				return new SupportPoint(poly.planes[i].v0, i);
			}

		};

		public static void InfoPushContact(ref cpCollisionInfo info, cpVect p1, cpVect p2, ulong hash)
		{
			cp.AssertSoft(info.count <= cpArbiter.CP_MAX_CONTACTS_PER_ARBITER, "Internal error: Tried to push too many contacts.");
			info.arr.Add(new cpContact(p1, p2, hash));
			//info.arr[info.count] = ;
			info.count++;// = count++;
		}

		//MARK: Support Points and Edges:
		public static int PolySupportPointIndex(List<cpSplittingPlane> planes, cpVect n)
		{
			float max = -cp.Infinity;
			int index = 0;

			for (int i = 0; i < planes.Count; i++)
			{
				cpVect v = planes[i].v0;
				float d = cpVect.cpvdot(v, n); // cpvdot(v, n);
				if (d > max)
				{
					max = d;
					index = i;
				}
			}

			return index;
		}

		// Calculate the maximal point on the minkowski difference of two shapes along a particular axis.
		public struct MinkowskiPoint
		{

			public cpVect a, b;
			public cpVect ab;
			public ulong id;

			public MinkowskiPoint(cpVect a, cpVect b, cpVect ab, ulong id)
			{
				this.a = a;
				this.b = b;
				this.ab = ab;
				this.id = id;
			}

			public static MinkowskiPoint MinkowskiPointNew(SupportPoint a, SupportPoint b)
			{
				var point = new MinkowskiPoint(a.p, b.p,
					cpVect.cpvsub(b.p, a.p),
					((a.id & 0xFF) << 8 | (b.id & 0xFF)));
				return point;
			}

			public static MinkowskiPoint Support(ref SupportContext ctx, cpVect n)
			{
				SupportPoint a = ctx.func1(ctx.shape1, cpVect.cpvneg(n));
				SupportPoint b = ctx.func2(ctx.shape2, n);
				return MinkowskiPointNew(a, b);
			}

		};

		public struct SupportContext
		{
			public cpShape shape1, shape2;
			public Func<cpShape, cpVect, SupportPoint> func1, func2;

			public SupportContext(cpShape shape1, cpShape shape2, Func<cpShape, cpVect, SupportPoint> func1, Func<cpShape, cpVect, SupportPoint> func2)
			{
				this.shape1 = shape1;
				this.shape2 = shape2;
				this.func1 = func1;
				this.func2 = func2;

			}

		};

		public struct EdgePoint
		{
			public cpVect p;
			public ulong hash;

			public EdgePoint(cpVect p, ulong hash)
			{

				this.p = p;
				this.hash = hash;
			}
		};

		// Support edges are the edges of a polygon or segment shape that are in contact.
		public struct Edge
		{
			public EdgePoint a, b;
			public float r;
			public cpVect n;

			public Edge(EdgePoint a, EdgePoint b, float r, cpVect n)
			{
				this.a = a;
				this.b = b;
				this.r = r;
				this.n = n;
			}

			public static Edge EdgeNew(cpVect va, cpVect vb, ulong ha, ulong hb, float r)
			{
				return new Edge(
					new EdgePoint(va, ha),
					new EdgePoint(vb, hb),
					r,
					cpVect.cpvnormalize(cpVect.cpvperp(cpVect.cpvsub(vb, va))));
			}

			public static Edge SupportEdgeForPoly(cpPolyShape poly, cpVect n)
			{
				ulong count = (ulong)poly.Count;

				ulong i1 = cpCollision.PolySupportPointIndex(poly.Count, poly.planes, n);

				// TODO get rid of mod eventually, very expensive on ARM
				ulong i0 = (ulong)((i1 - 1 + count) % count);
				ulong i2 = (ulong)((i1 + 1) % count);

				cpSplittingPlane[] planes = poly.planes;
				ulong hashid = poly.hashid;

				if (cpVect.cpvdot(n, planes[i1].n) > cpVect.cpvdot(n, planes[i2].n))
				{
					Edge edge = new Edge(

					 new EdgePoint(planes[i0].v0, cp.CP_HASH_PAIR(hashid, i0)),
					 new EdgePoint(planes[i1].v0, cp.CP_HASH_PAIR(hashid, i1)),

					 poly.r, poly.planes[i1].n);

					return edge;
				}
				else
				{

					Edge edge = new Edge(

					new EdgePoint(planes[i1].v0, cp.CP_HASH_PAIR(hashid, i1)),
					new EdgePoint(planes[i2].v0, cp.CP_HASH_PAIR(hashid, i2)),
					poly.r, poly.planes[i2].n);

					return edge;
				}
			}

			public static Edge SupportEdgeForSegment(cpSegmentShape seg, cpVect n)
			{

				ulong hashid = seg.hashid;

				Edge edge;

				if (cpVect.cpvdot(seg.tn, n) > 0.0f)
				{
					edge = new Edge(
					   new EdgePoint(seg.ta, cp.CP_HASH_PAIR(seg.hashid, 0)),
						new EdgePoint(seg.tb, cp.CP_HASH_PAIR(seg.hashid, 1)),
						seg.r, seg.tn);

				}
				else
				{
					edge = new Edge(
			new EdgePoint(seg.tb, cp.CP_HASH_PAIR(seg.hashid, 1)),
			 new EdgePoint(seg.ta, cp.CP_HASH_PAIR(seg.hashid, 0)),
			 seg.r, cpVect.cpvneg(seg.tn)
			 );


				}
				return edge;
			}

		};

		public static float ClosestT(cpVect a, cpVect b)
		{
			cpVect delta = cpVect.cpvsub(b, a);
			return -cp.cpfclamp(cpVect.cpvdot(delta, cpVect.cpvadd(a, b)) / cpVect.cpvlengthsq(delta), -1.0f, 1.0f);
		}

		public static cpVect LerpT(cpVect a, cpVect b, float t)
		{
			float ht = 0.5f * t;
			return cpVect.cpvadd(cpVect.cpvmult(a, 0.5f - ht), cpVect.cpvmult(b, 0.5f + ht));
		}

		// Closest points on the surface of two shapes.
		public struct ClosestPoints
		{
			// Surface points in absolute coordinates.
			public cpVect a, b;
			// Minimum separating axis of the two shapes.
			public cpVect n;
			// Signed distance between the points.
			public float d;
			// Concatenation of the id's of the minkoski points.
			public ulong id;


			public ClosestPoints(cpVect a, cpVect b, cpVect n, float d, ulong id)
			{
				this.a = a;
				this.b = b;
				this.n = n;
				this.d = d;
				this.id = id;
			}

			// Calculate the closest points on two shapes given the closest edge on their minkowski difference to (0, 0)
			public static ClosestPoints ClosestPointsNew(MinkowskiPoint v0, MinkowskiPoint v1)
			{
				// Find the closest p(t) on the minkowski difference to (0, 0)
				float t = cpCollision.ClosestT(v0.ab, v1.ab);
				cpVect p = cpCollision.LerpT(v0.ab, v1.ab, t);

				// Interpolate the original support points using the same 't' value as above.
				// This gives you the closest surface points in absolute coordinates. NEAT!
				cpVect pa = cpCollision.LerpT(v0.a, v1.a, t);
				cpVect pb = cpCollision.LerpT(v0.b, v1.b, t);

				ulong id = ((v0.id & 0xFFFF) << 16 | (v1.id & 0xFFFF));

				// First try calculating the MSA from the minkowski difference edge.
				// This gives us a nice, accurate MSA when the surfaces are close together.
				cpVect delta = cpVect.cpvsub(v1.ab, v0.ab);
				cpVect n = cpVect.cpvnormalize(cpVect.cpvrperp(delta));
				float d = cpVect.cpvdot(n, p);

				if (d <= 0.0f || (-1.0f < t && t < 1.0f))
				{
					// If the shapes are overlapping, or we have a regular vertex/edge collision, we are done.
					ClosestPoints points = new ClosestPoints(pa, pb, n, d, id);
					return points;
				}
				else
				{
					// Vertex/vertex collisions need special treatment since the MSA won't be shared with an axis of the minkowski difference.
					float d2 = cpVect.cpvlength(p);
					cpVect n2 = cpVect.cpvmult(p, 1.0f / (d2 + float.MinValue));

					ClosestPoints points = new ClosestPoints(pa, pb, n2, d2, (ulong)id);
					return points;
				}

			}
		}


		//MARK: EPA Functions
		public static float ClosestDist(cpVect v0, cpVect v1)
		{
			return cpVect.cpvlengthsq(LerpT(v0, v1, ClosestT(v0, v1)));
		}

		public static bool CheckArea(cpVect v1, cpVect v2)
		{
			return (v1.x * v2.y) > (v1.y * v2.x);
		}

		// Recursive implementation of the EPA loop.
		// Each recursion adds a point to the convex hull until it's known that we have the closest point on the surface.
		public static ClosestPoints EPARecurse(ref SupportContext ctx, int count, MinkowskiPoint[] hull, int iteration)
		{
			int mini = 0;
			float minDist = cp.Infinity;

			//int count = hull.Length;

			// TODO: precalculate this when building the hull and save a step.
			for (int j = 0, i = count - 1; j < count; i = j, j++)
			{
				float d = ClosestDist(hull[i].ab, hull[j].ab);
				if (d < minDist)
				{
					minDist = d;
					mini = i;
				}
			}

			MinkowskiPoint v0 = hull[mini];
			MinkowskiPoint v1 = hull[(mini + 1) % count];
			cp.AssertSoft(!cpVect.cpveql(v0.ab, v1.ab), string.Format("Internal Error: EPA vertexes are the same ({0} and {1})", mini, (mini + 1) % count));

			MinkowskiPoint p = MinkowskiPoint.Support(ref ctx,
				cpVect.cpvperp(cpVect.cpvsub(v1.ab, v0.ab)));

#if DRAW_EPA
	cpVect verts[count];
	for(int i=0; i<count; i++) verts[i] = hull[i].ab;
	
	ChipmunkDebugDrawPolygon(count, verts, RGBAColor(1, 1, 0, 1), RGBAColor(1, 1, 0, 0.25));
	ChipmunkDebugDrawSegment(v0.ab, v1.ab, RGBAColor(1, 0, 0, 1));
	
	ChipmunkDebugDrawPoints(5, 1, (cpVect[]){p.ab}, RGBAColor(1, 1, 1, 1));
#endif

			if (CheckArea(
				cpVect.cpvsub(v1.ab, v0.ab),
				cpVect.cpvadd(cpVect.cpvsub(p.ab, v0.ab), cpVect.cpvsub(p.ab, v1.ab)))
				&& iteration < MAX_EPA_ITERATIONS)
			{

				MinkowskiPoint[] hull2 = new MinkowskiPoint[count + 1];
				int count2 = 1;
				hull2[0] = p;

				for (int i = 0; i < count; i++)
				{
					int index = (mini + 1 + i) % count;

					cpVect h0 = hull2[count2 - 1].ab;
					cpVect h1 = hull[index].ab;
					cpVect h2 = (i + 1 < count ? hull[(index + 1) % count] : p).ab;

					if (CheckArea(cpVect.cpvsub(h2, h0), cpVect.cpvadd(cpVect.cpvsub(h1, h0), cpVect.cpvsub(h1, h2))))
					{
						hull2[count2] = hull[index];
						count2++;
					}
				}

				return EPARecurse(ref ctx, count2, hull2, iteration + 1);
			}
			else
			{
				cp.AssertWarn(iteration < WARN_EPA_ITERATIONS, string.Format("High EPA iterations: {0}", iteration));
				return ClosestPoints.ClosestPointsNew(v0, v1);
			}
		}

		// Find the closest points on the surface of two overlapping shapes using the EPA algorithm.
		// EPA is called from GJK when two shapes overlap.
		// This is moderately expensive step! Avoid it by adding radii to your shapes so their inner polygons won't overlap.
		public static ClosestPoints EPA(ref SupportContext ctx, ref MinkowskiPoint v0, ref MinkowskiPoint v1, ref MinkowskiPoint v2)
		{
			// TODO: allocate a NxM array here and do an in place convex hull reduction in EPARecurse
			MinkowskiPoint[] hull = new MinkowskiPoint[] { v0, v1, v2 };
			return EPARecurse(ref ctx, 3, hull, 1);

		}


		//MARK: GJK Functions.
		public static ClosestPoints GJKRecurse(ref SupportContext ctx, ref MinkowskiPoint v0, ref MinkowskiPoint v1, int iteration)
		{

			if (iteration > MAX_GJK_ITERATIONS)
			{
				cp.AssertWarn(iteration < WARN_GJK_ITERATIONS, string.Format("High GJK iterations: {0}", iteration));
				return ClosestPoints.ClosestPointsNew(v0, v1);
			}

			cpVect delta = cpVect.cpvsub(v1.ab, v0.ab);
			if (CheckArea(delta, cpVect.cpvadd(v0.ab, v1.ab)))
			{
				// Origin is behind axis. Flip and try again.
				return GJKRecurse(ref ctx, ref v1, ref v0, iteration);
			}
			else
			{
				float t = ClosestT(v0.ab, v1.ab);
				cpVect n = (-1.0f < t && t < 1.0f ? cpVect.cpvperp(delta) : cpVect.cpvneg(LerpT(v0.ab, v1.ab, t)));
				MinkowskiPoint p = MinkowskiPoint.Support(ref ctx, n);

#if DRAW_GJK
		ChipmunkDebugDrawSegment(v0.ab, v1.ab, RGBAColor(1, 1, 1, 1));
		cpVect c = cpvlerp(v0.ab, v1.ab, 0.5);
		ChipmunkDebugDrawSegment(c, cpvadd(c, cpvmult(cpvnormalize(n), 5.0)), RGBAColor(1, 0, 0, 1));
		
		ChipmunkDebugDrawPoints(5.0, 1, &p.ab, RGBAColor(1, 1, 1, 1));
#endif
				if (
		CheckArea(cpVect.cpvsub(v1.ab, p.ab), cpVect.cpvadd(v1.ab, p.ab)) &&
		CheckArea(cpVect.cpvadd(v0.ab, p.ab), cpVect.cpvsub(v0.ab, p.ab))
	)
				{
					// The triangle v0, p, v1 contains the origin. Use EPA to find the MSA.
					cp.AssertWarn(iteration < WARN_GJK_ITERATIONS, string.Format("High GJK->EPA iterations: {0}", iteration));
					return EPA(ref ctx, ref  v0, ref  p, ref  v1);
				}
				else
				{
					if (cpVect.cpvdot(p.ab, n) <= cp.cpfmax(cpVect.cpvdot(v0.ab, n), cpVect.cpvdot(v1.ab, n)))
					{
						// The edge v0, v1 that we already have is the closest to (0, 0) since p was not closer.
						cp.AssertWarn(iteration < WARN_GJK_ITERATIONS, string.Format("High GJK iterations: {0}", iteration));
						return ClosestPoints.ClosestPointsNew(v0, v1);
					}
					else
					{
						// p was closer to the origin than our existing edge.
						// Need to figure out which existing point to drop.
						if (ClosestDist(v0.ab, p.ab) < ClosestDist(p.ab, v1.ab))
						{
							return GJKRecurse(ref ctx, ref  v0, ref  p, iteration + 1);
						}
						else
						{
							return GJKRecurse(ref ctx, ref  p, ref  v1, iteration + 1);
						}
					}
				}


			}
		}


		public static SupportPoint ShapePoint(cpShape shape, ulong i)
		{
			switch (shape.shapeType)
			{
				case cpShapeType.Circle:
					return new SupportPoint((shape as cpCircleShape).tc, 0);
				case cpShapeType.Segment:
					cpSegmentShape seg = shape as cpSegmentShape;
					return new SupportPoint(i == 0 ? seg.ta : seg.tb, i);
				case cpShapeType.Polygon:
					cpPolyShape poly = (cpPolyShape)shape;
					// Poly shapes may change vertex count.
					ulong index = (i < (ulong)poly.Count ? i : 0);
					return new SupportPoint(poly.planes[index].v0, index);
				default:
					return new SupportPoint(cpVect.Zero, 0);
			}
		}


		public static ClosestPoints GJK(ref SupportContext ctx, ref ulong id)
		{
			MinkowskiPoint v0, v1;
			if (id > 0 && ENABLE_CACHING == 0)
			{
				v0 = MinkowskiPoint.MinkowskiPointNew(
					ShapePoint(ctx.shape1, (id >> 24) & 0xFF),
					ShapePoint(ctx.shape2, (id >> 16) & 0xFF)
					);

				v1 = MinkowskiPoint.MinkowskiPointNew(
					ShapePoint(ctx.shape1, (id >> 8) & 0xFF),
					ShapePoint(ctx.shape2, (id) & 0xFF)
					);
			}
			else
			{
				cpVect axis = cpVect.cpvperp(
					cpVect.cpvsub(
					cpBB.Center(ctx.shape1.bb),
					cpBB.Center(ctx.shape2.bb)));

				v0 = MinkowskiPoint.Support(ref ctx, axis);
				v1 = MinkowskiPoint.Support(ref ctx, cpVect.cpvneg(axis));
			}

			ClosestPoints points = GJKRecurse(ref ctx, ref  v0, ref  v1, 1);
			id = points.id;
			return points;
		}

		//MARK: Contact Clipping

		public static void ContactPoints(Edge e1, Edge e2, ClosestPoints points, ref cpCollisionInfo info)
		{
			float mindist = e1.r + e2.r;
			if (points.d <= mindist)
			{

				info.n = new cpVect(points.n);

				cpVect n = new cpVect(points.n);

				// Distances along the axis parallel to n
				float d_e1_a = cpVect.cpvcross(e1.a.p, n);
				float d_e1_b = cpVect.cpvcross(e1.b.p, n);
				float d_e2_a = cpVect.cpvcross(e2.a.p, n);
				float d_e2_b = cpVect.cpvcross(e2.b.p, n);

				float e1_denom = 1.0f / (d_e1_b - d_e1_a);
				float e2_denom = 1.0f / (d_e2_b - d_e2_a);

				// Project the endpoints of the two edges onto the opposing edge, clamping them as necessary.
				// Compare the projected points to the collision normal to see if the shapes overlap there.
				{
					cpVect p1 = cpVect.cpvadd(cpVect.cpvmult(n, e1.r), cpVect.cpvlerp(e1.a.p, e1.b.p, cp.cpfclamp01((d_e2_b - d_e1_a) * e1_denom)));
					cpVect p2 = cpVect.cpvadd(cpVect.cpvmult(n, -e2.r), cpVect.cpvlerp(e2.a.p, e2.b.p, cp.cpfclamp01((d_e1_a - d_e2_a) * e2_denom)));
					float dist = cpVect.cpvdot(cpVect.cpvsub(p2, p1), n);
					if (dist <= 0.0f)
					{
						ulong hash_1a2b = cp.CP_HASH_PAIR(e1.a.hash, e2.b.hash);
						InfoPushContact(ref info, p1, p2, hash_1a2b);
					}
				}
				{
					cpVect p1 = cpVect.cpvadd(cpVect.cpvmult(n, e1.r), cpVect.cpvlerp(e1.a.p, e1.b.p, cp.cpfclamp01((d_e2_a - d_e1_a) * e1_denom)));
					cpVect p2 = cpVect.cpvadd(cpVect.cpvmult(n, -e2.r), cpVect.cpvlerp(e2.a.p, e2.b.p, cp.cpfclamp01((d_e1_b - d_e2_a) * e2_denom)));
					float dist = cpVect.cpvdot(cpVect.cpvsub(p2, p1), n);
					if (dist <= 0.0f)
					{
						ulong hash_1b2a = cp.CP_HASH_PAIR(e1.b.hash, e2.a.hash);
						InfoPushContact(ref info, p1, p2, hash_1b2a);
					}
				}
			}
		}

		//MARK: Collision Functions

		// Collide circle shapes.


		public static void CircleToCircle(cpShape cir1, cpShape cir2, ref cpCollisionInfo info)
		{
			cpCircleShape c1 = (cpCircleShape)cir1;
			cpCircleShape c2 = (cpCircleShape)cir2;

			float mindist = c1.r + c2.r;
			cpVect delta = cpVect.cpvsub(c2.tc, c1.tc);
			float distsq = cpVect.cpvlengthsq(delta);

			if (distsq < mindist * mindist)
			{
				float dist = cp.cpfsqrt(distsq);
				cpVect n = info.n = (dist > 0.0f ? cpVect.cpvmult(delta, 1.0f / dist) : cpVect.cpv(1.0f, 0.0f));
				InfoPushContact(ref info, cpVect.cpvadd(c1.tc, cpVect.cpvmult(n, c1.r)), cpVect.cpvadd(c2.tc, cpVect.cpvmult(n, -c2.r)), 0);
			}
		}

		public static void CircleToSegment(cpShape circle1, cpShape segment2, ref cpCollisionInfo info)
		{
			cpCircleShape circle = (cpCircleShape)circle1;
			cpSegmentShape segment = (cpSegmentShape)segment2;

			cpVect seg_a = segment.ta;
			cpVect seg_b = segment.tb;
			cpVect center = circle.tc;

			// Find the closest point on the segment to the circle.
			cpVect seg_delta = cpVect.cpvsub(seg_b, seg_a);
			float closest_t = cp.cpfclamp01(cpVect.cpvdot(seg_delta, cpVect.cpvsub(center, seg_a)) / cpVect.cpvlengthsq(seg_delta));
			cpVect closest = cpVect.cpvadd(seg_a, cpVect.cpvmult(seg_delta, closest_t));

			// Compare the radii of the two shapes to see if they are colliding.
			float mindist = circle.r + segment.r;
			cpVect delta = cpVect.cpvsub(closest, center);
			float distsq = cpVect.cpvlengthsq(delta);
			if (distsq < mindist * mindist)
			{
				float dist = cp.cpfsqrt(distsq);
				// Handle coincident shapes as gracefully as possible.
				cpVect n = info.n = (dist > 0 ? cpVect.cpvmult(delta, 1.0f / dist) : segment.tn);

				// Reject endcap collisions if tangents are provided.
				cpVect rot = segment.body.GetRotation();
				if (
					(closest_t != 0.0f || cpVect.cpvdot(n, cpVect.cpvrotate(segment.a_tangent, rot)) >= 0.0) &&
					(closest_t != 1.0f || cpVect.cpvdot(n, cpVect.cpvrotate(segment.b_tangent, rot)) >= 0.0)
				)
				{
					InfoPushContact(ref info, cpVect.cpvadd(center, cpVect.cpvmult(n, circle.r)), cpVect.cpvadd(closest, cpVect.cpvmult(n, -segment.r)), 0);
				}
			}
		}

		public static void SegmentToSegment(cpShape segment1, cpShape segment2, ref cpCollisionInfo info)
		{

			cpSegmentShape seg1 = (cpSegmentShape)segment1;
			cpSegmentShape seg2 = (cpSegmentShape)segment2;

			SupportContext context = new SupportContext(seg1, seg2, (s0, s1) => SupportPoint.SegmentSupportPoint(s0 as cpSegmentShape, s1), (s0, s1) => SupportPoint.SegmentSupportPoint(s0 as cpSegmentShape, s1));
			ClosestPoints points = GJK(ref context, ref info.id);

			cpVect n = points.n;
			cpVect rot1 = seg1.body.GetRotation();
			cpVect rot2 = seg2.body.GetRotation();

			// If the closest points are nearer than the sum of the radii...
			if (
				points.d <= (seg1.r + seg2.r) &&
				(
				// Reject endcap collisions if tangents are provided.
					(!cpVect.cpveql(points.a, seg1.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg1.a_tangent, rot1)) <= 0.0) &&
					(!cpVect.cpveql(points.a, seg1.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg1.b_tangent, rot1)) <= 0.0) &&
					(!cpVect.cpveql(points.b, seg2.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg2.a_tangent, rot2)) >= 0.0) &&
					(!cpVect.cpveql(points.b, seg2.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg2.b_tangent, rot2)) >= 0.0)
				)
			)
			{
				ContactPoints(Edge.SupportEdgeForSegment(seg1, n), Edge.SupportEdgeForSegment(seg2, cpVect.cpvneg(n)), points, ref info);
			}
		}

		public static void PolyToPoly(cpShape polygon1, cpShape polygon2, ref cpCollisionInfo info)
		{

			cpPolyShape poly1 = (cpPolyShape)polygon1;
			cpPolyShape poly2 = (cpPolyShape)polygon2;

			SupportContext context = new SupportContext(poly1, poly2,
				(s0, s1) => PolySupportPoint(s0 as cpPolyShape, s1),
				(s0, s1) => PolySupportPoint(s0 as cpPolyShape, s1));

			ClosestPoints points = GJK(ref context, ref info.id);


			// If the closest points are nearer than the sum of the radii...
			if (points.d - poly1.r - poly2.r <= 0.0)
			{
				ContactPoints(
					Edge.SupportEdgeForPoly(poly1, points.n),
					Edge.SupportEdgeForPoly(poly2, cpVect.cpvneg(points.n)),
					points,
					ref info);
			}
		}



		public static void SegmentToPoly(cpShape seg1, cpShape poly2, ref cpCollisionInfo info)
		{

			cpSegmentShape seg = (cpSegmentShape)seg1;
			cpPolyShape poly = (cpPolyShape)poly2;

			SupportContext context = new SupportContext(seg, poly,
				(s, p) => SupportPoint.SegmentSupportPoint(s as cpSegmentShape, p),

				(s, p) => SupportPoint.PolySupportPoint(s as cpPolyShape, p));

			ClosestPoints points = GJK(ref context, ref info.id);


			cpVect n = points.n;
			cpVect rot = seg.body.GetRotation();

			if (
				// If the closest points are nearer than the sum of the radii...
				points.d - seg.r - poly.r <= 0.0f &&
				(
				// Reject endcap collisions if tangents are provided.
					(!cpVect.cpveql(points.a, seg.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg.a_tangent, rot)) <= 0.0f) &&
					(!cpVect.cpveql(points.a, seg.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg.b_tangent, rot)) <= 0.0f)
				)
			)
			{
				ContactPoints(Edge.SupportEdgeForSegment(seg, n), Edge.SupportEdgeForPoly(poly, cpVect.cpvneg(n)), points, ref info);
			}
		}

		// This one is less gross, but still gross.
		public static void CircleToPoly(cpShape circle1, cpShape poly2, ref cpCollisionInfo info)
		{

			cpCircleShape circle = (cpCircleShape)circle1;
			cpPolyShape poly = (cpPolyShape)poly2;

			SupportContext context = new SupportContext(
				circle,
				poly,
				(s, o) => SupportPoint.CircleSupportPoint(s as cpCircleShape, o),
				(s, o) => SupportPoint.PolySupportPoint(s as cpPolyShape, o));

			ClosestPoints points = GJK(ref context, ref info.id);


			// If the closest points are nearer than the sum of the radii...
			if (points.d <= circle.r + poly.r)
			{
				cpVect n = info.n = points.n;
				InfoPushContact(ref info, cpVect.cpvadd(points.a, cpVect.cpvmult(n, circle.r)), cpVect.cpvadd(points.b, cpVect.cpvmult(n, poly.r)), 0);
			}
		}

		public static void CollisionError(cpShape circle, cpShape poly, ref cpCollisionInfo info)
		{
			cp.AssertHard(false, "Internal Error: Shape types are not sorted.");
		}

		public delegate void ShapeToShapeDelegate(cpShape shape1, cpShape shape2, ref cpCollisionInfo info);

		static ShapeToShapeDelegate[] BuiltinCollisionFuncs = new ShapeToShapeDelegate[9]
		{
			CircleToCircle,
			CollisionError,
			CollisionError,
			CircleToSegment,	
			SegmentToSegment,	
			CollisionError,
			CircleToPoly,
			SegmentToPoly,
			PolyToPoly,
};


		public static ShapeToShapeDelegate[] CollisionFuncs = BuiltinCollisionFuncs;


		public static cpCollisionInfo cpCollide(cpShape a, cpShape b, ulong id, ref List<cpContact> contacts)
		{
			cpCollisionInfo info = new cpCollisionInfo(a, b, id, cpVect.Zero, contacts);

			// Make sure the shape types are in order.
			if ((int)a.shapeType > (int)b.shapeType)
			{
				info.a = b;
				info.b = a;
			}

			int idSelected = (int)info.a.shapeType + (int)info.b.shapeType * (int)cpShapeType.NumShapes;
			CollisionFuncs[idSelected](info.a, info.b, ref info);
			return info;
		}


		///////////////////////////////////////////////////////////////////////////


		public static SupportPoint PolySupportPoint(cpPolyShape poly, cpVect n)
		{
			cpSplittingPlane[] planes = poly.planes;
			ulong i = PolySupportPointIndex(poly.Count, planes, n);
			return new SupportPoint(planes[i].v0, i);
		}


	}

}

