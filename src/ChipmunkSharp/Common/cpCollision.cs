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
using ChipmunkSharp.Shapes;
using System.Linq;
// // typedef int (*collisionFunc)(cpShape , cpShape , cpContact );
using System;
using System.Collections.Generic;

namespace ChipmunkSharp
{

    //MARK: Support Points and Edges:
    #region Support Points and Edges

    public struct EdgePoint
    {
        public cpVect p;
        public int hash;

        public EdgePoint(cpVect p, int hash)
        {

            this.p = p;
            this.hash = hash;
        }
    };

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

        public static Edge EdgeNew(cpVect va, cpVect vb, int ha, int hb, float r)
        {
            return new Edge(
                new EdgePoint(va, ha),
                new EdgePoint(vb, hb),
                r,
                cpVect.cpvnormalize(cpVect.cpvperp(cpVect.cpvsub(vb, va))));
        }

        public static Edge SupportEdgeForPoly(cpPolyShape poly, cpVect n)
        {
            int numVerts = poly.numVerts;

            int i1 = cpCollision.PolySupportPointIndex(poly.tVerts, n);

            // TODO get rid of mod eventually, very expensive on ARM
            int i0 = (i1 - 1 + numVerts) % numVerts;
            int i2 = (i1 + 1) % numVerts;

            List<cpVect> verts = poly.tVerts;
            if (cpVect.cpvdot(n, poly.tPlanes[i1].n) > cpVect.cpvdot(n, poly.tPlanes[i2].n))
            {
                Edge edge = new Edge(

                 new EdgePoint(verts[i0], cpEnvironment.CP_HASH_PAIR(poly, (int)i0)),
                 new EdgePoint(verts[i1], cpEnvironment.CP_HASH_PAIR(poly, (int)i1)),

                 poly.r, poly.tPlanes[i1].n);

                return edge;
            }
            else
            {

                Edge edge = new Edge(
                new EdgePoint(verts[i1], cpEnvironment.CP_HASH_PAIR(poly, (int)i1)),
                new EdgePoint(verts[i2], cpEnvironment.CP_HASH_PAIR(poly, (int)i2)),
                poly.r, poly.tPlanes[i2].n);
                return edge;
            }
        }

        public static Edge SupportEdgeForSegment(cpSegmentShape seg, cpVect n)
        {

            Edge edge;

            if (cpVect.cpvdot(seg.tn, n) > 0.0f)
            {
                edge = new Edge(
                   new EdgePoint(seg.ta, cpEnvironment.CP_HASH_PAIR(seg, (int)0)),
                    new EdgePoint(seg.tb, cpEnvironment.CP_HASH_PAIR(seg, (int)0)),
                    seg.r, seg.tn);

            }
            else
            {
                edge = new Edge(
        new EdgePoint(seg.tb, cpEnvironment.CP_HASH_PAIR(seg, 0)),
         new EdgePoint(seg.ta, cpEnvironment.CP_HASH_PAIR(seg, 0)),
         seg.r, seg.tn.Neg());


            }
            return edge;
        }

    };

    public struct SupportContext
    {
        public cpShape shape1, shape2;
        public SupportPointFunc func1, func2;

        public SupportContext(cpShape shape1, cpShape shape2, SupportPointFunc func1, SupportPointFunc func2)
        {
            this.shape1 = shape1;
            this.shape2 = shape2;
            this.func1 = func1;
            this.func2 = func2;
        }

        public static MinkowskiPoint Support(SupportContext ctx, cpVect n)
        {

            SupportPoint a = ctx.func1(ctx.shape1, cpVect.cpvneg(n));
            SupportPoint b = ctx.func2(ctx.shape2, n);
            return MinkowskiPoint.MinkowskiPointNew(a, b);

        }

    };

    public struct SupportPoint
    {
        public cpVect p;
        public int id;

        public SupportPoint(cpVect p, int id)
        {
            this.p = p;
            this.id = id;
        }

    };

    public delegate SupportPoint SupportPointFunc(cpShape shape, cpVect n);

    public struct MinkowskiPoint
    {

        public cpVect a, b;
        public cpVect ab;
        public int id;

        public MinkowskiPoint(cpVect a, cpVect b, cpVect ab, int id)
        {
            this.a = a;
            this.b = b;
            this.ab = ab;
            this.id = id;
        }

        public static MinkowskiPoint MinkowskiPointNew(SupportPoint a, SupportPoint b)
        {
            return new MinkowskiPoint(a.p, b.p, b.p.Sub(a.p), ((a.id & 0xFF) << 8 | (b.id & 0xFF)));
        }

        public static MinkowskiPoint Support(SupportContext ctx, cpVect n)
        {
            SupportPoint a = ctx.func1(ctx.shape1, cpVect.cpvneg(n));
            SupportPoint b = ctx.func2(ctx.shape2, n);
            return MinkowskiPointNew(a, b);
        }

    };


    public struct ClosestPoints
    {

        public cpVect a, b;
        public cpVect n;
        public float d;
        public int id;


        public ClosestPoints(cpVect a, cpVect b, cpVect n, float d, int id)
        {
            this.a = a;
            this.b = b;
            this.n = n;
            this.d = d;
            this.id = id;
        }

        public static ClosestPoints ClosestPointsNew(MinkowskiPoint v0, MinkowskiPoint v1)
        {
            float t = cpCollision.ClosestT(v0.ab, v1.ab);
            cpVect p = cpCollision.LerpT(v0.ab, v1.ab, t);

            cpVect pa = cpCollision.LerpT(v0.a, v1.a, t);
            cpVect pb = cpCollision.LerpT(v0.b, v1.b, t);
            int id = (v0.id & 0xFFFF) << 16 | (v1.id & 0xFFFF);

            cpVect delta = cpVect.cpvsub(v1.ab, v0.ab);
            cpVect n = cpVect.cpvnormalize(cpVect.cpvperp(delta));
            float d = -cpVect.cpvdot(n, p);

            if (d <= 0.0f || (0.0f < t && t < 1.0f))
            {
                ClosestPoints points = new ClosestPoints(pa, pb, cpVect.cpvneg(n), d, id);
                return points;
            }
            else
            {
                float d2 = cpVect.cpvlength(p);
                cpVect n2 = cpVect.cpvmult(p, 1.0f / (d2 + float.MinValue));

                ClosestPoints points = new ClosestPoints(pa, pb, n2, d2, id);
                return points;
            }

        }
    }


    #endregion


    public delegate int CollisionFunc(cpShape a, cpShape b, int id, List<cpContact> arr);

    public class cpCollision
    {



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

        #region PROPERTIES

        public int a;

        int numContacts = 0;

        public cpVect r2 { get; set; }

        public cpVect r1 { get; set; }

        public float bias { get; set; }

        public float bounce { get; set; }

        public float tMass { get; set; }

        public float nMass { get; set; }

        public object hash { get; set; }

        public float jBias { get; set; }

        public float jtAcc { get; set; }

        public float jnAcc { get; set; }

        #endregion

        //public cpCollision(object p, object n, object dist, object hash)
        //{
        //    throw new NotImplementedException("Not implemented");
        //    //this.p = p;
        //    //this.n = n;
        //    //this.dist = dist;

        //    //this.r1 = this.r2 = cpVect.ZERO;
        //    //this.nMass = this.tMass = this.bounce = this.bias = 0.0f;

        //    //this.jnAcc = this.jtAcc = this.jBias = 0.0f;

        //    //this.hash = hash;
        //    //numContacts++;
        //}


        public static float ClosestT(cpVect a, cpVect b)
        {
            cpVect delta = cpVect.cpvsub(b, a);
            return -cpEnvironment.cpfclamp(cpVect.cpvdot(delta, cpVect.cpvadd(a, b)) / cpVect.cpvlengthsq(delta), -1.0f, 1.0f);
        }

        public static cpVect LerpT(cpVect a, cpVect b, float t)
        {
            float ht = 0.5f * t;
            return cpVect.cpvadd(cpVect.cpvmult(a, 0.5f - ht), cpVect.cpvmult(b, 0.5f + ht));
        }

        //MARK: EPA Functions

        #region EPA Functions
        public static float ClosestDist(cpVect v0, cpVect v1)
        {
            return cpVect.cpvlengthsq(LerpT(v0, v1, ClosestT(v0, v1)));
        }

        public static ClosestPoints EPARecurse(SupportContext ctx, List<MinkowskiPoint> hull, int iteration)
        {
            int mini = 0;
            float minDist = cpEnvironment.INFINITY_FLOAT;

            int count = hull.Count;

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
            MinkowskiPoint v1 = hull[(mini + 1) % hull.Count];
            cpEnvironment.cpAssertSoft(!cpVect.cpveql(v0.ab, v1.ab), string.Format("Internal Error: EPA vertexes are the same ({0} and {1})", mini, (mini + 1) % count));

            MinkowskiPoint p = MinkowskiPoint.Support(ctx, cpVect.cpvperp(cpVect.cpvsub(v1.ab, v0.ab)));

#if DRAW_EPA
	cpVect verts[count];
	for(int i=0; i<count; i++) verts[i] = hull[i].ab;
	
	ChipmunkDebugDrawPolygon(count, verts, RGBAColor(1, 1, 0, 1), RGBAColor(1, 1, 0, 0.25));
	ChipmunkDebugDrawSegment(v0.ab, v1.ab, RGBAColor(1, 0, 0, 1));
	
	ChipmunkDebugDrawPoints(5, 1, (cpVect[]){p.ab}, RGBAColor(1, 1, 1, 1));
#endif

            float area2x = cpVect.cpvcross(cpVect.cpvsub(v1.ab, v0.ab), cpVect.cpvadd(cpVect.cpvsub(p.ab, v0.ab), cpVect.cpvsub(p.ab, v1.ab)));
            if (area2x > 0.0f && iteration < MAX_EPA_ITERATIONS)
            {
                int count2 = 1;
                List<MinkowskiPoint> hull2 = new List<MinkowskiPoint>(count + 1);
                hull2[0] = p;

                for (int i = 0; i < count; i++)
                {
                    int index = (mini + 1 + i) % count;

                    cpVect h0 = hull2[count2 - 1].ab;
                    cpVect h1 = hull[index].ab;
                    cpVect h2 = (i + 1 < count ? hull[(index + 1) % count] : p).ab;

                    // TODO: Should this be changed to an area2x check?
                    if (cpVect.cpvcross(cpVect.cpvsub(h2, h0), cpVect.cpvsub(h1, h0)) > 0.0f)
                    {
                        hull2[count2] = hull[index];
                        count2++;
                    }
                }

                return EPARecurse(ctx, hull2, iteration + 1);
            }
            else
            {
                cpEnvironment.cpAssertWarn(iteration < WARN_EPA_ITERATIONS, string.Format("High EPA iterations: {0}", iteration));
                return ClosestPoints.ClosestPointsNew(v0, v1);
            }
        }

        public static ClosestPoints EPA(SupportContext ctx, MinkowskiPoint v0, MinkowskiPoint v1, MinkowskiPoint v2)
        {
            // TODO: allocate a NxM array here and do an in place convex hull reduction in EPARecurse
            List<MinkowskiPoint> hull = new List<MinkowskiPoint>() { v0, v1, v2 };
            return EPARecurse(ctx, hull, 1);
        }

        #endregion

        //MARK: GJK Functions.

        #region GJK Functions

        public static ClosestPoints GJKRecurse(SupportContext ctx, MinkowskiPoint v0, MinkowskiPoint v1, int iteration)
        {

            if (iteration > MAX_GJK_ITERATIONS)
            {
                cpEnvironment.cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK iterations: %d", iteration);
                return ClosestPoints.ClosestPointsNew(v0, v1);
            }

            cpVect delta = cpVect.cpvsub(v1.ab, v0.ab);
            if (cpVect.cpvcross(delta, cpVect.cpvadd(v0.ab, v1.ab)) > 0.0f)
            {
                // Origin is behind axis. Flip and try again.
                return GJKRecurse(ctx, v1, v0, iteration + 1);
            }
            else
            {
                float t = ClosestT(v0.ab, v1.ab);
                cpVect n = (-1.0f < t && t < 1.0f ? cpVect.cpvperp(delta) : cpVect.cpvneg(LerpT(v0.ab, v1.ab, t)));
                MinkowskiPoint p = MinkowskiPoint.Support(ctx, n);

#if DRAW_GJK
		ChipmunkDebugDrawSegment(v0.ab, v1.ab, RGBAColor(1, 1, 1, 1));
		cpVect c = cpvlerp(v0.ab, v1.ab, 0.5);
		ChipmunkDebugDrawSegment(c, cpvadd(c, cpvmult(cpvnormalize(n), 5.0)), RGBAColor(1, 0, 0, 1));
		
		ChipmunkDebugDrawPoints(5.0, 1, &p.ab, RGBAColor(1, 1, 1, 1));
#endif

                if (
                    cpVect.cpvcross(cpVect.cpvsub(v1.ab, p.ab), cpVect.cpvadd(v1.ab, p.ab)) > 0.0f &&
                   cpVect.cpvcross(cpVect.cpvsub(v0.ab, p.ab), cpVect.cpvadd(v0.ab, p.ab)) < 0.0f
                )
                {
                    cpEnvironment.cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK->EPA iterations: %d", iteration);
                    // The triangle v0, p, v1 contains the origin. Use EPA to find the MSA.
                    return EPA(ctx, v0, p, v1);
                }
                else
                {
                    // The new point must be farther along the normal than the existing points.
                    if (cpVect.cpvdot(p.ab, n) <= cpEnvironment.cpfmax(cpVect.cpvdot(v0.ab, n), cpVect.cpvdot(v1.ab, n)))
                    {
                        cpEnvironment.cpAssertWarn(iteration < WARN_GJK_ITERATIONS, "High GJK iterations: %d", iteration);
                        return ClosestPoints.ClosestPointsNew(v0, v1);
                    }
                    else
                    {
                        if (ClosestDist(v0.ab, p.ab) < ClosestDist(p.ab, v1.ab))
                        {
                            return GJKRecurse(ctx, v0, p, iteration + 1);
                        }
                        else
                        {
                            return GJKRecurse(ctx, p, v1, iteration + 1);
                        }
                    }
                }
            }
        }

        public static SupportPoint ShapePoint(cpShape shape, int i)
        {
            switch (shape.klass.type)
            {
                case cpShapeType.CP_CIRCLE_SHAPE:
                    return new SupportPoint(((cpCircleShape)shape).tc, 0);
                case cpShapeType.CP_SEGMENT_SHAPE:
                    cpSegmentShape seg = (cpSegmentShape)shape;
                    return new SupportPoint(i == 0 ? seg.ta : seg.tb, (int)i);
                case cpShapeType.CP_POLY_SHAPE:
                    cpPolyShape poly = (cpPolyShape)shape;
                    // Poly shapes may change vertex count.
                    int index = (i < (int)poly.numVerts ? i : (int)0);
                    return new SupportPoint(poly.tVerts[(int)index], (int)index);
                default:
                    return new SupportPoint(cpVect.ZERO, 0);
            }
        }


        public static ClosestPoints GJK(SupportContext ctx, int id)
        {
#if DRAW_GJK || DRAW_EPA
	// draw the minkowski difference origin
	cpVect origin = cpvzero;
	ChipmunkDebugDrawPoints(5.0, 1, &origin, RGBAColor(1,0,0,1));
	
	int mdiffCount = ctx->count1*ctx->count2;
	cpVect *mdiffVerts = alloca(mdiffCount*sizeof(cpVect));
	
	for(int i=0; i<ctx->count1; i++){
		for(int j=0; j<ctx->count2; j++){
			cpVect v1 = ShapePoint(ctx->count1, ctx->verts1, i).p;
			cpVect v2 = ShapePoint(ctx->count2, ctx->verts2, j).p;
			mdiffVerts[i*ctx->count2 + j] = cpvsub(v2, v1);
		}
	}
	 
	cpVect *hullVerts = alloca(mdiffCount*sizeof(cpVect));
	int hullCount = cpConvexHull(mdiffCount, mdiffVerts, hullVerts, NULL, 0.0);
	
	ChipmunkDebugDrawPolygon(hullCount, hullVerts, RGBAColor(1, 0, 0, 1), RGBAColor(1, 0, 0, 0.25));
	ChipmunkDebugDrawPoints(2.0, mdiffCount, mdiffVerts, RGBAColor(1, 0, 0, 1));
#endif

            MinkowskiPoint v0, v1;
            if (id == 0 && ENABLE_CACHING == 0)
            {
                v0 = MinkowskiPoint.MinkowskiPointNew(
                    ShapePoint(ctx.shape1, (id >> 24) & 0xFF),

                    ShapePoint(ctx.shape2, (id >> 16) & 0xFF));
                v1 = MinkowskiPoint.MinkowskiPointNew(ShapePoint(ctx.shape1, (id >> 8) & 0xFF), ShapePoint(ctx.shape2, (id) & 0xFF));
            }
            else
            {
                cpVect axis = cpVect.cpvperp(cpVect.cpvsub(cpBB.Center(ctx.shape1.bb), cpBB.Center(ctx.shape2.bb)));
                v0 = MinkowskiPoint.Support(ctx, axis);
                v1 = MinkowskiPoint.Support(ctx, cpVect.cpvneg(axis));
            }

            ClosestPoints points = GJKRecurse(ctx, v0, v1, 1);
            id = points.id;
            return points;
        }

        #endregion

        //MARK: Contact Clipping

        #region Contact Clipping


        public static void Contact1(float dist, cpVect a, cpVect b, float refr, float incr, cpVect n, int hash, List<cpContact> arr)
        {
            float rsum = refr + incr;
            float alpha = (rsum > 0.0f ? refr / rsum : 0.5f);
            cpVect point = cpVect.cpvlerp(a, b, alpha);

            //TODO: NOT CLEAR
            foreach (var item in arr)
            {
                item.Init(point, n, dist - rsum, hash);
            }


        }

        public static int Contact2(cpVect refp, cpVect inca, cpVect incb, float refr, float incr, cpVect refn, cpVect n, int hash, List<cpContact> arr)
        {
            float cian = cpVect.cpvcross(inca, refn);
            float cibn = cpVect.cpvcross(incb, refn);
            float crpn = cpVect.cpvcross(refp, refn);
            float t = 1.0f - cpEnvironment.cpfclamp01((cibn - crpn) / (cibn - cian));

            cpVect point = cpVect.cpvlerp(inca, incb, t);
            float pd = cpVect.cpvdot(cpVect.cpvsub(point, refp), refn);

            if (t > 0.0f && pd <= 0.0f)
            {
                float rsum = refr + incr;
                float alpha = (rsum > 0.0f ? incr * (1.0f - (rsum + pd) / rsum) : -0.5f * pd);

                //TODO: NOT CLEAR
                foreach (var item in arr)
                {
                    item.Init(cpVect.cpvadd(point, cpVect.cpvmult(refn, alpha)), n, pd, hash);
                }


                return 1;
            }
            else
            {
                return 0;
            }
        }

        public static int ClipContacts(Edge refe, Edge inc, ClosestPoints points, float nflip, List<cpContact> arr)
        {
            cpVect inc_offs = cpVect.cpvmult(inc.n, inc.r);
            cpVect ref_offs = cpVect.cpvmult(refe.n, refe.r);

            cpVect inca = cpVect.cpvadd(inc.a.p, inc_offs);
            cpVect incb = cpVect.cpvadd(inc.b.p, inc_offs);

            cpVect closest_inca = cpVect.closestPointOnSegment(inc.a.p, refe.a.p, refe.b.p);
            cpVect closest_incb = cpVect.closestPointOnSegment(inc.b.p, refe.a.p, refe.b.p);

            cpVect msa = cpVect.cpvmult(points.n, nflip * points.d);
            float cost_a = cpVect.cpvdistsq(cpVect.cpvsub(inc.a.p, closest_inca), msa);
            float cost_b = cpVect.cpvdistsq(cpVect.cpvsub(inc.b.p, closest_incb), msa);

#if DRAW_CLIP
	ChipmunkDebugDrawSegment(ref.a.p, ref.b.p, RGBAColor(1, 0, 0, 1));
	ChipmunkDebugDrawSegment(inc.a.p, inc.b.p, RGBAColor(0, 1, 0, 1));
	ChipmunkDebugDrawSegment(inca, incb, RGBAColor(0, 1, 0, 1));
	
	cpVect cref = cpvlerp(ref.a.p, ref.b.p, 0.5);
	ChipmunkDebugDrawSegment(cref, cpvadd(cref, cpvmult(ref.n, 5.0)), RGBAColor(1, 0, 0, 1));
	
	cpVect cinc = cpvlerp(inc.a.p, inc.b.p, 0.5);
	ChipmunkDebugDrawSegment(cinc, cpvadd(cinc, cpvmult(inc.n, 5.0)), RGBAColor(1, 0, 0, 1));
	
	ChipmunkDebugDrawPoints(5.0, 2, (cpVect[]){ref.a.p, inc.a.p}, RGBAColor(1, 1, 0, 1));
	ChipmunkDebugDrawPoints(5.0, 2, (cpVect[]){ref.b.p, inc.b.p}, RGBAColor(0, 1, 1, 1));
	
	if(cost_a < cost_b){
		ChipmunkDebugDrawSegment(closest_inca, inc.a.p, RGBAColor(1, 0, 1, 1));
	} else {
		ChipmunkDebugDrawSegment(closest_incb, inc.b.p, RGBAColor(1, 0, 1, 1));
	}
#endif

            int hash_iarb = cpEnvironment.CP_HASH_PAIR(inc.a.hash, refe.b.hash);
            int hash_ibra = cpEnvironment.CP_HASH_PAIR(inc.b.hash, refe.a.hash);

            if (cost_a < cost_b)
            {
                cpVect refp = cpVect.cpvadd(refe.a.p, ref_offs);

                //TODO: It's not clear
                Contact1(points.d, closest_inca, inc.a.p, refe.r, inc.r, points.n, hash_iarb, arr);
                return Contact2(refp, inca, incb, refe.r, inc.r, refe.n, points.n, hash_ibra, arr) + 1;
            }
            else
            {
                //TODO: It's not clear
                cpVect refp = cpVect.cpvadd(refe.b.p, ref_offs);
                Contact1(points.d, closest_incb, inc.b.p, refe.r, inc.r, points.n, hash_ibra, arr);
                return Contact2(refp, incb, inca, refe.r, inc.r, refe.n, points.n, hash_iarb, arr) + 1;
            }
        }


        public static int ContactPoints(Edge e1, Edge e2, ClosestPoints points, List<cpContact> arr)
        {
            float mindist = e1.r + e2.r;
            if (points.d <= mindist)
            {
                float pick = cpVect.cpvdot(e1.n, points.n) + cpVect.cpvdot(e2.n, points.n);

                if (
                    (pick != 0.0f && pick > 0.0f) ||
                    // If the edges are both perfectly aligned weird things happen.
                    // This is *very* common at the start of a simulation.
                    // Pick the longest edge as the reference to break the tie.
                    (pick == 0.0f && (cpVect.cpvdistsq(e1.a.p, e1.b.p) > cpVect.cpvdistsq(e2.a.p, e2.b.p)))
                )
                {
                    return ClipContacts(e1, e2, points, 1.0f, arr);
                }
                else
                {
                    return ClipContacts(e2, e1, points, -1.0f, arr);
                }
            }
            else
            {
                return 0;
            }
        }


        #endregion


        //MARK: Collision Functions


        #region  Collision Functions


        // Add contact points for circle to circle collisions.
        // Used by several collision tests.
        // TODO should accept hash parameter
        public static int CircleToCircleQuery(cpVect p1, cpVect p2, float r1, float r2, int hash, List<cpContact> con)
        {
            float mindist = r1 + r2;
            cpVect delta = cpVect.cpvsub(p2, p1);
            float distsq = cpVect.cpvlengthsq(delta);

            if (distsq < mindist * mindist)
            {

                float dist = cpEnvironment.cpfsqrt(distsq);
                cpVect n = (dist == 0 ? cpVect.cpvmult(delta, 1.0f / dist) : new cpVect(1.0f, 0.0f));
                con.FirstOrDefault().Init(cpVect.cpvlerp(p1, p2, r1 / (r1 + r2)), n, dist - mindist, hash);


                return 1;
            }
            else
            {
                return 0;
            }
        }

        // Collide circle shapes.
        public static int CircleToCircle(cpCircleShape c1, cpCircleShape c2, int id, List<cpContact> arr)
        {
            return CircleToCircleQuery(c1.tc, c2.tc, c1.r, c2.r, 0, arr);
        }

        public static int CircleToSegment(cpCircleShape circleShape, cpSegmentShape segmentShape, int id, List<cpContact> con)
        {

            cpVect seg_a = segmentShape.ta;
            cpVect seg_b = segmentShape.tb;
            cpVect center = circleShape.tc;

            cpVect seg_delta = cpVect.cpvsub(seg_b, seg_a);
            float closest_t = cpEnvironment.cpfclamp01(cpVect.cpvdot(seg_delta, cpVect.cpvsub(center, seg_a)) / cpVect.cpvlengthsq(seg_delta));
            cpVect closest = cpVect.cpvadd(seg_a, cpVect.cpvmult(seg_delta, closest_t));

            if (CircleToCircleQuery(center, closest, circleShape.r, segmentShape.r, 0, con) == 0)
            {
                cpVect n = con[0].n;

                // Reject endcap collisions if tangents are provided.
                if (
                    (closest_t != 0.0f || cpVect.cpvdot(n, cpVect.cpvrotate(segmentShape.a_tangent, segmentShape.shape.body.Rotation)) >= 0.0f) &&
                    (closest_t != 1.0f || cpVect.cpvdot(n, cpVect.cpvrotate(segmentShape.b_tangent, segmentShape.shape.body.Rotation)) >= 0.0f)
                )
                {
                    return 1;
                }
            }

            return 0;
        }

        public static int SegmentToSegment(cpSegmentShape seg1, cpSegmentShape seg2, int id, List<cpContact> arr)
        {

            SupportContext context = new SupportContext((cpShape)seg1, (cpShape)seg2, (a, b) =>
            {
                return SegmentSupportPoint((cpSegmentShape)a, b);
            }, (a, b) =>
            {
                return SegmentSupportPoint((cpSegmentShape)a, b);
            });


            ClosestPoints points = GJK(context, id);

#if DRAW_CLOSEST
#if PRINT_LOG
//	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
#endif
	
	ChipmunkDebugDrawDot(6.0, points.a, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawDot(6.0, points.b, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
#endif

            cpVect n = points.n;
            cpVect rot1 = seg1.shape.body.Rotation;
            cpVect rot2 = seg2.shape.body.Rotation;
            if (
                points.d <= (seg1.r + seg2.r) &&
                (
                    (!cpVect.cpveql(points.a, seg1.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg1.a_tangent, rot1)) <= 0.0) &&
                    (!cpVect.cpveql(points.a, seg1.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg1.b_tangent, rot1)) <= 0.0) &&
                    (!cpVect.cpveql(points.b, seg2.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg2.a_tangent, rot2)) >= 0.0) &&
                    (!cpVect.cpveql(points.b, seg2.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg2.b_tangent, rot2)) >= 0.0)
                )
            )
            {
                return ContactPoints(Edge.SupportEdgeForSegment(seg1, n), Edge.SupportEdgeForSegment(seg2, cpVect.cpvneg(n)), points, arr);
            }
            else
            {
                return 0;
            }
        }

        public static int PolyToPoly(cpPolyShape poly1, cpPolyShape poly2, int id, List<cpContact> arr)
        {

            SupportContext context = new SupportContext(
                 (cpShape)poly1, (cpShape)poly2,
                  (a, b) => { return PolySupportPoint((cpPolyShape)a, b); },

                  (a, b) => { return PolySupportPoint((cpPolyShape)a, b); }
        );


            ClosestPoints points = GJK(context, id);

#if DRAW_CLOSEST
#if PRINT_LOG
//	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
#endif
	
	ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
#endif

            if (points.d - poly1.r - poly2.r <= 0.0)
            {
                return ContactPoints(Edge.SupportEdgeForPoly(poly1, points.n), Edge.SupportEdgeForPoly(poly2, cpVect.cpvneg(points.n)), points, arr);
            }
            else
            {
                return 0;
            }
        }

        public static int SegmentToPoly(cpSegmentShape seg, cpPolyShape poly, int id, List<cpContact> arr)
        {
            SupportContext context = new SupportContext(
                   (cpShape)seg, (cpShape)poly,

                   (a, b) =>
                   {
                       return SegmentSupportPoint((cpSegmentShape)a, b);
                   }, (a, b) =>
                   {
                       return PolySupportPoint((cpPolyShape)a, b);
                   });

            ClosestPoints points = GJK(context, id);

#if DRAW_CLOSEST
#if PRINT_LOG
//	ChipmunkDemoPrintString("Distance: %.2f\n", points.d);
#endif
	
	ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
#endif

            // Reject endcap collisions if tangents are provided.
            cpVect n = points.n;
            cpVect rot = seg.shape.body.Rotation;
            if (
                points.d - seg.r - poly.r <= 0.0 &&
                (
                    (!cpVect.cpveql(points.a, seg.ta) || cpVect.cpvdot(n, cpVect.cpvrotate(seg.a_tangent, rot)) <= 0.0) &&
                    (!cpVect.cpveql(points.a, seg.tb) || cpVect.cpvdot(n, cpVect.cpvrotate(seg.b_tangent, rot)) <= 0.0)
                )
            )
            {
                return ContactPoints(Edge.SupportEdgeForSegment(seg, n), Edge.SupportEdgeForPoly(poly, cpVect.cpvneg(n)), points, arr);
            }
            else
            {
                return 0;
            }
        }

        // This one is less gross, but still gross.
        // TODO: Comment me!
        static int CircleToPoly(cpCircleShape circle, cpPolyShape poly, int id, cpContact con)
        {

            SupportContext context = new SupportContext(circle, poly,
                (s, n) =>
                {
                    return CircleSupportPoint((cpCircleShape)s, n);
                },
                 (s, n) =>
                 {
                     return PolySupportPoint((cpPolyShape)s, n);
                 }
                     );

            //{ (cpShape)circle, (cpShape)poly, (SupportPointFunc)CircleSupportPoint, (SupportPointFunc)PolySupportPoint };

            ClosestPoints points = GJK(context, id);

#if DRAW_CLOSEST
	ChipmunkDebugDrawDot(3.0, points.a, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawDot(3.0, points.b, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawSegment(points.a, points.b, RGBAColor(1, 1, 1, 1));
	ChipmunkDebugDrawSegment(points.a, cpvadd(points.a, cpvmult(points.n, 10.0)), RGBAColor(1, 0, 0, 1));
#endif

            float mindist = circle.r + poly.r;
            if (points.d - mindist <= 0.0)
            {
                cpVect p = cpVect.cpvlerp(points.a, points.b, circle.r / (mindist));
                con.Init(p, points.n, points.d - mindist, 0);
                return 1;
            }
            else
            {
                return 0;
            }
        }

        static CollisionFunc[] builtinCollisionFuncs = new CollisionFunc[]  {

            //(CollisionFunc)CircleToCircle,
            //null,
            //null,
            //(CollisionFunc)CircleToSegment,
            //null,
            //null,
            //(CollisionFunc)CircleToPoly,
            //(CollisionFunc)SegmentToPoly,
            //(CollisionFunc)PolyToPoly,
        };

        static CollisionFunc[] colfuncs = builtinCollisionFuncs;

        static CollisionFunc[] segmentCollisions = new CollisionFunc[] {

            (a,b,c,d) => { return CircleToCircle( (cpCircleShape) a, (cpCircleShape)b,c,d);  },
            null,
            null,
              (a,b,c,d) => { return CircleToCircle( (cpCircleShape) a, (cpCircleShape)b,c,d); },
            (a,b,c,d) => { return SegmentToSegment( (cpSegmentShape) a, (cpSegmentShape)b,c,d); },
            null,
              (a,b,c,d) => { return CircleToCircle( (cpCircleShape) a, (cpCircleShape)b,c,d); },
            (a,b,c,d) => { return SegmentToSegment( (cpSegmentShape) a, (cpSegmentShape)b,c,d); },
              (a,b,c,d) => { return PolyToPoly( (cpPolyShape) a, (cpPolyShape)b,c,d); }
        };

        public void cpEnableSegmentToSegmentCollisions()
        {
            colfuncs = segmentCollisions;
            //TODO: 
        }

        public static int cpCollideShapes(cpShape a, cpShape b, int id, List<cpContact> arr)
        {
            // Their shape types must be in order.
            cpEnvironment.cpAssertSoft(a.klass.type <= b.klass.type, "Internal Error: Collision shapes passed to cpCollideShapes() are not sorted.");

            CollisionFunc cfunc = colfuncs[((int)a.klass.type) + ((int)b.klass.type) * ((int)cpShapeType.CP_NUM_SHAPES)];

            int numContacts = (cfunc != null ? cfunc(a, b, id, arr) : 0);
            cpEnvironment.cpAssertSoft(numContacts <= cpArbiter.CP_MAX_CONTACTS_PER_ARBITER, "Internal error: Too many contact points returned.");

            return numContacts;
        }


        #endregion

        //MARK: Support Points and Edges:
        public static int PolySupportPointIndex(List<cpVect> verts, cpVect n)
        {
            float max = -cpEnvironment.INFINITY_FLOAT;
            int index = 0;

            for (int i = 0; i < verts.Count; i++)
            {
                cpVect v = verts[i];
                float d = v.Dot(n); // cpvdot(v, n);
                if (d > max)
                {
                    max = d;
                    index = i;
                }
            }

            return index;
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
            int i = PolySupportPointIndex(poly.tVerts, n);

            return new SupportPoint(poly.tVerts[i], i);
        }


        //#region NOT USED

        // Add contact points for circle to circle collisions.
        // Used by several collision tests.
        // TODO should accept hash parameter
        //public static int circle2circleQuery(cpVect p1, cpVect p2, float r1, float r2, int hash, cpContact con)
        //{
        //    float mindist = r1 + r2;
        //    cpVect delta = cpVect.Sub(p2, p1);
        //    float distsq = delta.LengthSQ;
        //    if (distsq >= mindist * mindist)
        //        return 0;

        //    float dist = cpEnvironment.cpfsqrt(distsq);
        //    cpVect n = (dist == 0 ? cpVect.cpvmult(delta, 1.0f / dist) : new cpVect(1.0f, 0.0f));

        //    con.Init(cpVect.cpvlerp(p1, p2, r1 / (r1 + r2)), n, dist - mindist, hash);
        //    return 1;
        //}


        //// Collide circle shapes.
        //public static int circle2circle(cpShape shape1, cpShape shape2, cpContact arr)
        //{
        //    throw new NotImplementedException("Not implemented");
        //    //cpCircleShape circ1 = (cpCircleShape)shape1; //TODO
        //    //cpCircleShape circ2 = (cpCircleShape)shape2;

        //    //return circle2circleQuery(circ1.tc, circ2.tc, circ1.r, circ2.r, arr);
        //}

        //public static int circle2segment(cpCircleShape circleShape, cpSegmentShape segmentShape, List<cpContact> con)
        //{
        //    cpVect seg_a = segmentShape.ta;
        //    cpVect seg_b = segmentShape.tb;
        //    cpVect center = circleShape.tc;

        //    cpVect seg_delta = cpVect.Sub(seg_b, seg_a);
        //    float closest_t = cpEnvironment.cpfclamp01(cpVect.Dot(seg_delta, cpVect.Sub(center, seg_a)) / seg_delta.LengthSQ);
        //    cpVect closest = cpVect.Add(seg_a, cpVect.Multiply(seg_delta, closest_t));

        //    if (circle2circleQuery(center, closest, circleShape.r, segmentShape.r, con) != null)
        //    {
        //        cpVect n = con[0].normal;

        //        // Reject endcap collisions if tangents are provided.
        //        if (
        //            (closest_t == 0.0f && cpVect.Dot(n, segmentShape.a_tangent) < 0.0) ||
        //            (closest_t == 1.0f && cpVect.Dot(n, segmentShape.b_tangent) < 0.0)
        //        ) return 0;

        //        return 1;
        //    }
        //    else
        //    {
        //        return 0;
        //    }
        //}

        //// Helper function for working with contact buffers
        //// This used to malloc/realloc memory on the fly but was repurposed.
        //public static cpContact nextContactPoint(List<cpContact> arr, int numPtr)
        //{


        //    int index = numPtr;

        //    if (index < cpArbiter.CP_MAX_CONTACTS_PER_ARBITER)
        //    {
        //        numPtr = index + 1;
        //        return arr[index];
        //    }
        //    else
        //    {
        //        return arr[cpArbiter.CP_MAX_CONTACTS_PER_ARBITER - 1];
        //    }
        //}


        //// Add contacts for probably penetrating vertexes.
        //// This handles the degenerate case where an overlap was detected, but no vertexes fall inside
        //// the opposing polygon. (like a star of david)
        //public static int findVertsFallback(List<cpContact> arr, cpPolyShape poly1, cpPolyShape poly2, cpVect n, double dist)
        //{

        //    throw new NotImplementedException("Not implemented");
        //    //int num = 0;

        //    //for (int i = 0; i < poly1.numVerts; i++)
        //    //{
        //    //    cpVect v = poly1.tVerts[i];
        //    //    if (poly2.ContainsVertPartial(v, n.Neg()))
        //    //        cpContactInit(nextContactPoint(arr, num), v, n, dist, cpEnvironment.CP_HASH_PAIR(poly1.shape.hashid, i));
        //    //}

        //    //for (int i = 0; i < poly2.numVerts; i++)
        //    //{
        //    //    cpVect v = poly2.tVerts[i];
        //    //    if (poly1.ContainsVertPartial(v, n))
        //    //        cpContactInit(nextContactPoint(arr, num), v, n, dist, cpEnvironment.CP_HASH_PAIR(poly2.shape.hashid, i));
        //    //}

        //    //return num;
        //}

        //// Add contacts for penetrating vertexes.
        //public static int findVerts(List<cpContact> arr, cpPolyShape poly1, cpPolyShape poly2, cpVect n, double dist)
        //{
        //    throw new NotImplementedException("Not implemented");

        //    //int num = 0;

        //    //for (int i = 0; i < poly1.numVerts; i++)
        //    //{
        //    //    cpVect v = poly1.tVerts[i];
        //    //    if (poly2.ContainsVert(v))
        //    //        cpContactInit(nextContactPoint(arr, num), v, n, dist, cpEnvironment.CP_HASH_PAIR(poly1.shape.hashid, i));
        //    //}

        //    //for (int i = 0; i < poly2.numVerts; i++)
        //    //{
        //    //    cpVect v = poly2.tVerts[i];
        //    //    if (cpPolyShapeContainsVert(poly1, v))
        //    //        cpContactInit(nextContactPoint(arr, num), v, n, dist, cpEnvironment.CP_HASH_PAIR(poly2.shape.hashid, i));
        //    //}

        //    //return (num != null) ? num : findVertsFallback(arr, poly1, poly2, n, dist);
        //}

        //// Collide poly shapes together.
        //public static int poly2poly(cpShape shape1, cpShape shape2, cpContact arr)
        //{
        //    throw new NotImplementedException("Not implemented");
        //    //cpPolyShape poly1 = (cpPolyShape)shape1;
        //    //cpPolyShape poly2 = (cpPolyShape)shape2;

        //    //double min1;
        //    //int mini1 = poly2.FindMSA(poly1.tPlanes, poly1.numVerts, min1);
        //    //if (mini1 == -1) return 0;

        //    //double min2;
        //    //int mini2 = poly1.FindMSA(poly2.tPlanes, poly2.numVerts, min2);
        //    //if (mini2 == -1) return 0;

        //    //// There is overlap, find the penetrating verts
        //    //if (min1 > min2)
        //    //    return findVerts(arr, poly1, poly2, poly1.tPlanes[mini1].n, min1);
        //    //else
        //    //    return findVerts(arr, poly1, poly2, poly2.tPlanes[mini2].n.Neg(), min2);
        //}

        //// Like cpPolyValueOnAxis(), but for segments.
        //public static double segValueOnAxis(cpSegmentShape seg, cpVect n, double d)
        //{
        //    double a = cpVect.Dot(n, seg.ta) - seg.r;
        //    double b = cpVect.Dot(n, seg.tb) - seg.r;
        //    return System.Math.Min(a, b) - d;
        //}

        //// Identify vertexes that have penetrated the segment.
        //public static void findPointsBehindSeg(cpContact arr, int num, cpSegmentShape seg, cpPolyShape poly, double pDist, double coef)
        //{
        //    throw new NotImplementedException("Not implemented");

        //    //double dta = cpVect.CrossProduct(seg.tn, seg.ta);
        //    //double dtb = cpVect.CrossProduct(seg.tn, seg.tb);
        //    //cpVect n = cpVect.Multiply(seg.tn, coef);

        //    //for (int i = 0; i < poly.numVerts; i++)
        //    //{
        //    //    cpVect v = poly.tVerts[i];
        //    //    if (cpVect.Dot(v, n) < cpVect.Dot(seg.tn, seg.ta) * coef + seg.r)
        //    //    {
        //    //        double dt = cpVect.CrossProduct(seg.tn, v);
        //    //        if (dta >= dt && dt >= dtb)
        //    //        {
        //    //            cpContactInit(nextContactPoint(arr, num), v, n, pDist, cpEnvironment.CP_HASH_PAIR(poly.shape.hashid, i));
        //    //        }
        //    //    }
        //    //}
        //}

        //// This one is complicated and gross. Just don't go there...
        //// TODO: Comment me!
        //public static int seg2poly(cpShape shape1, cpShape shape2, List<cpContact> arr)
        //{
        //    throw new NotImplementedException("Not implemented");

        //    //cpSegmentShape seg = (cpSegmentShape)shape1;
        //    //cpPolyShape poly = (cpPolyShape)shape2;
        //    //List<cpSplittingPlane> planes = poly.tPlanes;

        //    //double segD = cpVect.Dot(seg.tn, seg.ta);
        //    //double minNorm = poly.ValueOnAxis(seg.tn, segD) - seg.r;
        //    //double minNeg = poly.ValueOnAxis(seg.tn.Neg(), -segD) - seg.r;
        //    //if (minNeg > 0.0f || minNorm > 0.0f) return 0;

        //    //int mini = 0;
        //    //double poly_min = segValueOnAxis(seg, planes[0].n, planes[0].d);
        //    //if (poly_min > 0.0f) return 0;
        //    //for (int i = 0; i < poly.numVerts; i++)
        //    //{
        //    //    double dist = segValueOnAxis(seg, planes[i].n, planes[i].d);
        //    //    if (dist > 0.0f)
        //    //    {
        //    //        return 0;
        //    //    }
        //    //    else if (dist > poly_min)
        //    //    {
        //    //        poly_min = dist;
        //    //        mini = i;
        //    //    }
        //    //}

        //    //int num = 0;

        //    //cpVect poly_n = planes[mini].n.Neg(); // cpvneg();

        //    //cpVect va = cpVect.Add(seg.ta, cpVect.Multiply(poly_n, seg.r));
        //    //cpVect vb = cpVect.Add(seg.tb, cpVect.Multiply(poly_n, seg.r));

        //    //if (poly.ContainsVert(va))
        //    //    cpContactInit(nextContactPoint(arr, num), va, poly_n, poly_min, cpEnvironment.CP_HASH_PAIR(seg.shape.hashid, 0));
        //    //if (poly.ContainsVert(vb))
        //    //    cpContactInit(nextContactPoint(arr, num), vb, poly_n, poly_min, cpEnvironment.CP_HASH_PAIR(seg.shape.hashid, 1));

        //    //// doubleing point precision problems here.
        //    //// This will have to do for now.
        //    ////	poly_min -= cp_collision_slop; // TODO is this needed anymore?

        //    //if (minNorm >= poly_min || minNeg >= poly_min)
        //    //{
        //    //    if (minNorm > minNeg)
        //    //        findPointsBehindSeg(arr, num, seg, poly, minNorm, 1.0f);
        //    //    else
        //    //        findPointsBehindSeg(arr, num, seg, poly, minNeg, -1.0f);
        //    //}

        //    //// If no other collision points are found, try colliding endpoints.
        //    //if (num == 0)
        //    //{
        //    //    cpVect poly_a = poly.tVerts[mini];
        //    //    cpVect poly_b = poly.tVerts[(mini + 1) % poly.numVerts];

        //    //    //TODO: Revise
        //    //    if (circle2circleQuery(seg.ta, poly_a, seg.r, 0.0f) == 0) return 1;
        //    //    if (circle2circleQuery(seg.tb, poly_a, seg.r, 0.0f) == 0) return 1;
        //    //    if (circle2circleQuery(seg.ta, poly_b, seg.r, 0.0f) == 0) return 1;
        //    //    if (circle2circleQuery(seg.tb, poly_b, seg.r, 0.0f) == 0) return 1;
        //    //}

        //    //return num;
        //}

        //// This one is less gross, but still gross.
        //// TODO: Comment me!
        //public static int circle2poly(cpShape shape1, cpShape shape2, cpContact con)
        //{

        //    throw new NotImplementedException("Not implemented");
        //    //cpCircleShape circ = (cpCircleShape)shape1;
        //    //cpPolyShape poly = (cpPolyShape)shape2;
        //    //List<cpSplittingPlane> planes = poly.tPlanes;

        //    //int mini = 0;
        //    //float min = planes[0].Compare(circ.tc) - circ.r;
        //    //for (int i = 0; i < poly.numVerts; i++)
        //    //{
        //    //    double dist = planes[i].Compare(circ.tc) - circ.r;
        //    //    if (dist > 0.0f)
        //    //    {
        //    //        return 0;
        //    //    }
        //    //    else if (dist > min)
        //    //    {
        //    //        min = dist;
        //    //        mini = i;
        //    //    }
        //    //}

        //    //cpVect n = planes[mini].n;
        //    //cpVect a = poly.tVerts[mini];
        //    //cpVect b = poly.tVerts[(mini + 1) % poly.numVerts];
        //    //float dta = cpVect.CrossProduct(n, a);
        //    //float dtb = cpVect.CrossProduct(n, b);
        //    //float dt = cpVect.CrossProduct(n, circ.tc);

        //    //if (dt < dtb)
        //    //{
        //    //    return circle2circleQuery(circ.tc, b, circ.r, 0.0f, con);
        //    //}
        //    //else if (dt < dta)
        //    //{
        //    //    new cpContact(
        //    //          con,
        //    //          circ.tc.Sub(n.Multiply(circ.r + min / 2.0f)),
        //    //         n.Neg(),
        //    //          min,
        //    //          0
        //    //      );

        //    //    return 1;
        //    //}
        //    //else
        //    //{
        //    //    return circle2circleQuery(circ.tc, a, circ.r, 0.0f, con);
        //    //}
        //}

        //// Submitted by LegoCyclon
        //public static int seg2seg(cpShape shape1, cpShape shape2, cpContact con)
        //{

        //    throw new NotImplementedException("Not implemented");
        //    //cpSegmentShape seg1 = (cpSegmentShape)shape1;
        //    //cpSegmentShape seg2 = (cpSegmentShape)shape2;

        //    //cpVect v1 = cpVect.Sub(seg1.tb, seg1.ta);
        //    //cpVect v2 = cpVect.Sub(seg2.tb, seg2.ta);

        //    //double v1lsq = v1.LengthSQ;// cpVect.LengthSQ(v1);
        //    //double v2lsq = v2.LengthSQ;// cpVect.LengthSQ(v2);
        //    //// project seg2 onto seg1
        //    //cpVect p1a = seg2.ta.Sub(seg1.ta).Project(v1);  // cpVect.cpvproject(   cpVect.Sub(seg2.ta, seg1.ta), v1);
        //    //cpVect p1b = seg2.tb.Sub(seg1.ta).Project(v1); // cpVect.cpvproject(cpVect.Sub(seg2.tb, seg1.ta), v1);
        //    //// project seg1 onto seg2
        //    //cpVect p2a = seg2.ta.Sub(seg1.ta).Project(v1); //cpVect.cpvproject(cpVect.Sub(seg1.ta, seg2.ta), v2);
        //    //cpVect p2b = seg2.ta.Sub(seg1.ta).Project(v1); //cpVect.cpvproject(cpVect.Sub(seg1.tb, seg2.ta), v2);

        //    //// clamp projections to segment endcaps
        //    //if (cpVect.Dot(p1a, v1) < 0.0f)
        //    //    p1a = cpVect.ZERO;
        //    //else if (cpVect.Dot(p1a, v1) > 0.0f && p1a.LengthSQ > v1lsq)
        //    //    p1a = v1;
        //    //if (cpVect.Dot(p1b, v1) < 0.0f)
        //    //    p1b = cpVect.ZERO;
        //    //else if (cpVect.Dot(p1b, v1) > 0.0f && p1b.LengthSQ > v1lsq)
        //    //    p1b = v1;
        //    //if (cpVect.Dot(p2a, v2) < 0.0f)
        //    //    p2a = cpVect.ZERO;
        //    //else if (cpVect.Dot(p2a, v2) > 0.0f && p2a.LengthSQ > v2lsq)
        //    //    p2a = v2;
        //    //if (cpVect.Dot(p2b, v2) < 0.0f)
        //    //    p2b = cpVect.ZERO;
        //    //else if (cpVect.Dot(p2b, v2) > 0.0f && p2b.LengthSQ > v2lsq)
        //    //    p2b = v2;

        //    //p1a = cpVect.Add(p1a, seg1.ta);
        //    //p1b = cpVect.Add(p1b, seg1.ta);
        //    //p2a = cpVect.Add(p2a, seg2.ta);
        //    //p2b = cpVect.Add(p2b, seg2.ta);

        //    //int num = 0;

        //    //if (circle2circleQuery(p1a, p2a, seg1.r, seg2.r, nextContactPoint(con, num)) != 0)
        //    //    --num;

        //    //if (circle2circleQuery(p1b, p2b, seg1.r, seg2.r, nextContactPoint(con, num)) != 0)
        //    //    --num;

        //    //if (circle2circleQuery(p1a, p2b, seg1.r, seg2.r, nextContactPoint(con, num)) != 0)
        //    //    --num;

        //    //if (circle2circleQuery(p1b, p2a, seg1.r, seg2.r, nextContactPoint(con, num)) != 0)
        //    //    --num;

        //    //return num;
        //}

        //public static int cpCollideShapes(cpShape a, cpShape b, cpContact arr)
        //{
        //    // Their shape types must be in order.
        //    // cpAssertSoft(a.klass.type <= b.klass.type, "Collision shapes passed to cpCollideShapes() are not sorted.");
        //    throw new NotImplementedException("Not implemented");
        //    //collisionFunc cfunc = colfuncs[a.type + b.type * CP_NUM_SHAPES];
        //    //return (cfunc) ? cfunc(a, b, arr) : 0;
        //}

        //public static object cpCollisionBeginFunc { get; set; }
        //#endregion


    }
}
