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

using ChipmunkSharp.Shapes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{

	public class cpSplittingPlane
	{
		public cpVect n;
		public float d;

		/// Initialize a polygon shape.
		/// The vertexes must be convex and have a clockwise winding.

		public cpSplittingPlane(cpVect n, float d)
		{
			// TODO: Complete member initialization
			this.n = n;
			this.d = d;
		}

		public float Compare(cpVect v)
		{
			return cpVect.cpvdot(n, v) - d;
		}

	}

	/// @private
	public class cpPolyShape : cpShape, ICollisionShape
	{

		public int numVerts { get { return verts.Length; } }

		//int numVerts;
		public float[] verts, tVerts;
		public cpSplittingPlane[] planes, tPlanes;

		public float r;

		public cpPolyShape(cpBody body, float[] verts, cpTransform transform,
			float radius)
			: this(body, GetVertices(transform, verts), radius)
		{

		}


		public cpPolyShape(cpBody body, float[] verts, float radius)
			: base(body, cpShapeMassInfo.cpPolyShapeMassInfo(0.0f, verts, radius))
		{

			//cp.BoxShape

			this.SetVerts(verts, cpVect.Zero);
			this.shapeType = cpShapeType.Polygon;
			this.r = radius;
		}

		public static cpPolyShape BoxShape(cpBody body, float width, float height, float radius)
		{
			var hw = width / 2;
			var hh = height / 2;

			return BoxShape2(body, new cpBB(-hw, -hh, hw, hh), radius);


		}
		public static cpPolyShape BoxShape2(cpBody body, cpBB box, float radius)
		{
			float[] verts = new float[] {
		box.l, box.b,
		box.l, box.t,
		box.r, box.t,
		box.r, box.b};

			return new cpPolyShape(body, verts, radius);
		}

		public static float[] GetVertices(cpTransform transform, float[] verts)
		{

			float[] hullVerts = new float[verts.Length];
			cpVect tmp;
			int count = (verts.Length / 2);
			for (int i = 0; i < count - 1; i++)
			{
				tmp = cpTransform.cpTransformPoint(transform, new cpVect(verts[i], verts[i + 1]));
				hullVerts[i] = tmp.x;
				hullVerts[i + 1] = tmp.y;
			}
			return hullVerts;
		}

		public void SetVerts(float[] verts, cpVect offset)
		{
			cp.assert(verts.Length >= 4, "Polygons require some verts");
			//cpEnvironment.assert(typeof(verts[0]) == 'number',
			//        'Polygon verticies should be specified in a flattened list (eg [x1,y1,x2,y2,x3,y3,...])');

			// Fail if the user attempts to pass a concave poly, or a bad winding.
			cp.assert(cp.polyValidate(verts), "Polygon is concave or has a reversed winding. Consider using cpConvexHull()");

			var len = verts.Length;
			var numVerts = len >> 1;

			// This a pretty bad way to do this in javascript. As a first pass, I want to keep
			// the code similar to the C.
			this.verts = new float[len];
			this.tVerts = new float[len];
			this.planes = new cpSplittingPlane[numVerts];
			this.tPlanes = new cpSplittingPlane[numVerts];

			for (var i = 0; i < len; i += 2)
			{
				//var a = vadd(offset, verts[i]);
				//var b = vadd(offset, verts[(i+1)%numVerts]);
				var ax = verts[i] + offset.x;
				var ay = verts[i + 1] + offset.y;
				var bx = verts[(i + 2) % len] + offset.x;
				var by = verts[(i + 3) % len] + offset.y;

				// Inefficient, but only called during object initialization.
				var n = cpVect.cpvnormalize(cpVect.cpvperp(new cpVect(bx - ax, by - ay)));

				this.verts[i] = ax;
				this.verts[i + 1] = ay;
				this.planes[i >> 1] = new cpSplittingPlane(n, cpVect.cpvdot2(n.x, n.y, ax, ay));
				this.tPlanes[i >> 1] = new cpSplittingPlane(new cpVect(0, 0), 0);
			}
		}

		public void TransformVerts(cpVect p, cpVect rot)
		{
			var src = this.verts;
			var dst = this.tVerts;

			float l = cp.Infinity, r = -cp.Infinity;
			float b = cp.Infinity, t = -cp.Infinity;

			for (var i = 0; i < src.Length; i += 2)
			{
				//var v = vadd(p, vrotate(src[i], rot));
				var x = src[i];
				var y = src[i + 1];

				var vx = p.x + x * rot.x - y * rot.y;
				var vy = p.y + x * rot.y + y * rot.x;

				//console.log('(' + x + ',' + y + ') -> (' + vx + ',' + vy + ')');

				dst[i] = vx;
				dst[i + 1] = vy;

				l = Math.Min(l, vx);
				r = Math.Max(r, vx);
				b = Math.Min(b, vy);
				t = Math.Max(t, vy);
			}

			this.bb_l = l;
			this.bb_b = b;
			this.bb_r = r;
			this.bb_t = t;
		}

		public void TransformAxes(cpVect p, cpVect rot)
		{
			var src = this.planes;
			var dst = this.tPlanes;

			for (var i = 0; i < src.Length; i++)
			{
				var n = cpVect.cpvrotate(src[i].n, rot);
				dst[i].n = n;
				dst[i].d = cpVect.cpvdot(p, n) + src[i].d;
			}
		}

		public float ValueOnAxis(cpVect n, float d)
		{
			var verts = this.tVerts;
			var m = cpVect.cpvdot2(n.x, n.y, verts[0], verts[1]);

			for (var i = 2; i < verts.Length; i += 2)
			{
				m = Math.Min(m, cpVect.cpvdot2(n.x, n.y, verts[i], verts[i + 1]));
			}

			return m - d;
		}

		public bool ContainsVert(float vx, float vy)
		{
			var planes = this.tPlanes;

			for (var i = 0; i < planes.Length; i++)
			{
				var n = planes[i].n;
				var dist = cpVect.cpvdot2(n.x, n.y, vx, vy) - planes[i].d;
				if (dist > 0) return false;
			}

			return true;
		}

		public bool ContainsVertPartial(float vx, float vy, cpVect n)
		{
			var planes = this.tPlanes;

			for (var i = 0; i < planes.Length; i++)
			{
				var n2 = planes[i].n;
				if (cpVect.cpvdot(n2, n) < 0) continue;
				var dist = cpVect.cpvdot2(n2.x, n2.y, vx, vy) - planes[i].d;
				if (dist > 0) return false;
			}

			return true;
		}

		public int GetNumVerts()
		{
			return this.verts.Length / 2;
		}

		// These methods are provided for API compatibility with Chipmunk. I recommend against using
		// them - just access the poly.verts list directly.
		public cpVect GetVert(int i)
		{
			return new cpVect(this.verts[i * 2], this.verts[i * 2 + 1]);
		}

		public List<cpVect> GetVers()
		{
			List<cpVect> dev = new List<cpVect>();
			for (int i = 0; i < GetNumVerts(); i++)
				dev.Add(GetVert(i));
			return dev;
		}

		public override void Draw(cpDebugDraw m_debugDraw)
		{

			var len = tVerts.Count();

			List<cpVect> vertices = new List<cpVect>();

			var lastPoint = new cpVect(tVerts[len - 2], tVerts[len - 1]);

			cpColor color = cp.GetShapeColor(this);

			for (var i = 0; i < len; i += 2)
			{
				var p = new cpVect(tVerts[i], tVerts[i + 1]);
				m_debugDraw.DrawSegment(lastPoint, p, color);
				lastPoint = p;
			}

		}


		#region OBSOLETE

		[Obsolete("This method was obsolete from Chipmunk JS")]
		public override void CacheData(cpVect p, cpVect rot)
		{
			this.TransformAxes(p, rot);
			this.TransformVerts(p, rot);
		}

		[Obsolete("This method was obsolete from Chipmunk JS")]
		public override cpPointQueryInfo NearestPointQuery(cpVect p)
		{
			var planes = this.tPlanes;
			var verts = this.tVerts;

			var v0x = verts[verts.Length - 2];
			var v0y = verts[verts.Length - 1];
			var minDist = cp.Infinity;
			var closestPoint = cpVect.Zero;
			var outside = false;

			for (var i = 0; i < planes.Length; i++)
			{
				if (planes[i].Compare(p) > 0) outside = true;

				var v1x = verts[i * 2];
				var v1y = verts[i * 2 + 1];
				var closest = cp.closestPointOnSegment2(p.x, p.y, v0x, v0y, v1x, v1y);

				var dist = cpVect.cpvdist(p, closest);
				if (dist < minDist)
				{
					minDist = dist;
					closestPoint = closest;
				}

				v0x = v1x;
				v0y = v1y;
			}

			return new cpPointQueryInfo(this, closestPoint, (outside ? minDist : -minDist), cpVect.Zero);
		}

		[Obsolete("This method was obsolete from Chipmunk JS")]
		public override cpSegmentQueryInfo SegmentQuery(cpVect a, cpVect b)
		{
			var axes = this.tPlanes;
			var verts = this.tVerts;
			var numVerts = axes.Length;
			var len = numVerts * 2;

			for (var i = 0; i < numVerts; i++)
			{
				var n = axes[i].n;
				var an = cpVect.cpvdot(a, n);
				if (axes[i].d > an) continue;

				var bn = cpVect.cpvdot(b, n);
				var t = (axes[i].d - an) / (bn - an);
				if (t < 0 || 1 < t) continue;

				var point = cpVect.cpvlerp(a, b, t);
				var dt = -cpVect.cpvcross(n, point);
				var dtMin = -cpVect.cpvcross2(n.x, n.y, verts[i * 2], verts[i * 2 + 1]);
				var dtMax = -cpVect.cpvcross2(n.x, n.y, verts[(i * 2 + 2) % len], verts[(i * 2 + 3) % len]);

				if (dtMin <= dt && dt <= dtMax)
				{
					// josephg: In the original C code, this function keeps
					// looping through axes after finding a match. I *think*
					// this code is equivalent...
					return new cpSegmentQueryInfo(this, n, cpVect.Zero, t);
				}
			}
			return null;
		}


		#endregion


		public Func<object, object, List<ContactPoint>>[] CollisionTable
		{
			get
			{
				return new Func<object, object, List<ContactPoint>>[] {
                    null,
                    null,
                    (o1,o2) => cpCollision.Poly2Poly(o1 as cpPolyShape ,o2 as cpPolyShape)
                };
			}
		}

		public int CollisionCode
		{
			get { return 2; }
		}


	}

}
