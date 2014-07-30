
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

	public class cpVect
	{

		public static cpVect Zero
		{
			get
			{
				return new cpVect(0, 0);
			}
		}


		#region PROPERTIES

		public float x { get; set; }
		public float y { get; set; }

		#endregion


		public cpVect(float _x, float _y)
		{
			x = _x;
			y = _y;
		}

		public cpVect(cpVect pt)
		{
			x = pt.x;
			y = pt.y;
		}

		public static bool Equal(cpVect point1, cpVect point2)
		{
			return ((point1.x == point2.x) && (point1.y == point2.y));
		}

		public cpVect Offset(float dx, float dy)
		{
			return new cpVect(
				 x + dx,
				 y + dy

				);
		}

		public cpVect Reverse
		{
			get { return new cpVect(-x, -y); }
		}

		public override int GetHashCode()
		{
			return x.GetHashCode() + y.GetHashCode();
		}

		public override bool Equals(object obj)
		{
			if (ReferenceEquals(this, obj)) return true;
			return Equals(obj as cpVect);
		}

		public bool Equals(cpVect p)
		{
			if (p == null) return false;
			return x == p.x && y == p.y;
		}

		public override string ToString()
		{
			return string.Format("cpVect : ({0:N3} {1:N3})", x, y);
		}

		public float DistanceSQ(cpVect v2)
		{
			return Sub(v2).LengthSQ;
		}

		public static float DistanceSQ(cpVect v1, cpVect v2)
		{
			return v1.Sub(v2).LengthSQ;
		}

		//public static float Distance(cpVect v1, cpVect v2)
		//{
		//    return (v1 - v2).Length;
		//}

		public float Distance(cpVect v2)
		{
			return Distance(this, v2);
		}

		public static float Distance(cpVect v1, cpVect v2)
		{
			return (float)Math.Sqrt(v1.Sub(v2).LengthSQ);
		}

		public static cpVect Multiply(cpVect v1, float v)
		{
			return new cpVect(
				v1.x * v,
				v1.y * v
				);
		}

		public cpVect Clamp(float len)
		{
			return cpvclamp(this, len);
		}

		public cpVect Add(cpVect v1)
		{
			return Add(this, v1);
		}

		public static cpVect Add(cpVect v1, cpVect v2)
		{
			return new cpVect(
				v1.x + v2.x,
				v1.y + v2.y
				);
		}

		public cpVect Sub(cpVect v2)
		{
			return new cpVect(
				 x - v2.x,
				 y - v2.y
				);
		}

		public static cpVect Sub(cpVect v1, cpVect v2)
		{
			return new cpVect(
				v1.x - v2.x,
				v1.y - v2.y
				);
		}

		public float LengthSQ
		{
			get { return x * x + y * y; }
		}

		public float LengthSquare
		{
			get { return LengthSQ; }
		}

		/// <summary>
		///     Computes the length of this point as if it were a vector with XY components relative to the
		///     origin. This is computed each time this property is accessed, so cache the value that is
		///     returned.
		/// </summary>
		public float Length
		{
			get { return (float)System.Math.Sqrt(x * x + y * y); }
		}

		/// <summary>
		///     Inverts the direction or location of the Y component.
		/// </summary>
		public cpVect InvertY
		{
			get
			{
				return new cpVect(
					 x,
					 -y
					);
			}
		}

		///// <summary>
		/////     Normalizes the components of this point (convert to mag 1), and returns the orignial
		/////     magnitude of the vector defined by the XY components of this point.
		///// </summary>
		///// <returns></returns>
		//public float Normalize()
		//{
		//    var mag = (float)System.Math.Sqrt(x * x + y * y);
		//    if (mag < float.Epsilon)
		//    {
		//        return (0f);
		//    }
		//    float l = 1f / mag;
		//    x *= l;
		//    y *= l;
		//    return (mag);
		//}

		#region Static Methods

		public static cpVect Lerp(cpVect a, cpVect b, float alpha)
		{
			return (a * (1f - alpha) + b * alpha);
		}

		public cpVect Lerp(cpVect a, float alpha)
		{
			return Lerp(this, a, alpha);
		}


		/** @returns if points have fuzzy equality which means equal with some degree of variance.
			@since v0.99.1
		*/

		public static bool FuzzyEqual(cpVect a, cpVect b, float variance)
		{
			if (a.x - variance <= b.x && b.x <= a.x + variance)
				if (a.y - variance <= b.y && b.y <= a.y + variance)
					return true;

			return false;
		}


		/** Multiplies a nd b components, a.x*b.x, a.y*b.y
			@returns a component-wise multiplication
			@since v0.99.1
		*/

		public static cpVect MultiplyComponents(cpVect a, cpVect b)
		{
			return new cpVect(
				a.x * b.x,
				a.y * b.y
				);
		}

		/** @returns the signed angle in radians between two vector directions
			@since v0.99.1
		*/

		public static float AngleSigned(cpVect a, cpVect b)
		{
			cpVect a2 = Normalize(a);
			cpVect b2 = Normalize(b);
			var angle = (float)System.Math.Atan2(a2.x * b2.y - a2.y * b2.x, DotProduct(a2, b2));

			if (System.Math.Abs(angle) < float.Epsilon)
			{
				return 0.0f;
			}

			return angle;
		}

		/** Rotates a point counter clockwise by the angle around a pivot
			@param v is the point to rotate
			@param pivot is the pivot, naturally
			@param angle is the angle of rotation cw in radians
			@returns the rotated point
			@since v0.99.1
		*/

		public static cpVect RotateByAngle(cpVect v, cpVect pivot, float angle)
		{
			cpVect r = v - pivot;
			float cosa = (float)Math.Cos(angle), sina = (float)Math.Sin(angle);
			float t = r.x;

			r.x = t * cosa - r.y * sina + pivot.x;
			r.y = t * sina + r.y * cosa + pivot.y;

			return r;
		}

		/** A general line-line intersection test
		 @param p1 
			is the startpoint for the first line P1 = (p1 - p2)
		 @param p2 
			is the endpoint for the first line P1 = (p1 - p2)
		 @param p3 
			is the startpoint for the second line P2 = (p3 - p4)
		 @param p4 
			is the endpoint for the second line P2 = (p3 - p4)
		 @param s 
			is the range for a hitpoint in P1 (pa = p1 + s*(p2 - p1))
		 @param t
			is the range for a hitpoint in P3 (pa = p2 + t*(p4 - p3))
		 @return bool 
			indicating successful intersection of a line
			note that to truly test intersection for segments we have to make 
			sure that s & t lie within [0..1] and for rays, make sure s & t > 0
			the hit point is		p3 + t * (p4 - p3);
			the hit point also is	p1 + s * (p2 - p1);
		 @since v0.99.1
		 */

		public static bool LineIntersect(cpVect A, cpVect B, cpVect C, cpVect D, float S, float T)
		{
			// FAIL: Line undefined
			if ((A.x == B.x && A.y == B.y) || (C.x == D.x && C.y == D.y))
			{
				return false;
			}

			float BAx = B.x - A.x;
			float BAy = B.y - A.y;
			float DCx = D.x - C.x;
			float DCy = D.y - C.y;
			float ACx = A.x - C.x;
			float ACy = A.y - C.y;

			float denom = DCy * BAx - DCx * BAy;

			S = DCx * ACy - DCy * ACx;
			T = BAx * ACy - BAy * ACx;

			if (denom == 0)
			{
				if (S == 0 || T == 0)
				{
					// Lines incident
					return true;
				}
				// Lines parallel and not incident
				return false;
			}

			S = S / denom;
			T = T / denom;

			// Point of intersection
			// CGPoint P;
			// P.x = A.x + *S * (B.x - A.x);
			// P.y = A.y + *S * (B.y - A.y);

			return true;
		}

		/*
		ccpSegmentIntersect returns YES if Segment A-B intersects with segment C-D
		@since v1.0.0
		*/

		public static bool SegmentIntersect(cpVect A, cpVect B, cpVect C, cpVect D)
		{
			float S = 0, T = 0;

			if (LineIntersect(A, B, C, D, S, T)
				&& (S >= 0.0f && S <= 1.0f && T >= 0.0f && T <= 1.0f))
			{
				return true;
			}

			return false;
		}

		/*
		ccpIntersectPoint returns the intersection point of line A-B, C-D
		@since v1.0.0
		*/

		public static cpVect IntersectPoint(cpVect A, cpVect B, cpVect C, cpVect D)
		{
			float S = 0, T = 0;

			if (LineIntersect(A, B, C, D, S, T))
			{
				// Point of intersection
				return new cpVect(
					 A.x + S * (B.x - A.x),
					 A.y + S * (B.y - A.y)

					);
			}

			return Zero;
		}

		/** Converts radians to a normalized vector.
			@return cpVect
			@since v0.7.2
		*/

		public static cpVect ForAngle(float a)
		{
			return new cpVect(
				 (float)Math.Cos(a),
				 (float)Math.Sin(a)

				);
			//            return CreatePoint((float)Math.Cos(a), (float)Math.Sin(a));
		}

		/** Converts a vector to radians.
			@return CGfloat
			@since v0.7.2
		*/

		public static float ToAngle(cpVect v)
		{
			return (float)Math.Atan2(v.y, v.x);
		}


		/** Clamp a point between from and to.
			@since v0.99.1
		*/

		public static cpVect Clamp(cpVect p, cpVect from, cpVect to)
		{
			return new cpVect(
				cp.cpclamp(p.x, from.x, to.x),
			   cp.cpclamp(p.y, from.y, to.y)
				);
			//            return CreatePoint(Clamp(p.x, from.x, to.x), Clamp(p.y, from.y, to.y));
		}

		/** Quickly convert CCSize to a cpVect
			@since v0.99.1
		*/

		//[Obsolete("Use explicit cast (cpVect)size.")]
		//public static cpVect FromSize(CCSize s)
		//{
		//    return new cpVect(
		//         s.Width,
		//         s.Height
		//        );
		//}

		/**
		 * Allow Cast CCSize to cpVect
		 */

		//public static explicit operator cpVect(CCSize size)
		//{
		//    return new cpVect(size.Width, size.Height);
		//}

		public cpVect Perp()
		{
			return Perp(this);
		}

		public static cpVect Perp(cpVect p)
		{
			return new cpVect(-p.y, p.x);
		}

		public static float Dot(cpVect p1, cpVect p2)
		{
			return p1.x * p2.x + p1.y * p2.y;
		}

		public float Dot(cpVect p1)
		{
			return Dot(this, p1);
		}

		public static cpVect Normalize(cpVect p)
		{
			float x = p.x;
			float y = p.y;
			float l = 1f / (float)Math.Sqrt(x * x + y * y);
			return new cpVect(x * l, y * l);
		}

		public cpVect Normalize()
		{
			return Normalize(this);
		}

		public static cpVect Midpoint(cpVect p1, cpVect p2)
		{
			return new cpVect(
				(p1.x + p2.x) / 2f,
				 (p1.y + p2.y) / 2f
				);
		}

		public static float DotProduct(cpVect v1, cpVect v2)
		{
			return v1.x * v2.x + v1.y * v2.y;
		}

		/** Calculates cross product of two points.
			@return CGfloat
			@since v0.7.2
		*/

		public static float CrossProduct(cpVect v1, cpVect v2)
		{
			return v1.x * v2.y - v1.y * v2.x;
		}

		public float CrossProduct(cpVect v1)
		{
			return CrossProduct(this, v1);
		}

		/** Calculates perpendicular of v, rotated 90 degrees counter-clockwise -- cross(v, perp(v)) >= 0
			@return cpVect
			@since v0.7.2
		*/

		public static cpVect PerpendicularCounterClockwise(cpVect v)
		{
			return new cpVect(-v.y, v.x);
		}

		/** Calculates perpendicular of v, rotated 90 degrees clockwise -- cross(v, rperp(v)) <= 0
			@return cpVect
			@since v0.7.2
		*/

		public static cpVect PerpendicularClockwise(cpVect v)
		{
			return new cpVect(v.y, -v.x);
		}

		/** Calculates the projection of v1 over v2.
			@return cpVect
			@since v0.7.2
		*/

		public cpVect Project(cpVect v1)
		{
			return Project(this, v1);
		}

		static cpVect Project(cpVect v1, cpVect v2)
		{
			float dp1 = v1.x * v2.x + v1.y * v2.y;
			float dp2 = v2.LengthSQ;
			float f = dp1 / dp2;
			return new cpVect(v2.x * f, v2.y * f);
			// return Multiply(v2, DotProduct(v1, v2) / DotProduct(v2, v2));
		}

		/** Rotates two points.
			@return cpVect
			@since v0.7.2
		*/

		public cpVect Rotate(cpVect v1)
		{
			return Rotate(this, v1);
		}

		public static cpVect Rotate(cpVect v1, cpVect v2)
		{
			return new cpVect(
				v1.x * v2.x - v1.y * v2.y,
				v1.x * v2.y + v1.y * v2.x);
		}

		/** Unrotates two points.
			@return cpVect
			@since v0.7.2
		*/

		public static cpVect Unrotate(cpVect v1, cpVect v2)
		{
			return new cpVect(
				v1.x * v2.x + v1.y * v2.y,
				v1.y * v2.x - v1.x * v2.y
				);
		}

		#endregion

		#region Operator Overloads

		public static bool operator ==(cpVect p1, cpVect p2)
		{
			return Equals(p1, p2);
		}

		public static bool operator !=(cpVect p1, cpVect p2)
		{
			return !Equals(p1, p2);
		}

		public static cpVect operator -(cpVect p1, cpVect p2)
		{
			return new cpVect(
				p1.x - p2.x,
				p1.y - p2.y
				); ;
		}

		public static cpVect operator -(cpVect p1)
		{
			return new cpVect(
				 -p1.x,
				 -p1.y
				);
		}

		public static cpVect operator +(cpVect p1, cpVect p2)
		{
			return new cpVect(
				p1.x + p2.x,
				p1.y + p2.y
				);
		}

		public static cpVect operator +(cpVect p1)
		{
			return new cpVect(
				+p1.x,
				+p1.y
				);
		}

		public static cpVect operator *(cpVect p, float value)
		{
			return new cpVect(
				p.x * value,
				p.y * value
				);
		}

		#endregion

		public cpVect Neg()
		{
			return new cpVect(-x, -y);
		}

		public cpVect Multiply(float p)
		{
			return Multiply(this, p);
		}


		#region POINT OPERATIONS NOT NORMALICED

		/// Convenience constructor for cpVect structs.
		public static cpVect cpv(float x, float y)
		{
			return new cpVect(x, y);
		}

		/// Check if two vectors are equal. (Be careful when comparing floating point numbers!)
		/// 
		public static bool cpveql(cpVect v1, cpVect v2)
		{
			return (v1.x == v2.x && v1.y == v2.y);
		}

		/// Add two vectors
		public static cpVect cpvadd(cpVect v1, cpVect v2)
		{
			return cpv(v1.x + v2.x, v1.y + v2.y);
		}

		/// Subtract two vectors.
		public static cpVect cpvsub(cpVect v1, cpVect v2)
		{
			return cpv(v1.x - v2.x, v1.y - v2.y);
		}

		/// Negate a vector.
		public static cpVect cpvneg(cpVect v)
		{
			return cpv(-v.x, -v.y);
		}

		/// Scalar multiplication.
		public static cpVect cpvmult(cpVect v, float s)
		{
			return cpv(v.x * s, v.y * s);
		}

		public static float cpvdot(cpVect v1, cpVect v2)
		{
			return v1.x * v2.x + v1.y * v2.y;
		}

		/// Vector dot product.
		public static float cpvdot2(float x1, float y1, float x2, float y2)
		{
			return x1 * x2 + y1 * y2;
		}

		/// 2D vector cross product analog.
		/// The cross product of 2D vectors results in a 3D vector with only a z component.
		/// This function returns the magnitude of the z value.
		public static float cpvcross(cpVect v1, cpVect v2)
		{
			return v1.x * v2.y - v1.y * v2.x;
		}

		public static float cpvcross2(float x1, float y1, float x2, float y2)
		{
			return x1 * y2 - y1 * x2;
			//return v1.x * v2.y - v1.y * v2.x;
		}

		/// Returns a perpendicular vector. (-90 degree rotation)


		/// Returns a perpendicular vector. (90 degree rotation)
		public static cpVect cpvperp(cpVect v)
		{
			return cpv(-v.y, v.x);
		}

		/// Returns a perpendicular vector. (90 degree rotation)
		public static cpVect cpvrperp(cpVect v)
		{
			return cpv(v.y, -v.x);
		}


		/// Returns the vector projection of v1 onto v2.
		public static cpVect cpvproject(cpVect v1, cpVect v2)
		{
			return cpvmult(v2, cpvdot(v1, v2) / cpvdot(v2, v2));
		}

		/// Returns the unit length vector for the given angle (in radians).
		public static cpVect cpvforangle(float a)
		{
			return cpv(cp.cpfcos(a), cp.cpfsin(a));
		}


		/// Uses complex number multiplication to rotate v1 by v2. Scaling will occur if v1 is not a unit vector.
		public static cpVect cpvrotate(cpVect v1, cpVect v2)
		{
			return cpv(v1.x * v2.x - v1.y * v2.y, v1.x * v2.y + v1.y * v2.x);
		}

		/// Inverse of cpvrotate().
		public static cpVect cpvunrotate(cpVect v1, cpVect v2)
		{
			return cpv(v1.x * v2.x + v1.y * v2.y, v1.y * v2.x - v1.x * v2.y);
		}

		//static float cpvlength(cpVect v)
		//{
		//    return v.Length;
		//}

		public static float cpvlength(cpVect v)
		{
			return (float)Math.Sqrt(v.Dot(v));
			//return v.Length;
		}

		/// Returns the squared length of v. Faster than cpvlength() when you only need to compare lengths.
		public static float cpvlengthsq(cpVect v)
		{
			return cpvdot(v, v);
		}

		/// Linearly interpolate between v1 and v2.
		public static cpVect cpvlerp(cpVect v1, cpVect v2, float t)
		{
			return cpvadd(cpvmult(v1, 1.0f - t), cpvmult(v2, t));
		}

		public static cpVect cpvnormalize(cpVect v)
		{
			return cpvmult(v, 1.0f / cpvlength(v));
		}

		/// Returns a normalized copy of v.
		//static cpVect cpvnormalize2(cpVect v)
		//{
		//    // Neat trick I saw somewhere to avoid div/0.
		//    return cpvmult(v, 1.0f / (cpvlength(v) + float_MIN));
		//}

		/// Spherical linearly interpolate between v1 and v2.
		public static cpVect cpvslerp(cpVect v1, cpVect v2, float t)
		{
			float dot = cpVect.Dot(cpVect.Normalize(v1), cpvnormalize(v2));
			float omega = (float)System.Math.Acos(cp.cpfclamp(dot, -1.0f, 1.0f));

			if (omega < 1e-3)
			{
				// If the angle between two vectors is very small, lerp instead to avoid precision issues.
				return cpvlerp(v1, v2, t);
			}
			else
			{
				float denom = 1.0f / (float)Math.Sin(omega);
				return cpVect.Add(cpVect.Multiply(v1, (float)System.Math.Sin((1.0f - t) * omega) * denom), cpVect.Multiply(v2, (float)System.Math.Sin(t * omega) * denom));
			}
		}

		/// Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
		public static cpVect cpvslerpconst(cpVect v1, cpVect v2, float a)
		{
			float dot = cpVect.Dot(cpvnormalize(v1), cpvnormalize(v2));
			float omega = (float)System.Math.Acos(cp.cpfclamp(dot, -1.0f, 1.0f));

			return cpvslerp(v1, v2, System.Math.Min(a, omega) / omega);
		}

		/// Clamp v to length len.
		public static cpVect cpvclamp(cpVect v, float len)
		{
			return (cpvdot(v, v) > len * len) ? cpvmult(cpvnormalize(v), len) : v;
		}

		/// Linearly interpolate between v1 towards v2 by distance d.
		public static cpVect cpvlerpconst(cpVect v1, cpVect v2, float d)
		{
			return cpvadd(v1, cpvclamp(cpvsub(v2, v1), d));
		}

		/// Returns the distance between v1 and v2.
		public static float cpvdist(cpVect v1, cpVect v2)
		{
			return cpvlength(cpvsub(v1, v2));
		}

		/// Returns the squared distance between v1 and v2. Faster than cpvdist() when you only need to compare distances.
		public static float cpvdistsq(cpVect v1, cpVect v2)
		{
			return cpvlengthsq(cpvsub(v1, v2));
		}

		/// Returns true if the distance between v1 and v2 is less than dist.
		public static bool cpvnear(cpVect v1, cpVect v2, float dist)
		{
			return cpvdistsq(v1, v2) < dist * dist;
		}

		/// @}

		/// @defgroup cpMat2x2 cpMat2x2
		/// 2x2 matrix type used for tensors and such.
		/// @{

		// NUKE
		public static cpMat2x2 cpMat2x2New(float a, float b, float c, float d)
		{
			return new cpMat2x2(a, b, c, d);
		}

		public static cpVect cpMat2x2Transform(cpMat2x2 m, cpVect v)
		{
			return cpv(v.x * m.a + v.y * m.b, v.x * m.c + v.y * m.d);
		}

		public static cpVect cpvnormalize_safe(cpVect v)
		{
			return (v.x == 0.0f && v.y == 0.0f ? cpVect.Zero : cpvnormalize(v));
		}


		public static string cpvstr(cpVect v)
		{
			return (string.Format("({0:N3}, {1:N3})", v.x, v.y));
		}

		public static float cplength2(float v1, float v2)
		{
			//TODO: Revisar si es correcta
			return Math.Abs(v1 - v2);
		}

		public static cpVect closestPointOnSegment(cpVect p, cpVect a, cpVect b)
		{
			var delta = cpvsub(a, b);
			var t = cp.cpfclamp01(cpvdot(delta, cpvsub(p, b)) / cpvlengthsq(delta));
			return cpvadd(b, cpvmult(delta, t));
		}


		/// Returns a normalized copy of v or vzero if v was already vzero. Protects against divide by zero errors.
		public static cpVect vnormalize_safe(cpVect v)
		{
			return (v.x == 0 && v.y == 0 ? cpVect.Zero : v.Normalize());
		}

		/// Clamp v to length len.
		public static cpVect vnormalize_safe(cpVect v, float len)
		{
			return (cpvdot(v, v) > len * len) ? cpvmult(cpvnormalize(v), len) : v;
		}

		/// Returns a normalized copy of v.
		public static cpVect vnormalize(cpVect v)
		{
			return cpvmult(v, 1 / cpvlength(v));
		}

		/// Linearly interpolate between v1 and v2.
		public static cpVect vlerp(cpVect v1, cpVect v2, float t)
		{
			return cpvadd(cpvmult(v1, 1 - t), cpvmult(v2, t));
		}

		/// Returns the squared length of v. Faster than vlength() when you only need to compare lengths.
		public static float vlengthsq2(float x, float y)
		{
			return x * x + y * y;
		}

		//public static float vlengthsq2(float x, float y)
		//{
		//    return x * x + y * y;
		//}

		/// Linearly interpolate between v1 towards v2 by distance d.
		public static cpVect vlerpconst(cpVect v1, cpVect v2, float d)
		{
			return v1.Add(v2.Sub(v1).Clamp(d));
			//return cpVect.cpvadd(v1, cpvclamp(cpvsub(v2, v1), d));
		}

		/// Returns the squared distance between v1 and v2. Faster than vdist() when you only need to compare distances.
		public static float vdistsq(cpVect v1, cpVect v2)
		{
			return v1.Sub(v2).LengthSQ;
			//return cpvlengthsq(cpvsub(v1, v2));
		}

		/// Returns true if the distance between v1 and v2 is less than dist.
		public static bool vnear(cpVect v1, cpVect v2, float dist)
		{
			return v1.DistanceSQ(v2) < dist * dist;
		}

		public static cpVect mult_k(cpVect vr, cpVect k1, cpVect k2)
		{
			return new cpVect(vr.Dot(k1), vr.Dot(k2));
		}

		#endregion

		public cpVect NormalizeSafe()
		{
			return vnormalize_safe(this);
			//throw new NotImplementedException();
		}

		public cpVect getMidpoint(cpVect b)
		{
			return Midpoint(this, b);
		}
	}


}

