
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
			return new cpVect(cp.cpfcos(a), cp.cpfsin(a));
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

		public static float cpvlength(cpVect v)
		{
			return cp.cpfsqrt(cpvdot(v, v));
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

		/// Spherical linearly interpolate between v1 and v2.
		public static cpVect cpvslerp(cpVect v1, cpVect v2, float t)
		{
			float dot = cpvdot(cpvnormalize(v1), cpvnormalize(v2));
			float omega = cp.cpfacos(cp.cpfclamp(dot, -1.0f, 1.0f));

			if (omega < 1e-3)
			{
				// If the angle between two vectors is very small, lerp instead to avoid precision issues.
				return cpvlerp(v1, v2, t);
			}
			else
			{
				float denom = 1.0f / cp.cpfsin(omega);
				return cpvadd(cpvmult(v1, cp.cpfsin((1.0f - t) * omega) * denom), cpvmult(v2, cp.cpfsin(t * omega) * denom));
			}
		}

		/// Spherical linearly interpolate between v1 towards v2 by no more than angle a radians
		public static cpVect cpvslerpconst(cpVect v1, cpVect v2, float a)
		{
			float dot = cpvdot(cpvnormalize(v1), cpvnormalize(v2));
			float omega = cp.cpfacos(cp.cpfclamp(dot, -1.0f, 1.0f));

			return cpvslerp(v1, v2, cp.cpfmin(a, omega) / omega);
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
			return cp.cpfabs(v1 - v2);
		}


		/// Returns a normalized copy of v.
		public static cpVect vnormalize(cpVect v)
		{
			return cpvmult(v, 1.0f / cpvlength(v));
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

		/// Linearly interpolate between v1 towards v2 by distance d.
		public static cpVect vlerpconst(cpVect v1, cpVect v2, float d)
		{
			return cpvadd(v1, cpvclamp(cpvsub(v2, v1), d));
		}

		/// Returns the squared distance between v1 and v2. Faster than vdist() when you only need to compare distances.
		public static float vdistsq(cpVect v1, cpVect v2)
		{
			return cpvlengthsq(cpvsub(v1, v2));
		}
	

		#endregion

		public static float cpvtoangle(cpVect v)
		{
			return cp.cpfatan2(v.y, v.x);
		}

	}


}

