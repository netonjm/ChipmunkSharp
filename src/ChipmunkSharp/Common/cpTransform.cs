using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{
	public class cpTransform
	{
		public static cpTransform Identity = new cpTransform(1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);

		public float a;
		public float b;
		public float c;
		public float d;
		public float tx;
		public float ty;

		public cpTransform()
		{

		}

		public cpTransform(float a, float b, float c, float d, float tx, float ty)
		{
			// TODO: Complete member initialization
			this.a = a;
			this.b = b;
			this.c = c;
			this.d = d;
			this.tx = tx;
			this.ty = ty;
		}


		/// Construct a new transform matrix.
		/// (a, b) is the x basis vector.
		/// (c, d) is the y basis vector.
		/// (tx, ty) is the translation.
		public static cpTransform New(float a, float b, float c, float d, float tx, float ty)
		{
			cpTransform t = new cpTransform(a, b, c, d, tx, ty);
			return t;
		}


		/// Construct a new transform matrix in transposed order.
		public static cpTransform NewTranspose(float a, float c, float tx, float b, float d, float ty)
		{
			cpTransform t = new cpTransform(a, b, c, d, tx, ty);
			return t;
		}

		/// Get the inverse of a transform matrix.
		public static cpTransform Inverse(cpTransform t)
		{
			float inv_det = 1.0f / (t.a * t.d - t.c * t.b);
			return NewTranspose(
			   t.d * inv_det, -t.c * inv_det, (t.c * t.ty - t.tx * t.d) * inv_det,
			  -t.b * inv_det, t.a * inv_det, (t.tx * t.b - t.a * t.ty) * inv_det
			);
		}

		/// Multiply two transformation matrices.
		public static cpTransform Mult(cpTransform t1, cpTransform t2)
		{
			return NewTranspose(
			  t1.a * t2.a + t1.c * t2.b, t1.a * t2.c + t1.c * t2.d, t1.a * t2.tx + t1.c * t2.ty + t1.tx,
			  t1.b * t2.a + t1.d * t2.b, t1.b * t2.c + t1.d * t2.d, t1.b * t2.tx + t1.d * t2.ty + t1.ty
			);
		}

		/// Transform an absolute point. (i.e. a vertex)
		public static cpVect Point(cpTransform t, cpVect p)
		{
			return new cpVect(t.a * p.x + t.c * p.y + t.tx, t.b * p.x + t.d * p.y + t.ty);
		}

		/// Transform a vector (i.e. a normal)
		public static cpVect Vect(cpTransform t, cpVect v)
		{
			return new cpVect(t.a * v.x + t.c * v.y, t.b * v.x + t.d * v.y);
		}

		/// Transform a cpBB.
		public static cpBB BB(cpTransform t, cpBB bb)
		{
			cpVect center = cpBB.Center(bb);
			float hw = (bb.r - bb.l) * 0.5f;
			float hh = (bb.t - bb.b) * 0.5f;

			float a = t.a * hw, b = t.c * hh, d = t.b * hw, e = t.d * hh;
			float hw_max = cp.cpfmax(cp.cpfabs(a + b), cp.cpfabs(a - b));
			float hh_max = cp.cpfmax(cp.cpfabs(d + e), cp.cpfabs(d - e));
			return cpBB.NewForExtents(Point(t, center), hw_max, hh_max);
		}

		/// Create a transation matrix.
		public static cpTransform Translate(cpVect translate)
		{
			return NewTranspose(
			  1.0f, 0.0f, translate.x,
			  0.0f, 1.0f, translate.y
			);
		}

		/// Create a scale matrix.
		public static cpTransform Scale(float scaleX, float scaleY)
		{
			return NewTranspose(
				scaleX, 0.0f, 0.0f,
				   0.0f, scaleY, 0.0f
			);
		}

		/// Create a rotation matrix.
		public static cpTransform Rotate(float radians)
		{
			cpVect rot = cpVect.cpvforangle(radians);
			return NewTranspose(
				rot.x, -rot.y, 0.0f,
				rot.y, rot.x, 0.0f
			);
		}

		/// Create a rigid transformation matrix. (transation + rotation)
		public static cpTransform Rigid(cpVect translate, float radians)
		{
			cpVect rot = cpVect.cpvforangle(radians);
			return NewTranspose(
				rot.x, -rot.y, translate.x,
				rot.y, rot.x, translate.y
			);
		}

		/// Fast inverse of a rigid transformation matrix.
		public static cpTransform RigidInverse(cpTransform t)
		{
			return NewTranspose(
			   t.d, -t.c, (t.c * t.ty - t.tx * t.d),
			  -t.b, t.a, (t.tx * t.b - t.a * t.ty)
			);
		}

		// Miscelaneous (but useful) transformation matrices.

		public static cpTransform Wrap(cpTransform outer, cpTransform inner)
		{
			return Mult(Inverse(outer), Mult(inner, outer));
		}

		public static cpTransform WrapInverse(cpTransform outer, cpTransform inner)
		{
			return Mult(outer, Mult(inner, Inverse(outer)));
		}

		public static cpTransform Ortho(cpBB bb)
		{
			return NewTranspose(
			  2.0f / (bb.r - bb.l), 0.0f, -(bb.r + bb.l) / (bb.r - bb.l),
			  0.0f, 2.0f / (bb.t - bb.b), -(bb.t + bb.b) / (bb.t - bb.b)
			);
		}

		public static cpTransform BoneScale(cpVect v0, cpVect v1)
		{
			cpVect d = cpVect.cpvsub(v1, v0);
			return NewTranspose(
			  d.x, -d.y, v0.x,
			  d.y, d.x, v0.y
			);
		}

		public static cpTransform AxialScale(cpVect axis, cpVect pivot, float scale)
		{
			float A = axis.x * axis.y * (scale - 1.0f);
			float B = cpVect.cpvdot(axis, pivot) * (1.0f - scale);

			return NewTranspose(
			  scale * axis.x * axis.x + axis.y * axis.y, A, axis.x * B,
			  A, axis.x * axis.x + scale * axis.y * axis.y, axis.y * B
			);
		}


	}
}
