using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{
	public class cpTransform
	{
		public static cpTransform cpTransformIdentity = new cpTransform(1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);

		public double a;
		public double b;
		public double c;
		public double d;
		public double tx;
		public double ty;

		public cpTransform()
		{

		}

		public cpTransform(double a, double b, double c, double d, double tx, double ty)
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
		public static cpTransform cpTransformNew(double a, double b, double c, double d, double tx, double ty)
		{
			cpTransform t = new cpTransform(a, b, c, d, tx, ty);
			return t;
		}


		/// Construct a new transform matrix in transposed order.
		public static cpTransform cpTransformNewTranspose(double a, double c, double tx, double b, double d, double ty)
		{
			cpTransform t = new cpTransform(a, b, c, d, tx, ty);
			return t;
		}

		/// Get the inverse of a transform matrix.
		public static cpTransform cpTransformInverse(cpTransform t)
		{
			double inv_det = 1.0f / (t.a * t.d - t.c * t.b);
			return cpTransformNewTranspose(
			   t.d * inv_det, -t.c * inv_det, (t.c * t.ty - t.tx * t.d) * inv_det,
			  -t.b * inv_det, t.a * inv_det, (t.tx * t.b - t.a * t.ty) * inv_det
			);
		}

		/// Multiply two transformation matrices.
		public static cpTransform
		cpTransformMult(cpTransform t1, cpTransform t2)
		{
			return cpTransformNewTranspose(
			  t1.a * t2.a + t1.c * t2.b, t1.a * t2.c + t1.c * t2.d, t1.a * t2.tx + t1.c * t2.ty + t1.tx,
			  t1.b * t2.a + t1.d * t2.b, t1.b * t2.c + t1.d * t2.d, t1.b * t2.tx + t1.d * t2.ty + t1.ty
			);
		}

		/// Transform an absolute point. (i.e. a vertex)
		public static cpVect cpTransformPoint(cpTransform t, cpVect p)
		{
			return cpVect.cpv(t.a * p.x + t.c * p.y + t.tx, t.b * p.x + t.d * p.y + t.ty);
		}

		/// Transform a vector (i.e. a normal)
		public static cpVect cpTransformVect(cpTransform t, cpVect v)
		{
			return cpVect.cpv(t.a * v.x + t.c * v.y, t.b * v.x + t.d * v.y);
		}

		/// Transform a cpBB.
		public static cpBB cpTransformbBB(cpTransform t, cpBB bb)
		{
			cpVect center = cpBB.Center(bb);
			double hw = (bb.r - bb.l) * 0.5f;
			double hh = (bb.t - bb.b) * 0.5f;

			double a = t.a * hw, b = t.c * hh, d = t.b * hw, e = t.d * hh;
			double hw_max = cp.cpfmax(cp.cpfabs(a + b), cp.cpfabs(a - b));
			double hh_max = cp.cpfmax(cp.cpfabs(d + e), cp.cpfabs(d - e));
			return cpBB.NewForExtents(cpTransformPoint(t, center), hw_max, hh_max);
		}

		/// Create a transation matrix.
		public static cpTransform cpTransformTranslate(cpVect translate)
		{
			return cpTransformNewTranspose(
			  1.0f, 0.0f, translate.x,
			  0.0f, 1.0f, translate.y
			);
		}

		/// Create a scale matrix.
		public static cpTransform cpTransformScale(double scaleX, double scaleY)
		{
			return cpTransformNewTranspose(
				scaleX, 0.0f, 0.0f,
				   0.0f, scaleY, 0.0f
			);
		}

		/// Create a rotation matrix.
		public static cpTransform cpTransformRotate(double radians)
		{
			cpVect rot = cpVect.cpvforangle(radians);
			return cpTransformNewTranspose(
				rot.x, -rot.y, 0.0f,
				rot.y, rot.x, 0.0f
			);
		}

		/// Create a rigid transformation matrix. (transation + rotation)
		public static cpTransform cpTransformRigid(cpVect translate, double radians)
		{
			cpVect rot = cpVect.cpvforangle(radians);
			return cpTransformNewTranspose(
				rot.x, -rot.y, translate.x,
				rot.y, rot.x, translate.y
			);
		}

		/// Fast inverse of a rigid transformation matrix.
		public static cpTransform cpTransformRigidInverse(cpTransform t)
		{
			return cpTransformNewTranspose(
			   t.d, -t.c, (t.c * t.ty - t.tx * t.d),
			  -t.b, t.a, (t.tx * t.b - t.a * t.ty)
			);
		}

		// Miscelaneous (but useful) transformation matrices.

		public static cpTransform cpTransformWrap(cpTransform outer, cpTransform inner)
		{
			return cpTransformMult(cpTransformInverse(outer), cpTransformMult(inner, outer));
		}

		public static cpTransform cpTransformWrapInverse(cpTransform outer, cpTransform inner)
		{
			return cpTransformMult(outer, cpTransformMult(inner, cpTransformInverse(outer)));
		}

		public static cpTransform cpTransformOrtho(cpBB bb)
		{
			return cpTransformNewTranspose(
			  2.0f / (bb.r - bb.l), 0.0f, -(bb.r + bb.l) / (bb.r - bb.l),
			  0.0f, 2.0f / (bb.t - bb.b), -(bb.t + bb.b) / (bb.t - bb.b)
			);
		}

		public static cpTransform cpTransformBoneScale(cpVect v0, cpVect v1)
		{
			cpVect d = cpVect.cpvsub(v1, v0);
			return cpTransformNewTranspose(
			  d.x, -d.y, v0.x,
			  d.y, d.x, v0.y
			);
		}

		public static cpTransform cpTransformAxialScale(cpVect axis, cpVect pivot, double scale)
		{
			double A = axis.x * axis.y * (scale - 1.0f);
			double B = cpVect.cpvdot(axis, pivot) * (1.0f - scale);

			return cpTransformNewTranspose(
			  scale * axis.x * axis.x + axis.y * axis.y, A, axis.x * B,
			  A, axis.x * axis.x + scale * axis.y * axis.y, axis.y * B
			);
		}


	}
}
