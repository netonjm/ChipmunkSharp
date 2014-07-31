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

	public class cpPinJoint : cpConstraint
	{


		internal cpVect anchorA, anchorB;
		internal double dist;

		internal cpVect r1, r2;
		internal cpVect n;
		internal double nMass;

		internal double jnAcc;
		internal double bias;



		public override void PreStep(double dt)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			this.r1 = cpTransform.cpTransformVect(a.transform, cpVect.cpvsub(this.anchorA, a.cog));
			this.r2 = cpTransform.cpTransformVect(b.transform, cpVect.cpvsub(this.anchorB, b.cog));

			cpVect delta = cpVect.cpvsub(cpVect.cpvadd(b.p, this.r2), cpVect.cpvadd(a.p, this.r1));
			double dist = cpVect.cpvlength(delta);
			this.n = cpVect.cpvmult(delta, 1.0f / (dist > 0 ? dist : cp.Infinity));

			// calculate mass normal
			this.nMass = 1.0f / cp.k_scalar(a, b, this.r1, this.r2, this.n);

			// calculate bias velocity
			double maxBias = this.maxBias;
			this.bias = cp.cpfclamp(-cp.bias_coef(this.errorBias, dt) * (dist - this.dist) / dt, -maxBias, maxBias);
		}

		public override void ApplyCachedImpulse(double dt_coef)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			cpVect j = cpVect.cpvmult(this.n, this.jnAcc * dt_coef);
			cp.apply_impulses(a, b, this.r1, this.r2, j);
		}

		public override void ApplyImpulse(double dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;
			cpVect n = this.n;

			// compute relative velocity
			double vrn = cp.normal_relative_velocity(a, b, this.r1, this.r2, n);

			double jnMax = this.maxForce * dt;

			// compute normal impulse
			double jn = (this.bias - vrn) * this.nMass;
			double jnOld = this.jnAcc;
			this.jnAcc = cp.cpfclamp(jnOld + jn, -jnMax, jnMax);
			jn = this.jnAcc - jnOld;

			// apply impulse
			cp.apply_impulses(a, b, this.r1, this.r2, cpVect.cpvmult(n, jn));
		}

		public override double GetImpulse()
		{
			return cp.cpfabs(this.jnAcc);
		}

		public override void Draw(cpDebugDraw m_debugDraw)
		{
			var a = this.a.LocalToWorld(this.anchorA);
			var b = this.b.LocalToWorld(this.anchorB);
			m_debugDraw.DrawSegment(a, b, cpColor.Grey);
		}



		public override cpVect GetAnchorA()
		{
			return anchorA;
		}

		public override void SetAnchorA(cpVect anchr)
		{
			ActivateBodies();
			anchorA = anchr;
		}

		public override cpVect GetAnchorB()
		{
			return anchorB;
		}

		public override void SetAnchorB(cpVect anchr)
		{
			ActivateBodies();
			anchorB = anchr;
		}


		public override void SetDist(double distance)
		{
			ActivateBodies();
			dist = distance;
		}
		public override double GetDist()
		{

			return dist;
		}


		public cpPinJoint(cpBody a, cpBody b, cpVect anchorA, cpVect anchorB)
			: base(a, b)
		{

			this.anchorA = anchorA;
			this.anchorB = anchorB;

			// STATIC_BODY_CHECK
			cpVect p1 = (a != null ? cpTransform.cpTransformPoint(a.transform, anchorA) : anchorA);
			cpVect p2 = (b != null ? cpTransform.cpTransformPoint(b.transform, anchorB) : anchorB);
			this.dist = cpVect.cpvlength(cpVect.cpvsub(p2, p1));

			cp.assertSoft(this.dist > 0, "You created a 0 length pin joint. A pivot joint will be much more stable.");
			//if (this.dist <= 0)
			//	throw new NotImplementedException("You created a 0 length pin joint. A pivot joint will be much more stable.");

			this.jnAcc = 0.0f;
		}





	}

}

