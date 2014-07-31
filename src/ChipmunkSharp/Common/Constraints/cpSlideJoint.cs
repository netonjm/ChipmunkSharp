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
	public class cpSlideJoint : cpConstraint
	{


		internal cpVect anchorA, anchorB;
		internal double min, max;

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
			double pdist = 0.0f;
			if (dist > this.max)
			{
				pdist = dist - this.max;
				this.n = cpVect.cpvnormalize(delta);
			}
			else if (dist < this.min)
			{
				pdist = this.min - dist;
				this.n = cpVect.cpvneg(cpVect.cpvnormalize(delta));
			}
			else
			{
				this.n = cpVect.Zero;
				this.jnAcc = 0.0f;
			}

			// calculate mass normal
			this.nMass = 1.0f / cp.k_scalar(a, b, this.r1, this.r2, this.n);

			// calculate bias velocity
			double maxBias = this.maxBias;
			this.bias = cp.cpfclamp(-cp.bias_coef(this.errorBias, dt) * pdist / dt, -maxBias, maxBias);
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
			if (cpVect.cpveql(this.n, cpVect.Zero)) return;  // early exit

			cpBody a = this.a;
			cpBody b = this.b;

			cpVect n = this.n;
			cpVect r1 = this.r1;
			cpVect r2 = this.r2;

			// compute relative velocity
			cpVect vr = cp.relative_velocity(a, b, r1, r2);
			double vrn = cpVect.cpvdot(vr, n);

			// compute normal impulse
			double jn = (this.bias - vrn) * this.nMass;
			double jnOld = this.jnAcc;
			this.jnAcc = cp.cpfclamp(jnOld + jn, -this.maxForce * dt, 0.0f);
			jn = this.jnAcc - jnOld;

			// apply impulse
			cp.apply_impulses(a, b, this.r1, this.r2, cpVect.cpvmult(n, jn));

		}

		public override double GetImpulse()
		{
			return cp.cpfabs(jnAcc);
		}

		public cpSlideJoint(cpBody a, cpBody b, cpVect anchorA, cpVect anchorB, double min, double max)
			: base(a, b)
		{

			this.anchorA = anchorA;
			this.anchorB = anchorB;
			this.min = min;
			this.max = max;

			this.jnAcc = 0.0f;
		}


		public override cpVect GetAnchorA()
		{
			return this.anchorA;
		}

		public override void SetAnchorA(cpVect anchorA)
		{
			ActivateBodies();
			this.anchorA = anchorA;
		}

		public override cpVect GetAnchorB()
		{

			return this.anchorB;
		}

		public override void SetAnchorB(cpVect anchorB)
		{
			ActivateBodies();
			this.anchorB = anchorB;
		}


		public override double GetMin()
		{
			return this.min;
		}

		public override void SetMin(double min)
		{
			ActivateBodies();
			this.min = min;
		}

		public override void SetMax(double max)
		{
			ActivateBodies();
			this.max = max;
		}


		public override double GetMax()
		{
			return this.max;
		}



		public override void Draw(cpDebugDraw m_debugDraw)
		{
			var a = this.a.LocalToWorld(this.anchorA);
			var b = this.b.LocalToWorld(this.anchorB);
			var midpoint = cpVect.cpvadd(a, cpVect.cpvclamp(cpVect.cpvsub(b, a), this.min));

			m_debugDraw.DrawSegment(a, b, cpColor.Grey);
			m_debugDraw.DrawSegment(a, midpoint, cpColor.Grey);

		}

	}

}

