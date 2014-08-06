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

	public class cpPivotJoint : cpConstraint
	{


		internal cpVect anchorA, anchorB;

		internal cpVect r1, r2;
		internal cpMat2x2 k;

		internal cpVect jAcc;
		internal cpVect bias;

		public override void PreStep(float dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			this.r1 = cpTransform.Vect(a.transform, cpVect.cpvsub(this.anchorA, a.cog));
			this.r2 = cpTransform.Vect(b.transform, cpVect.cpvsub(this.anchorB, b.cog));

			// Calculate mass tensor
			this.k = cp.k_tensor(a, b, this.r1, this.r2);

			// calculate bias velocity
			cpVect delta = cpVect.cpvsub(cpVect.cpvadd(b.p, this.r2), cpVect.cpvadd(a.p, this.r1));
			this.bias = cpVect.cpvclamp(cpVect.cpvmult(delta, -cp.bias_coef(this.errorBias, dt) / dt), this.maxBias);
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			cp.apply_impulses(a, b, this.r1, this.r2, cpVect.cpvmult(this.jAcc, dt_coef));
		}

		public override void ApplyImpulse(float dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			cpVect r1 = this.r1;
			cpVect r2 = this.r2;

			// compute relative velocity
			cpVect vr = cp.relative_velocity(a, b, r1, r2);

			// compute normal impulse
			cpVect j = cpMat2x2.Transform(this.k, cpVect.cpvsub(this.bias, vr));
			cpVect jOld = this.jAcc;
			this.jAcc = cpVect.cpvclamp(cpVect.cpvadd(this.jAcc, j), this.maxForce * dt);
			j = cpVect.cpvsub(this.jAcc, jOld);

			// apply impulse
			cp.apply_impulses(a, b, this.r1, this.r2, j);

		}


		public cpPivotJoint(cpBody a, cpBody b, cpVect pivot)
			: this(a, b,
			(a != null ? a.WorldToLocal(pivot) : pivot),
			(b != null ? b.WorldToLocal(pivot) : pivot))
		{

		}

		public cpPivotJoint(cpBody a, cpBody b, cpVect anchorA, cpVect anchorB)
			: base(a, b)
		{
			this.anchorA = anchorA;
			this.anchorB = anchorB;
			this.jAcc = cpVect.Zero;
		}


		public override float GetImpulse()
		{
			return cpVect.cpvlength(jAcc);
		}

		public override void SetAnchorA(cpVect anchr1)
		{
			ActivateBodies();
			this.anchorA = anchr1;
		}

		public override cpVect GetAnchorA()
		{
			return anchorA;
		}

		public override void SetAnchorB(cpVect anchr2)
		{
			ActivateBodies();
			this.anchorB = anchr2;
		}

		public override cpVect GetAnchorB()
		{
			return anchorB;
		}




	}


}