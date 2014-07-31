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

	public class cpDampedSpring : cpConstraint
	{

		internal cpVect anchorA, anchorB;
		internal double restLength;
		internal double stiffness;
		internal double damping;
		internal Func<cpDampedSpring, double, double> springForceFunc;

		internal double v_coef;
		internal double target_vrn;

		internal cpVect n;
		internal cpVect r1, r2;
		internal double nMass;

		internal double jAcc;


		public double defaultSpringForce(cpDampedSpring spring, double dist)
		{
			return (spring.restLength - dist) * spring.stiffness;
		}

		public override void PreStep(double dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			this.r1 = cpTransform.cpTransformVect(a.transform, cpVect.cpvsub(this.anchorA, a.cog));
			this.r2 = cpTransform.cpTransformVect(b.transform, cpVect.cpvsub(this.anchorB, b.cog));

			cpVect delta = cpVect.cpvsub(cpVect.cpvadd(b.p, this.r2), cpVect.cpvadd(a.p, this.r1));
			double dist = cpVect.cpvlength(delta);
			this.n = cpVect.cpvmult(delta, 1.0f / (dist > 0 ? dist : cp.Infinity));

			double k = cp.k_scalar(a, b, this.r1, this.r2, this.n);
			cp.assertSoft(k != 0.0, "Unsolvable this.");
			this.nMass = 1.0f / k;

			this.target_vrn = 0.0f;
			this.v_coef = 1.0f - cp.cpfexp(-this.damping * dt * k);

			// apply spring force
			double f_spring = this.springForceFunc(this, dist);
			double j_spring = this.jAcc = f_spring * dt;
			cp.apply_impulses(a, b, this.r1, this.r2, cpVect.cpvmult(this.n, j_spring));
		}

		public override void ApplyCachedImpulse(double coef)
		{
		}

		public override void ApplyImpulse(double dt)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			cpVect n = this.n;
			cpVect r1 = this.r1;
			cpVect r2 = this.r2;

			// compute relative velocity
			double vrn = cp.normal_relative_velocity(a, b, r1, r2, n);

			// compute velocity loss from drag
			double v_damp = (this.target_vrn - vrn) * this.v_coef;
			this.target_vrn = vrn + v_damp;

			double j_damp = v_damp * this.nMass;
			this.jAcc += j_damp;
			cp.apply_impulses(a, b, this.r1, this.r2, cpVect.cpvmult(this.n, j_damp));
		}

		public cpDampedSpring(cpBody a, cpBody b, cpVect anchr1, cpVect anchr2, double restLength, double stiffness, double damping)
			: base(a, b)
		{

			this.anchorA = anchr1;
			this.anchorB = anchr2;

			this.restLength = restLength;

			this.stiffness = stiffness;
			this.damping = damping;

			this.springForceFunc = defaultSpringForce;

			this.target_vrn = this.v_coef = 0;

			this.r1 = this.r2 = null;
			this.nMass = 0;
			this.n = null;

			this.jAcc = 0f;


		}


		#region PROPS OVERIDE


		public override void SetStiffness(double stiffness)
		{
			this.stiffness = stiffness;
		}

		public override double GetStiffness()
		{
			return base.GetStiffness();
		}

		public override void SetAnchorA(cpVect anchr1)
		{
			this.anchorA = anchr1;
		}

		public override cpVect GetAnchorA()
		{
			return this.anchorA;
		}

		public override void SetAnchorB(cpVect anchr2)
		{
			this.anchorB = anchr2;
		}

		public override cpVect GetAnchorB()
		{
			return anchorB;
		}

		public override void SetRestLength(double restLength)
		{
			this.restLength = restLength;
		}

		public override double GetRestLength()
		{
			return restLength;
		}

		public override double GetDamping()
		{
			return damping;
		}

		public override void SetDamping(double damping)
		{
			this.damping = damping;
		}

		#endregion


		public override double GetImpulse()
		{
			return this.jAcc;
		}

		public override void Draw(cpDebugDraw m_debugDraw)
		{
			var a = this.a.LocalToWorld(this.anchorA);
			var b = this.b.LocalToWorld(this.anchorB);

			m_debugDraw.DrawSpring(a, b, cpColor.Grey);
		}




	}

}

