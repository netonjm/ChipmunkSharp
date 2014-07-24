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
namespace ChipmunkSharp.Constraints
{

	public class cpPinJoint : cpConstraint
	{

		#region PUBLIC PROPS
		public cpVect anchorA, anchorB;
		public cpVect r1, r2;

		public float dist, nMass, jnMax, bias, jnAcc;

		public cpVect n { get; set; }

		#endregion

		public override cpVect GetAnchorA()
		{
			return anchorA;
		}

		public override void SetAnchorA(cpVect anchr)
		{
			anchorA = anchr;
		}

		public override cpVect GetAnchorB()
		{
			return anchorB;
		}

		public override void SetAnchorB(cpVect anchr)
		{
			anchorB = anchr;
		}


		public override void SetDist(float distance)
		{
			dist = distance;
		}
		public override float GetDist()
		{
			return dist;
		}


		public cpPinJoint(cpBody a, cpBody b, cpVect anchr1, cpVect anchr2)
			: base(a, b)
		{


			this.anchorA = anchr1;
			this.anchorB = anchr2;

			// STATIC_BODY_CHECK
			var p1 = (a != null ? a.Position.Add(anchr1.Rotate(a.Rotation)) : anchr1);
			var p2 = (b != null ? b.Position.Add(anchr2.Rotate(b.Rotation)) : anchr2);
			this.dist = cpVect.cpvlength(cpVect.cpvsub(p2, p1));//  //p2.Sub(p1).Length; // vlength(vsub(p2, p1));

			cp.assertSoft(this.dist > 0, "You created a 0 length pin joint. A pivot joint will be much more stable.");
			//if (this.dist <= 0)
			//	throw new NotImplementedException("You created a 0 length pin joint. A pivot joint will be much more stable.");

			this.r1 = null;
			this.r2 = null;
			this.n = null;
			this.nMass = 0.0f;

			this.jnAcc = this.jnMax = 0.0f;
			this.bias = 0.0f;
		}





		public override void PreStep(float dt)
		{
			//var a = this.a;
			//var b = this.b;


			this.r1 = cpVect.cpvrotate(this.anchorA, a.Rotation);
			this.r2 = cpVect.cpvrotate(this.anchorB, b.Rotation);

			var delta = cpVect.cpvsub(cpVect.cpvadd(b.Position, this.r2), cpVect.cpvadd(a.Position, this.r1));
			var dist = cpVect.cpvlength(delta);
			this.n = cpVect.cpvmult(delta, 1 / (dist > 0 ? dist : cp.Infinity));

			// calculate mass normal
			this.nMass = 1 / cp.k_scalar(a, b, this.r1, this.r2, this.n);

			// calculate bias velocity
			var maxBias = this.maxBias;
			this.bias = cp.cpclamp(-cp.bias_coef(this.errorBias, dt) * (dist - this.dist) / dt, -maxBias, maxBias);

			// compute max impulse
			this.jnMax = this.maxForce * dt;
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{
			var j = cpVect.cpvmult(this.n, this.jnAcc * dt_coef);
			cp.apply_impulses(this.a, this.b, this.r1, this.r2, j.x, j.y);
		}

		public override void ApplyImpulse(float dt)
		{


			// compute relative velocity
			var vrn = cp.normal_relative_velocity(a, b, this.r1, this.r2, n);

			// compute normal impulse
			var jn = (this.bias - vrn) * this.nMass;
			var jnOld = this.jnAcc;
			this.jnAcc = cp.cpclamp(jnOld + jn, -this.jnMax, this.jnMax);
			jn = this.jnAcc - jnOld;

			// apply impulse
			cp.apply_impulses(a, b, this.r1, this.r2, n.x * jn, n.y * jn);
		}

		public override float GetImpulse()
		{
			return Math.Abs(this.jnAcc);
		}

		public override void Draw(cpDebugDraw m_debugDraw)
		{
			var a = this.a.Local2World(this.anchorA);
			var b = this.b.Local2World(this.anchorB);
			m_debugDraw.DrawSegment(a, b, cpColor.Grey);

		}


	}

}

