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
	public class cpSlideJoint : cpConstraint
	{

		#region PUBLIC PROPS

		public cpVect r1 { get; set; }

		public cpVect r2 { get; set; }
		public cpVect n { get; set; }

		public float min { get; set; }

		public float max { get; set; }

		public float nMass { get; set; }

		public float jnMax { get; set; }

		public float bias { get; set; }

		public float jnAcc { get; set; }

		public float iSum { get; set; }

		public float jMax { get; set; }

		public float jAcc { get; set; }

		public float rate { get; set; }

		#endregion

		#region PRIVATE PROPS


		public cpVect anchr1 { get; set; }

		public cpVect anchr2 { get; set; }

		#endregion


		public cpSlideJoint(cpBody a, cpBody b, cpVect anchr1, cpVect anchr2, float min, float max)
			: base(a, b)
		{

			this.anchr1 = anchr1;
			this.anchr2 = anchr2;
			this.min = min;
			this.max = max;

			this.r1 = null;
			this.r2 = null;
			this.n = null;
			this.nMass = 0.0f;

			this.jnAcc = this.jnMax = 0;
			this.bias = 0.0f;
		}

		public override void PreStep(float dt)
		{


			this.r1 = anchr1.Rotate(a.Rotation); // cpvrotate(this.anchr1, a.rot);
			this.r2 = anchr2.Rotate(b.Rotation); // cpvrotate(this.anchr2, b.rot);

			cpVect delta = b.Position.Add(r2).Sub(a.Position.Add(r1)); // cpvsub(cpvadd(b.p, this.r2), cpvadd(a.p, this.r1));

			float dist = delta.Length; // cpvlength(delta);
			float pdist = 0.0f;
			if (dist > this.max)
			{
				pdist = dist - this.max;
				this.n = cpVect.vnormalize_safe(delta); // cpEnvironment.vnormalize_safe(delta);
			}
			else if (dist < this.min)
			{
				pdist = this.min - dist;
				this.n = cpVect.cpvneg(cpVect.vnormalize_safe(delta)); // vnormalize_safe(delta).Neg();
			}
			else
			{
				this.n = cpVect.Zero;
				this.jnAcc = 0;
			}

			// calculate mass normal
			this.nMass = 1 / cp.k_scalar(a, b, this.r1, this.r2, this.n);


			// calculate bias velocity
			this.bias = cp.cpclamp(-cp.bias_coef(this.errorBias, dt) * pdist / dt, -maxBias, maxBias);

			// compute max impulse
			this.jnMax = this.maxForce * dt;
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{
			var jn = this.jnAcc * dt_coef;
			cp.apply_impulses(this.a, this.b, this.r1, this.r2, this.n.x * jn, this.n.y * jn);
		}


		public override void ApplyImpulse(float dt)
		{

			if (this.n.x == 0 && this.n.y == 0) return;  // early exit


			// compute relative velocity
			var vr = cp.relative_velocity(a, b, r1, r2);
			var vrn = vr.Dot(n); // cpvdot(vr, n);

			// compute normal impulse
			var jn = (this.bias - vrn) * this.nMass;
			var jnOld = this.jnAcc;
			this.jnAcc = cp.cpclamp(jnOld + jn, -this.jnMax, 0f);
			jn = this.jnAcc - jnOld;

			// apply impulse
			cp.apply_impulses(a, b, this.r1, this.r2, n.x * jn, n.y * jn);

		}

		public override float GetImpulse()
		{
			return Math.Abs(jAcc);
		}

		public override void Draw(cpDebugDraw m_debugDraw)
		{
			var a = this.a.local2World(this.anchr1);
			var b = this.b.local2World(this.anchr2);
			var midpoint = cpVect.cpvadd(a, cpVect.cpvclamp(cpVect.cpvsub(b, a), this.min));

			m_debugDraw.DrawSegment(a, b, cpColor.Grey);
			m_debugDraw.DrawSegment(a, midpoint, cpColor.Grey);

		}

	}

}

