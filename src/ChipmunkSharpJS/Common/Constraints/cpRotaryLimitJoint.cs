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

	public class cpRotaryLimitJoint : cpConstraint
	{

		internal float min, max;

		internal float iSum;

		internal float bias;
		internal float jAcc;

		public override void PreStep(float dt)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			float dist = b.a - a.a;
			float pdist = 0.0f;
			if (dist > this.max)
			{
				pdist = this.max - dist;
			}
			else if (dist < this.min)
			{
				pdist = this.min - dist;
			}

			// calculate moment of inertia coefficient.
			this.iSum = 1.0f / (a.i_inv + b.i_inv);

			// calculate bias velocity
			float maxBias = this.maxBias;
			this.bias = cp.cpfclamp(-cp.bias_coef(this.errorBias, dt) * pdist / dt, -maxBias, maxBias);

			// If the bias is 0, the joint is not at a limit. Reset the impulse.
			if (this.bias == 0) this.jAcc = 0.0f;
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			float j = this.jAcc * dt_coef;
			a.w -= j * a.i_inv;
			b.w += j * b.i_inv;
		}


		public override void ApplyImpulse(float dt)
		{

			if (this.bias == 0) return; // early exit

			cpBody a = this.a;
			cpBody b = this.b;

			// compute relative rotational velocity
			float wr = b.w - a.w;

			float jMax = this.maxForce * dt;

			// compute normal impulse	
			float j = -(this.bias + wr) * this.iSum;
			float jOld = this.jAcc;
			if (this.bias < 0.0f)
			{
				this.jAcc = cp.cpfclamp(jOld + j, 0.0f, jMax);
			}
			else
			{
				this.jAcc = cp.cpfclamp(jOld + j, -jMax, 0.0f);
			}
			j = this.jAcc - jOld;

			// apply impulse
			a.w -= j * a.i_inv;
			b.w += j * b.i_inv;
		}


		public cpRotaryLimitJoint(cpBody a, cpBody b, float min, float max)
			: base(a, b)
		{

			this.min = min;
			this.max = max;

			this.jAcc = 0.0f;

		}


		public override float GetImpulse()
		{
			return cp.cpfabs(jAcc);
		}


		public override float GetMin()
		{
			return base.GetMin();
		}

		public override void SetMin(float min)
		{
			base.SetMin(min);
		}

		public override float GetMax()
		{
			return base.GetMax();
		}

		public override void SetMax(float max)
		{
			base.SetMax(max);
		}


	}

}