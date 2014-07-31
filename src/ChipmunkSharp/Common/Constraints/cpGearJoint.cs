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

	public class cpGearJoint : cpConstraint
	{

		internal double phase, ratio;
		internal double ratio_inv;

		internal double iSum;

		internal double bias;
		internal double jAcc;


		public override void PreStep(double dt)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			// calculate moment of inertia coefficient.
			this.iSum = 1.0f / (a.i_inv * this.ratio_inv + this.ratio * b.i_inv);

			// calculate bias velocity
			double maxBias = this.maxBias;
			this.bias = cp.cpfclamp(-cp.bias_coef(this.errorBias, dt) * (b.a * this.ratio - a.a - this.phase) / dt, -maxBias, maxBias);
		}

		public override void ApplyCachedImpulse(double dt_coef)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			double j = this.jAcc * dt_coef;
			a.w -= j * a.i_inv * this.ratio_inv;
			b.w += j * b.i_inv;
		}

		public override void ApplyImpulse(double dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			// compute relative rotational velocity
			double wr = b.w * this.ratio - a.w;

			double jMax = this.maxForce * dt;

			// compute normal impulse	
			double j = (this.bias - wr) * this.iSum;
			double jOld = this.jAcc;
			this.jAcc = cp.cpfclamp(jOld + j, -jMax, jMax);
			j = this.jAcc - jOld;

			// apply impulse
			a.w -= j * a.i_inv * this.ratio_inv;
			b.w += j * b.i_inv;
		}

		public override double GetImpulse()
		{
			return cp.cpfabs(this.jAcc);
		}


		public cpGearJoint(cpBody a, cpBody b, double phase, double ratio)
			: base(a, b)
		{

			this.phase = phase;
			this.ratio = ratio;
			this.ratio_inv = 1 / ratio;

			this.jAcc = 0.0f;


		}



		public double GetPhase()
		{
			return this.phase;
		}

		public void SetPhase(double phase)
		{
			ActivateBodies();
			this.phase = phase;
		}

		public double GetRatio()
		{
			return this.ratio;
		}

		public override void SetRatio(double value)
		{
			this.ActivateBodies();
			this.ratio = value;
			this.ratio_inv = 1 / value;

		}

	}

}

