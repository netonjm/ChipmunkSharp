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

	public class cpSimpleMotor : cpConstraint
	{

	internal	double rate;
	internal	double iSum;
	internal	double jAcc;

		public override void PreStep(double dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			// calculate moment of inertia coefficient.
			this.iSum = 1.0f / (a.i_inv + b.i_inv);
		}

		public override void ApplyCachedImpulse(double dt_coef)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			double j = this.jAcc * dt_coef;
			a.w -= j * a.i_inv;
			b.w += j * b.i_inv;
		}


		public override void ApplyImpulse(double dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			// compute relative rotational velocity
			double wr = b.w - a.w + this.rate;

			double jMax = this.maxForce * dt;

			// compute normal impulse	
			double j = -wr * this.iSum;
			double jOld = this.jAcc;
			this.jAcc = cp.cpfclamp(jOld + j, -jMax, jMax);
			j = this.jAcc - jOld;

			// apply impulse
			a.w -= j * a.i_inv;
			b.w += j * b.i_inv;

		}

		public override double GetImpulse()
		{
			return cp.cpfabs(jAcc);
		}




		public cpSimpleMotor(cpBody a, cpBody b, double rate)
			: base(a, b)
		{

			this.rate = rate;

			this.jAcc = 0.0f;

		}



		public override void SetRate(double rate)
		{
			ActivateBodies();
			this.rate = rate;
		}


		public override double GetRate()
		{
			return rate;
		}



	}


}