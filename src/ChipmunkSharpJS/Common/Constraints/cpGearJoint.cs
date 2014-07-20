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

	public class cpGearJoint : cpConstraint
	{

		#region PUBLIC PROPS

		public float phase { get; set; }
		public float ratio { get; set; }
		public float ratio_inv { get; set; }
		public float jAcc { get; set; }
		public float jMax { get; set; }
		public float bias { get; set; }
		public float iSum { get; set; }

		#endregion

		public cpGearJoint(cpBody a, cpBody b, float phase, float ratio)
			: base(a, b)
		{

			this.phase = phase;
			this.ratio = ratio;
			this.ratio_inv = 1 / ratio;

			this.jAcc = 0.0f;

			this.iSum = this.bias = this.jMax = 0.0f;
		}

		public override void PreStep(float dt)
		{
			// calculate moment of inertia coefficient.
			this.iSum = 1 / (a.i_inv * this.ratio_inv + this.ratio * b.i_inv);

			// calculate bias velocity
			this.bias = cp.cpfclamp(-cp.bias_coef(this.errorBias, dt) * (b.a * this.ratio - a.a - this.phase) / dt, -maxBias, maxBias);

			// compute max impulse
			this.jMax = this.maxForce * dt;
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{

			var j = this.jAcc * dt_coef;
			a.w -= j * a.i_inv * this.ratio_inv;
			b.w += j * b.i_inv;
		}

		public override void ApplyImpulse(float dt)
		{

			// compute relative rotational velocity
			var wr = b.w * this.ratio - a.w;

			// compute normal impulse	
			var j = (this.bias - wr) * this.iSum;
			var jOld = this.jAcc;
			this.jAcc = cp.cpfclamp(jOld + j, -this.jMax, this.jMax);

			j = this.jAcc - jOld;

			// apply impulse
			a.w -= j * a.i_inv * this.ratio_inv;
			b.w += j * b.i_inv;
		}


		public override float GetImpulse()
		{
			return Math.Abs(this.jAcc);
		}


		public override void SetRatio(float value)
		{
			this.ratio = value;
			this.ratio_inv = 1 / value;
			this.activateBodies();
		}


	}

}

