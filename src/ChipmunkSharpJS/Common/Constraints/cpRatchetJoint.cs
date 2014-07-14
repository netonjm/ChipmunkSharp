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
	public class cpRatchetJoint : cpConstraint
	{

		#region PUBLIC PROPS

		public float angle { get; set; }

		public float phase { get; set; }

		public float ratchet { get; set; }

		public float jMax { get; set; }

		public float jAcc { get; set; }

		public float bias { get; set; }

		public float iSum { get; set; }


		#endregion


		public cpRatchetJoint(cpBody a, cpBody b, float phase, float ratchet)
			: base(a, b)
		{


			this.angle = 0.0f;
			this.phase = phase;
			this.ratchet = ratchet;

			// STATIC_BODY_CHECK
			this.angle = (b != null ? b.a : 0) - (a != null ? a.a : 0);

			this.iSum = this.bias = this.jAcc = this.jMax = 0.0f;
		}

		public override void PreStep(float dt)
		{

			var delta = b.a - a.a;
			var diff = angle - delta;
			var pdist = 0.0f;

			if (diff * ratchet > 0)
			{
				pdist = diff;
			}
			else
			{
				this.angle = (float)Math.Floor((delta - phase) / ratchet) * ratchet + phase;
			}

			// calculate moment of inertia coefficient.
			this.iSum = 1 / (a.i_inv + b.i_inv);

			// calculate bias velocity
			var maxBias = this.maxBias;
			this.bias = cp.cpclamp(-cp.bias_coef(this.errorBias, dt) * pdist / dt, -maxBias, maxBias);

			// compute max impulse
			this.jMax = this.maxForce * dt;

			// If the bias is 0, the joint is not at a limit. Reset the impulse.
			if (this.bias == 0) this.jAcc = 0;
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{

			var j = this.jAcc * dt_coef;
			a.w -= j * a.i_inv;
			b.w += j * b.i_inv;
		}

		public override void ApplyImpulse(float dt)
		{
			if (this.bias == 0) return; // early exit

			// compute relative rotational velocity
			var wr = b.w - a.w;

			// compute normal impulse	
			var j = -(this.bias + wr) * this.iSum;
			var jOld = this.jAcc;
			this.jAcc = cp.cpclamp((jOld + j) * ratchet, 0, this.jMax * (float)Math.Abs(ratchet)) / ratchet;
			j = this.jAcc - jOld;

			// apply impulse
			a.w -= j * a.i_inv;
			b.w += j * b.i_inv;
		}

		public override float GetImpulse()
		{
			return (float)Math.Abs(jAcc);
		}



	}


}