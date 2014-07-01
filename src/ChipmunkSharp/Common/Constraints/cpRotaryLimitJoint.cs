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

    public class cpRotaryLimitJoint : cpConstraint
    {

        #region PUBLIC PROPS

        public float jMax { get; set; }

        public float bias { get; set; }

        public float iSum { get; set; }

        public float jAcc { get; set; }

        public float min { get; set; }

        public float max { get; set; }

        #endregion

        public cpRotaryLimitJoint(cpBody a, cpBody b, float min, float max)
            : base(a, b)
        {

            this.min = min;
            this.max = max;

            this.jAcc = 0.0f;

            this.iSum = this.bias = this.jMax = 0.0f;
        }

        public override void PreStep(float dt)
        {

            float dist = b.Angle - a.Angle;
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
            this.iSum = 1 / (1 / a.Moment + 1 / b.Moment);

            // calculate bias velocity
            var maxBias = this.maxBias;
            this.bias = cpEnvironment.cpclamp(-cpEnvironment.bias_coef(this.errorBias, dt) * pdist / dt, -maxBias, maxBias);

            // compute max impulse
            this.jMax = this.maxForce * dt;

            // If the bias is 0, the joint is not at a limit. Reset the impulse.
            if (this.bias == 0)
                this.jAcc = 0;
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
            if (this.bias < 0)
            {
                this.jAcc = cpEnvironment.cpclamp(jOld + j, 0, this.jMax);
            }
            else
            {
                this.jAcc = cpEnvironment.cpclamp(jOld + j, -this.jMax, 0);
            }

            j = this.jAcc - jOld;

            // apply impulse
            a.w -= j * a.i_inv;
            b.w += j * b.i_inv;
        }

        public override float GetImpulse()
        {
            return Math.Abs(jAcc);
        }





    }

}