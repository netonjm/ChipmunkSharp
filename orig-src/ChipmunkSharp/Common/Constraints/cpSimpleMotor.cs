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

    public class cpSimpleMotor : cpConstraint
    {

        public cpSimpleMotor(cpBody a, cpBody b, float rate)
            : base(a, b)
        {

            this.rate = rate;

            this.jAcc = 0.0f;

            this.iSum = this.jMax = 0.0f;
        }

        public override void PreStep(float dt)
        {

            // calculate moment of inertia coefficient.
            this.iSum = 1 / (this.a.i_inv + this.b.i_inv);

            // compute max impulse
            this.jMax = this.maxForce * dt;
        }

        public override void ApplyCachedImpulse(float dt_coef)
        {

            var j = this.jAcc * dt_coef;
            a.w -= j * a.i_inv;
            b.w += j * b.i_inv;
        }


        public override void ApplyImpulse(float dt)
        {


            // compute relative rotational velocity
            var wr = b.w - a.w + this.rate;

            // compute normal impulse	
            var j = -wr * this.iSum;
            var jOld = this.jAcc;
            this.jAcc = cpEnvironment.cpclamp(jOld + j, -this.jMax, this.jMax);
            j = this.jAcc - jOld;

            // apply impulse
            a.w -= j * a.i_inv;
            b.w += j * b.i_inv;

        }

        public override float GetImpulse()
        {
            return Math.Abs(jAcc);
        }


        public float jMax { get; set; }

        public float iSum { get; set; }

        public float jAcc { get; set; }

        public float rate { get; set; }

        public static cpConstraint cpSimpleMotorNew(cpBody cpBody1, cpBody cpBody2, float rate)
        {
            throw new NotImplementedException();
        }
    }


}