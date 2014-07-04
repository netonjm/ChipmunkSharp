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
        public cpVect anchr1 { get; set; }
        public cpVect anchr2 { get; set; }
        public float dist { get; set; }
        public float nMass { get; set; }
        public float jnMax { get; set; }
        public float bias { get; set; }
        public float jnAcc { get; set; }
        public cpVect r2 { get; set; }
        public cpVect r1 { get; set; }
        public cpVect n { get; set; }

        #endregion


        public cpPinJoint(cpBody a, cpBody b, cpVect anchr1, cpVect anchr2)
            : base(a, b)
        {


            this.anchr1 = anchr1;
            this.anchr2 = anchr2;

            // STATIC_BODY_CHECK
            var p1 = (a != null ? a.Position.Add(anchr1.Rotate(a.Rotation)) : anchr1);
            var p2 = (b != null ? b.Position.Add(anchr2.Rotate(b.Rotation)) : anchr2);
            this.dist = p2.Sub(p1).Length; // vlength(vsub(p2, p1));

            //assertSoft(this.dist > 0, "You created a 0 length pin joint. A pivot joint will be much more stable.");
            if (this.dist <= 0)
                throw new NotImplementedException("You created a 0 length pin joint. A pivot joint will be much more stable.");

            this.r1 = this.r2 = null;
            this.n = null;
            this.nMass = 0.0f;

            this.jnAcc = this.jnMax = 0.0f;
            this.bias = 0.0f;
        }


        public override void PreStep(float dt)
        {
            //var a = this.a;
            //var b = this.b;

            this.r1 = this.anchr1.Rotate(a.Rotation); // vrotate(this.anchr1, a.rot);
            this.r2 = this.anchr2.Rotate(b.Rotation); // vrotate(this.anchr2, b.rot);

            var delta = b.Position.Add(this.r2).Sub(a.Position.Add(this.r1));// vsub(vadd(b.p, this.r2), vadd(a.p, this.r1));
            var dist = delta.Length; // vlength(delta);

            this.n = delta.Multiply(1 / (dist > 0 ? dist : cpEnvironment.Infinity));  //vmult(delta, 1 / (dist ? dist : Infinity));

            // calculate mass normal
            this.nMass = 1 / cpEnvironment.k_scalar(a, b, this.r1, this.r2, this.n);

            // calculate bias velocity
            var maxBias = this.maxBias;
            this.bias = cpEnvironment.cpfclamp(-cpEnvironment.bias_coef(this.errorBias, dt) * (dist - this.dist) / dt, -maxBias, maxBias);

            // compute max impulse
            this.jnMax = this.maxForce * dt;
        }

        public override void ApplyCachedImpulse(float dt_coef)
        {
            var j = cpVect.cpvmult(this.n, this.jnAcc * dt_coef);
            cpEnvironment.apply_impulses(this.a, this.b, this.r1, this.r2, j.x, j.y);
        }

        public override void ApplyImpulse(float dt)
        {


            // compute relative velocity
            var vrn = cpEnvironment.normal_relative_velocity(a, b, this.r1, this.r2, n);

            // compute normal impulse
            var jn = (this.bias - vrn) * this.nMass;
            var jnOld = this.jnAcc;
            this.jnAcc = cpEnvironment.cpclamp(jnOld + jn, -this.jnMax, this.jnMax);
            jn = this.jnAcc - jnOld;

            // apply impulse
            cpEnvironment.apply_impulses(a, b, this.r1, this.r2, n.x * jn, n.y * jn);
        }

        public override float GetImpulse()
        {
            return Math.Abs(this.jnAcc);
        }


        public static cpConstraint cpPinJointNew(cpBody cpBody1, cpBody cpBody2, cpVect anchr1, cpVect anchr2)
        {
            throw new NotImplementedException();
        }
    }

}

