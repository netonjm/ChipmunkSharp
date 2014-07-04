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

    public class cpGrooveJoint : cpConstraint
    {

        #region PUBLIC PROPS

        public cpVect grv_a { get; set; }

        public cpVect grv_b { get; set; }

        public cpVect grv_n { get; set; }

        public cpVect anchr2 { get; set; }

        public float clamp { get; set; }

        public cpVect grv_tn { get; set; }

        public float jMaxLen { get; set; }

        public cpVect k1 { get; set; }

        public cpVect k2 { get; set; }

        public cpVect jAcc { get; set; }

        public cpVect r2 { get; set; }

        public cpVect r1 { get; set; }

        public cpVect bias { get; set; }


        #endregion

        public cpGrooveJoint(cpBody a, cpBody b, cpVect groove_a, cpVect groove_b, cpVect anchr2)
            : base(a, b)
        {


            this.grv_a = groove_a;
            this.grv_b = groove_b;
            this.grv_n = groove_b.Sub(groove_a).Normalize().Perp();// vperp(vnormalize(vsub(groove_b, groove_a)));
            this.anchr2 = anchr2;

            this.grv_tn = null;
            this.clamp = 0.0f;
            this.r1 = this.r2 = null;

            this.k1 = cpVect.ZERO;// new cpVect(0, 0);
            this.k2 = cpVect.ZERO;// new Vect(0, 0);

            this.jAcc = cpVect.ZERO;
            this.jMaxLen = 0.0f;
            this.bias = null;

        }

        public override void PreStep(float dt)
        {

            // calculate endpoints in worldspace
            var ta = a.Local2World(this.grv_a);
            var tb = a.Local2World(this.grv_b);

            // calculate axis
            var n = this.grv_n.Rotate(a.Rotation); // vrotate(, a.rot);
            var d = ta.Dot(n); // vdot(ta, n);

            this.grv_tn = n;
            this.r2 = this.anchr2.Rotate(b.Rotation);// vrotate(this.anchr2, b.rot);

            // calculate tangential distance along the axis of r2
            var td = b.Position.Add(this.r2).CrossProduct(n);// vcross(vadd(b.p, this.r2), n);
            // calculate clamping factor and r2
            if (td <= ta.CrossProduct(n))// vcross(ta, n))
            {
                this.clamp = 1;
                this.r1 = ta.Sub(a.Position);// vsub(ta, a.p);
            }
            else if (td >= tb.CrossProduct(n)) // vcross(tb, n))
            {
                this.clamp = -1;
                this.r1 = tb.Sub(a.Position); // vsub(tb, a.p);
            }
            else
            {
                this.clamp = 0;
                this.r1 = n.Perp().Multiply(-td).Add(n.Multiply(d)).Sub(a.Position);

            }

            // Calculate mass tensor
            cpEnvironment.k_tensor(a, b, this.r1, this.r2, this.k1, this.k2);

            // compute max impulse
            this.jMaxLen = this.maxForce * dt;

            // calculate bias velocity
            var delta = b.Position.Add(this.r2).Sub(a.Position.Add(this.r1)); //  vsub(vadd(b.p, this.r2), vadd(a.p, this.r1));

            this.bias = delta.Multiply(-cpEnvironment.bias_coef(this.errorBias, dt) / dt).Clamp(this.maxBias);
            //this.bias = vclamp(vmult(delta, -bias_coef(this.errorBias, dt) / dt), this.maxBias);
        }

        public override void ApplyCachedImpulse(float dt_coef)
        {
            cpEnvironment.apply_impulses(this.a, this.b, this.r1, this.r2, this.jAcc.x * dt_coef, this.jAcc.y * dt_coef);
        }

        public cpVect grooveConstrain(cpVect j)
        {
            var n = this.grv_tn;
            var jClamp = (this.clamp * j.CrossProduct(n) > 0) ? j : j.Project(n);
            return jClamp.Clamp(this.jMaxLen); // vclamp(jClamp, this.jMaxLen);
        }

        public override void ApplyImpulse(float dt)
        {

            // compute impulse
            var vr = cpEnvironment.relative_velocity(a, b, r1, r2);

            var j = cpVect.mult_k(this.bias.Sub(vr), this.k1, this.k2);
            var jOld = this.jAcc;
            this.jAcc = this.grooveConstrain(jOld.Add(j));

            // apply impulse
            cpEnvironment.apply_impulses(a, b, this.r1, this.r2, this.jAcc.x - jOld.x, this.jAcc.y - jOld.y);
        }

        public override float GetImpulse()
        {
            return this.jAcc.Length;
            //return vlength(this.jAcc);
        }

        public void SetGrooveA(cpVect value)
        {
            this.grv_a = value;
            this.grv_n = this.grv_b.Sub(value).Normalize().Perp(); //vperp(vnormalize(vsub( , value)));

            this.ActivateBodies();
        }
        public void SetGrooveB(cpVect value)
        {
            this.grv_b = value;
            this.grv_n = this.grv_a.Sub(value).Normalize().Perp(); //vperp(vnormalize(vsub( , value)));

            this.ActivateBodies();
        }



        public static cpConstraint cpGrooveJointNew(cpBody cpBody1, cpBody cpBody2, cpVect grooveA, cpVect grooveB, cpVect anchr)
        {
            throw new NotImplementedException();
        }
    }



}

