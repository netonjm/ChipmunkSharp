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

    public class cpDampedSpring : cpConstraint
    {

        #region PUBLIC PROPS
        public cpVect r1 { get; set; }
        public cpVect r2 { get; set; }
        public cpVect n { get; set; }
        public float v_coef { get; set; }
        public float target_vrn { get; set; }
        public float nMass { get; set; }
        #endregion

        #region PRIVATE PROPS
        float dist { get; set; }
        cpVect anchr1 { get; set; }
        cpVect anchr2 { get; set; }
        float restLength { get; set; }
        float damping { get; set; }
        float stiffness { get; set; }

        #endregion

        #region PROPS OVERIDE

        public override float getDist()
        {
            return dist;
        }
        public override void setDist(float distance)
        {
            dist = distance;
        }
        public override void setStiffness(float stiffness)
        {
            this.stiffness = stiffness;
        }

        public override float getStiffness()
        {
            return base.getStiffness();
        }

        public override void setAnchr1(cpVect anchr1)
        {
            this.anchr1 = anchr1;
        }

        public override cpVect getAnchr1()
        {
            return this.anchr1;
        }

        public override void setAnchr2(cpVect anchr2)
        {
            this.anchr2 = anchr2;
        }

        public override cpVect getAnchr2()
        {
            return anchr2;
        }

        public override void setRestLength(float restLength)
        {
            this.restLength = restLength;
        }

        public override float getRestLength()
        {
            return restLength;
        }

        public override float getDamping()
        {
            return damping;
        }

        public override void setDamping(float damping)
        {
            this.damping = damping;
        }

        #endregion

        public cpDampedSpring(cpBody a, cpBody b, cpVect anchr1, cpVect anchr2, float restLength, float stiffness, float damping)
            : base(a, b)
        {

            this.anchr1 = anchr1;
            this.anchr2 = anchr2;

            this.restLength = restLength;

            this.stiffness = stiffness;
            this.damping = damping;

            this.target_vrn = this.v_coef = 0;

            this.r1 = this.r2 = null;
            this.nMass = 0;
            this.n = null;

        }

        public float springForceFunc(cpDampedSpring spring, float relativeAngle)
        {
            return (spring.restLength - dist) * spring.stiffness;
        }

        public override void PreStep(float dt)
        {

            this.r1 = anchr1.Rotate(a.Rotation); // vrotate(this.anchr1, a.rot);
            this.r2 = anchr2.Rotate(b.Rotation); // vrotate(this.anchr2, b.rot);

            var delta = b.Position.Add(r2).Sub(a.Position.Add(r1)); //  vsub(vadd(b.p, this.r2), vadd(a.p, this.r1));
            var dist = delta.Length; // vlength(delta);
            this.n = delta.Multiply(1 / (dist > 0 ? dist : cpEnvironment.Infinity));

            var k = cpEnvironment.k_scalar(a, b, this.r1, this.r2, this.n);

            //assertSoft(k !== 0, "Unsolvable this.");
            if (k == 0)
                throw new NotImplementedException("Unsolvable this.");

            this.nMass = 1 / k;

            this.target_vrn = 0;
            this.v_coef = 1 - (float)Math.Exp(-this.damping * dt * k);

            // apply this force
            var f_spring = this.springForceFunc(this, dist);

            cpEnvironment.apply_impulses(a, b, this.r1, this.r2, this.n.x * f_spring * dt, this.n.y * f_spring * dt);
        }

        public override void ApplyCachedImpulse(float coef)
        {
            //base.ApplyCachedImpulse(coef);
        }

        public override void ApplyImpulse(float dt)
        {
            // compute relative velocity
            var vrn = cpEnvironment.normal_relative_velocity(a, b, r1, r2, n);

            // compute velocity loss from drag
            var v_damp = (this.target_vrn - vrn) * this.v_coef;
            this.target_vrn = vrn + v_damp;

            v_damp *= this.nMass;
            cpEnvironment.apply_impulses(a, b, this.r1, this.r2, this.n.x * v_damp, this.n.y * v_damp);
        }

        public override float GetImpulse()
        {
            return 0;
        }

    }

}

