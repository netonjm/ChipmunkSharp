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
	public class cpDampedRotarySpring : cpConstraint
	{

		#region PROPS

		public float target_wrn { get; set; }
		public float w_coef { get; set; }
		public float iSum { get; set; }

		float restAngle { get; set; }
		float stiffness { get; set; }
		float damping { get; set; }

		#endregion

		#region PROPS OVERWRTE

		public override float getRestAngle()
		{
			return this.restAngle;
		}

		public override void setRestAngle(float restAngle)
		{
			this.restAngle = restAngle;
		}

		public override void setStiffness(float stiffness)
		{
			this.stiffness = stiffness;
		}
		public override float getStiffness()
		{
			return this.stiffness;
		}

		public override void setDamping(float damping)
		{
			base.setDamping(damping);
		}

		public override float getDamping()
		{
			return this.damping;
		}

		#endregion

		Func<cpDampedRotarySpring, float, float> springTorqueFunc;

		public cpDampedRotarySpring(cpBody a, cpBody b, float restAngle, float stiffness, float damping)
			: base(a, b)
		{

			this.restAngle = restAngle;
			this.stiffness = stiffness;
			this.damping = damping;

			this.springTorqueFunc = defaultSpringTorque;

			this.target_wrn = 0.0f;
			this.w_coef = 0.0f;
			this.iSum = 0.0f;
		}

		public float defaultSpringTorque(cpDampedRotarySpring spring, float relativeAngle)
		{
			return (relativeAngle - spring.restAngle) * spring.stiffness;
		}

		public override void PreStep(float dt)
		{

			var moment = a.i_inv + b.i_inv;

			cp.assertSoft(moment != 0, "Unsolvable spring.");

			this.iSum = 1 / moment;

			this.w_coef = 1 - (float)Math.Exp(-this.damping * dt * moment);
			this.target_wrn = 0;

			// apply this torque
			float j_spring = springTorqueFunc(this, a.Angle - b.Angle) * dt;

			a.w -= j_spring * a.i_inv;
			b.w += j_spring * b.i_inv;
		}

		public override void ApplyImpulse(float dt)
		{
			// compute relative velocity
			var wrn = a.w - b.w;//normal_relative_velocity(a, b, r1, r2, n) - this.target_vrn;

			// compute velocity loss from drag
			// not 100% certain spring is derived correctly, though it makes sense
			var w_damp = (this.target_wrn - wrn) * this.w_coef;
			this.target_wrn = wrn + w_damp;

			//apply_impulses(a, b, this.r1, this.r2, vmult(this.n, v_damp*this.nMass));
			var j_damp = w_damp * this.iSum;
			a.w += j_damp * a.i_inv;
			b.w -= j_damp * b.i_inv;
		}


		public override void Draw(cpDebugDraw m_debugDraw)
		{
			//base.Draw(m_debugDraw);
			//var a = this.a.local2World(this.anchr1);
			//var b = this.b.local2World(this.anchr2);

			//ctx.strokeStyle = "grey";
			//drawSpring(ctx, scale, point2canvas, a, b);


		}


		//public static cpConstraint cpDampedRotarySpringNew(cpBody cpBody1, cpBody cpBody2, float p, float stiffness, float damping)
		//{
		//    throw new NotImplementedException();
		//}
	}




}