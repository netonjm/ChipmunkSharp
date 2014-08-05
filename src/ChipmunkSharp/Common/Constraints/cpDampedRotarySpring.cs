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
	public class cpDampedRotarySpring : cpConstraint
	{

		#region PROPS


		internal float restAngle { get; set; }
		internal float stiffness { get; set; }
		internal float damping { get; set; }


		internal float target_wrn { get; set; }
		internal float w_coef { get; set; }


		internal float iSum { get; set; }
		internal float jAcc { get; set; }

		internal Func<cpDampedRotarySpring, float, float> springTorqueFunc { get; set; }

		#endregion

		public float defaultSpringTorque(cpDampedRotarySpring spring, float relativeAngle)
		{
			return (relativeAngle - spring.restAngle) * spring.stiffness;
		}

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


		public Func<cpDampedRotarySpring, float, float> GetSpringTorqueFunc()
		{
			return this.springTorqueFunc;
		}

		public void SetSpringTorqueFunc(Func<cpDampedRotarySpring, float, float> springTorqueFunc)
		{
			this.springTorqueFunc = springTorqueFunc;
		}


		/// ///////////////////////////////////////////////////////////////

		#region PROPS OVERWRTE

		public override float GetRestAngle()
		{
			return this.restAngle;
		}

		public override void SetRestAngle(float restAngle)
		{
			ActivateBodies();
			this.restAngle = restAngle;
		}

		public override void SetStiffness(float stiffness)
		{
			ActivateBodies();
			this.stiffness = stiffness;
		}
		public override float GetStiffness()
		{

			return this.stiffness;
		}

		public override void SetDamping(float damping)
		{
			ActivateBodies();
			base.SetDamping(damping);
		}

		public override float GetDamping()
		{
			return this.damping;
		}


		#endregion

		public override void ApplyCachedImpulse(float dt_coef)
		{

		}

		public override void PreStep(float dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			float moment = a.i_inv + b.i_inv;
			cp.AssertSoft(moment != 0.0f, "Unsolvable spring.");
			this.iSum = 1.0f / moment;

			this.w_coef = 1.0f - cp.cpfexp(-this.damping * dt * moment);
			this.target_wrn = 0.0f;

			// apply spring torque
			float j_spring = this.springTorqueFunc(this, a.a - b.a) * dt;
			this.jAcc = j_spring;

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
			this.jAcc += j_damp;

			a.w += j_damp * a.i_inv;
			b.w -= j_damp * b.i_inv;
		}

		public override float GetImpulse()
		{
			return this.jAcc;
		}

		//public override void Draw(cpDebugDraw m_debugDraw)
		//{
		//	//base.Draw(m_debugDraw);
		//	//var a = this.a.LocalToWorld(this.anchr1);
		//	//var b = this.b.LocalToWorld(this.anchr2);

		//	////ctx.strokeStyle = "grey";
		//	//drawSpring(ctx, scale, point2canvas, a, b);


		//}


		//public static cpConstraint cpDampedRotarySpringNew(cpBody cpBody1, cpBody cpBody2, float p, float stiffness, float damping)
		//{
		//    throw new NotImplementedException();
		//}
	}




}