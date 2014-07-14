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

			this.k1 = cpVect.Zero;// new cpVect(0, 0);
			this.k2 = cpVect.Zero;// new Vect(0, 0);

			this.jAcc = cpVect.Zero;
			this.jMaxLen = 0.0f;
			this.bias = null;

		}

		public override void PreStep(float dt)
		{

			// calculate endpoints in worldspace
			var ta = a.local2World(this.grv_a);
			var tb = a.local2World(this.grv_b);

			// calculate axis
			var n = cpVect.cpvrotate(this.grv_n, a.Rotation);
			var d = cpVect.cpvdot(ta, n);

			this.grv_tn = n;
			this.r2 = cpVect.cpvrotate(this.anchr2, b.Rotation);// vrotate(this.anchr2, b.rot);

			// calculate tangential distance along the axis of r2
			var td = cpVect.cpvcross(cpVect.cpvadd(b.Position, this.r2), n);
			// calculate clamping factor and r2
			if (td <= cpVect.cpvcross(ta, n))// vcross(ta, n))
			{
				this.clamp = 1;
				this.r1 = cpVect.cpvsub(ta, a.Position);
			}
			else if (td >= cpVect.cpvcross(tb, n))
			{
				this.clamp = -1;
				this.r1 = cpVect.cpvsub(tb, a.Position);
			}
			else
			{
				this.clamp = 0;
				this.r1 = cpVect.cpvsub(cpVect.cpvadd(cpVect.cpvmult(cpVect.cpvperp(n), -td), cpVect.cpvmult(n, d)), a.Position);  //n.Perp().Multiply(-td).Add(n.Multiply(d)).Sub(a.Position);

			}

			// Calculate mass tensor
			cp.k_tensor(a, b, this.r1, this.r2, this.k1, this.k2);

			// compute max impulse
			this.jMaxLen = this.maxForce * dt;

			// calculate bias velocity
			var delta = cpVect.cpvsub(cpVect.cpvadd(b.Position, this.r2), cpVect.cpvadd(a.Position, this.r1));

			//this.bias = delta.Multiply(-cp.bias_coef(this.errorBias, dt) / dt).Clamp(this.maxBias);
			this.bias = cpVect.cpvclamp(cpVect.cpvmult(delta, -cp.bias_coef(this.errorBias, dt) / dt), this.maxBias);
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{
			cp.apply_impulses(this.a, this.b, this.r1, this.r2, this.jAcc.x * dt_coef, this.jAcc.y * dt_coef);
		}

		public cpVect grooveConstrain(cpVect j)
		{
			var n = this.grv_tn;
			var jClamp = (this.clamp * cpVect.cpvcross(j, n) > 0) ? j : cpVect.cpvproject(j, n);
			return cpVect.cpvclamp(jClamp, this.jMaxLen);
		}

		public override void ApplyImpulse(float dt)
		{

			// compute impulse
			var vr = cp.relative_velocity(a, b, r1, r2);

			var j = cpVect.mult_k(cpVect.cpvsub(this.bias, vr), this.k1, this.k2);
			var jOld = this.jAcc;
			this.jAcc = this.grooveConstrain(cpVect.cpvadd(jOld, j));

			// apply impulse
			cp.apply_impulses(a, b, this.r1, this.r2, this.jAcc.x - jOld.x, this.jAcc.y - jOld.y);
		}

		public override float GetImpulse()
		{
			//return this.jAcc.Length;
			return cpVect.cpvlength(this.jAcc);
		}

		public void SetGrooveA(cpVect value)
		{
			this.grv_a = value;
			this.grv_n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(this.grv_b, value)));

			this.activateBodies();
		}
		public void SetGrooveB(cpVect value)
		{
			this.grv_b = value;
			this.grv_n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(value, this.grv_a)));

			this.activateBodies();
		}

	}



}

