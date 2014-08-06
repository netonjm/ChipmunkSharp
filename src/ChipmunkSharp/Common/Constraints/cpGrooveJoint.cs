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

	public class cpGrooveJoint : cpConstraint
	{

		#region PUBLIC PROPS


		public cpVect grv_n, grv_a, grv_b;

		public cpVect anchorB;

		cpVect grv_tn;
		float clamp;
		cpVect r1, r2;
		cpMat2x2 k;

		cpVect jAcc;
		cpVect bias;


		#endregion


		public override void PreStep(float dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			// calculate endpoints in worldspace
			cpVect ta = cpTransform.Point(a.transform, this.grv_a);
			cpVect tb = cpTransform.Point(a.transform, this.grv_b);

			// calculate axis
			cpVect n = cpTransform.Vect(a.transform, this.grv_n);
			float d = cpVect.cpvdot(ta, n);

			this.grv_tn = n;
			this.r2 = cpTransform.Vect(b.transform, cpVect.cpvsub(this.anchorB, b.cog));

			// calculate tangential distance along the axis of r2
			float td = cpVect.cpvcross(cpVect.cpvadd(b.p, this.r2), n);
			// calculate clamping factor and r2
			if (td <= cpVect.cpvcross(ta, n))
			{
				this.clamp = 1.0f;
				this.r1 = cpVect.cpvsub(ta, a.p);
			}
			else if (td >= cpVect.cpvcross(tb, n))
			{
				this.clamp = -1.0f;
				this.r1 = cpVect.cpvsub(tb, a.p);
			}
			else
			{
				this.clamp = 0.0f;
				this.r1 = cpVect.cpvsub(cpVect.cpvadd(cpVect.cpvmult(cpVect.cpvperp(n), -td), cpVect.cpvmult(n, d)), a.p);
			}

			// Calculate mass tensor
			this.k = cp.k_tensor(a, b, this.r1, this.r2);

			// calculate bias velocity
			cpVect delta = cpVect.cpvsub(cpVect.cpvadd(b.p, this.r2), cpVect.cpvadd(a.p, this.r1));
			this.bias = cpVect.cpvclamp(cpVect.cpvmult(delta, -cp.bias_coef(this.errorBias, dt) / dt), this.maxBias);
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{
			cpBody a = this.a;
			cpBody b = this.b;

			cp.apply_impulses(a, b, this.r1, this.r2, cpVect.cpvmult(this.jAcc, dt_coef));
		}

		public cpVect grooveConstrain(cpVect j, float dt)
		{
			cpVect n = this.grv_tn;
			cpVect jClamp = (this.clamp * cpVect.cpvcross(j, n) > 0.0f) ? j : cpVect.cpvproject(j, n);
			return cpVect.cpvclamp(jClamp, this.maxForce * dt);
		}

		public override void ApplyImpulse(float dt)
		{

			cpBody a = this.a;
			cpBody b = this.b;

			cpVect r1 = this.r1;
			cpVect r2 = this.r2;

			// compute impulse
			cpVect vr = cp.relative_velocity(a, b, r1, r2);

			cpVect j = cpMat2x2.Transform(this.k, cpVect.cpvsub(this.bias, vr));
			cpVect jOld = this.jAcc;
			this.jAcc = grooveConstrain(cpVect.cpvadd(jOld, j), dt);
			j = cpVect.cpvsub(this.jAcc, jOld);

			// apply impulse
			cp.apply_impulses(a, b, this.r1, this.r2, j);
		}

		public override float GetImpulse()
		{
			//return this.jAcc.Length;
			return cpVect.cpvlength(this.jAcc);
		}


		public cpGrooveJoint(cpBody a, cpBody b, cpVect groove_a, cpVect groove_b, cpVect anchorB)
			: base(a, b)
		{

			this.grv_a = groove_a;
			this.grv_b = groove_b;
			this.grv_n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(groove_b, groove_a)));
			this.anchorB = anchorB;

			this.grv_tn = null;
			this.clamp = 0.0f;
			this.r1 = this.r2 = null;

			this.jAcc = cpVect.Zero;
			this.bias = null;

		}


		public override cpVect GetGrooveA()
		{
			return this.grv_a;
		}

		public override void SetGrooveA(cpVect value)
		{
			this.grv_a = value;
			this.grv_n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(this.grv_b, value)));

			this.ActivateBodies();
		}

		public override cpVect GetGrooveB()
		{
			return this.grv_b;
		}

		public override void SetGrooveB(cpVect value)
		{
			this.grv_b = value;
			this.grv_n = cpVect.cpvperp(cpVect.cpvnormalize(cpVect.cpvsub(value, this.grv_a)));

			this.ActivateBodies();
		}

		public override cpVect GetAnchorB()
		{
			return this.anchorB;
		}

		public override void SetAnchorB(cpVect anchorB)
		{
			this.ActivateBodies();
			this.anchorB = anchorB;
		}

		//public override void Draw(cpDebugDraw m_debugDraw)
		//{
		//	var a = this.a.LocalToWorld(this.grv_a);
		//	var b = this.a.LocalToWorld(this.grv_b);
		//	var c = this.b.LocalToWorld(this.anchorB);

		//	m_debugDraw.DrawSegment(a, b, cpColor.Grey);
		//	m_debugDraw.DrawCircle(c, 3f, cpColor.Grey);
		//}


	}



}

