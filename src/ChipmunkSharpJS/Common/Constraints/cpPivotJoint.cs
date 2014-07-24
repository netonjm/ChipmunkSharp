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

	public class cpPivotJoint : cpConstraint
	{

		#region PUBLIC PROPS

		public cpVect anchorA, anchorB;

		public cpVect r2 { get; set; }

		public cpVect r1 { get; set; }

		public cpVect k1 { get; set; }

		public cpVect k2 { get; set; }

		public cpVect jAcc { get; set; }

		public float jMaxLen { get; set; }

		public cpVect bias { get; set; }

		#endregion

		public override void SetAnchorA(cpVect anchr1)
		{
			this.anchorA = anchr1;
		}

		public override cpVect GetAnchorA()
		{
			return anchorA;
		}

		public override void SetAnchorB(cpVect anchr2)
		{
			this.anchorB = anchr2;
		}

		public override cpVect GetAnchorB()
		{
			return anchorB;
		}


		public cpPivotJoint(cpBody a, cpBody b, cpVect pivot)
			: this(a, b, (a != null ? a.World2Local(pivot) : pivot), (b != null ? b.World2Local(pivot) : pivot))
		{

		}
		public cpPivotJoint(cpBody a, cpBody b, cpVect anchr1, cpVect anchr2)
			: base(a, b)
		{


			// if(typeof(cpVect) ==anchr2   ) {
			if (anchr2 == null)
			{
				var pivot = anchr1;

				anchr1 = (a != null ? a.World2Local(pivot) : pivot);
				anchr2 = (b != null ? b.World2Local(pivot) : pivot);
			}

			this.anchorA = anchr1;
			this.anchorB = anchr2;

			this.r1 = cpVect.Zero;
			this.r2 = cpVect.Zero;

			this.k1 = cpVect.Zero;
			this.k2 = cpVect.Zero;

			this.jAcc = cpVect.Zero;

			this.jMaxLen = 0.0f;
			this.bias = cpVect.Zero;
		}

		public override void PreStep(float dt)
		{

			this.r1 = anchorA.Rotate(a.Rotation);// cpvrotate();
			this.r2 = anchorB.Rotate(b.Rotation); // cpvrotate(this.anchr2, b.rot);

			// Calculate mass tensor. Result is stored into this.k1 & this.k2.
			cp.k_tensor(a, b, this.r1, this.r2, this.k1, this.k2);

			// compute max impulse
			this.jMaxLen = this.maxForce * dt;

			// calculate bias velocity
			var delta = cpVect.cpvsub(cpVect.cpvadd(b.Position, this.r2), cpVect.cpvadd(a.Position, this.r1));
			this.bias = cpVect.cpvclamp(cpVect.cpvmult(delta, -cp.bias_coef(this.errorBias, dt) / dt), this.maxBias);  //delta.Multiply(-cp.bias_coef(errorBias, dt) / dt).Clamp(maxBias);  //cpvclamp(cpvmult(delta, -Util.bias_coef(this.errorBias, dt) / dt), this.maxBias);
		}

		public override void ApplyCachedImpulse(float dt_coef)
		{
			cp.apply_impulses(this.a, this.b, this.r1, this.r2, this.jAcc.x * dt_coef, this.jAcc.y * dt_coef);
		}

		public override void ApplyImpulse(float dt)
		{

			// compute relative velocity
			var vr = cp.relative_velocity(a, b, r1, r2);

			// compute normal impulse
			var j = cpVect.mult_k(bias.Sub(vr), k1, k2);   // Util.mult_k(cpvsub(this.bias, vr), this.k1, this.k2);
			var jOld = this.jAcc;
			this.jAcc = jAcc.Add(j).Clamp(jMaxLen);  // cpEnvironment.cpvclamp(cpvadd(this.jAcc, j), this.jMaxLen);

			// apply impulse
			cp.apply_impulses(a, b, this.r1, this.r2, this.jAcc.x - jOld.x, this.jAcc.y - jOld.y);
		}

		public override float GetImpulse()
		{
			return jAcc.Length;  //cpvlength(this.jAcc);
		}

		public override void Draw(cpDebugDraw m_debugDraw)
		{
			var a = this.a.Local2World(this.anchorA);
			var b = this.b.Local2World(this.anchorB);
			m_debugDraw.DrawCircle(a, 2, cpColor.Grey);
			m_debugDraw.DrawCircle(b, 2, cpColor.Grey);

		}


	}


}