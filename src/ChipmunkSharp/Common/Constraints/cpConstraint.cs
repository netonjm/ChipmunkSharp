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

// TODO: Comment me!
using System;
namespace ChipmunkSharp
{
	public class cpConstraint
	{

		internal cpSpace space;


		/// The first body connected to this constraint.
		public cpBody a, b;

		internal cpConstraint next_a, next_b;



		/// The maximum force that this constraint is allowed to use.
		/// Defaults to infinity.
		internal float maxForce;
		/// The rate at which joint error is corrected.
		/// Defaults to pow(1.0 - 0.1, 60.0) meaning that it will
		/// correct 10% of the error every 1/60th of a second.
		internal float errorBias;
		/// The maximum rate at which joint error is corrected.
		/// Defaults to infinity.
		internal float maxBias;

		internal bool collideBodies;


		/// Function called before the solver runs.
		/// Animate your joint anchors, update your motor torque, etc.
		internal Action<cpSpace> preSolve;
		//TODO: cpConstraintPreSolveFunc

		/// Function called after the solver runs.
		/// Use the applied impulse to perform effects like breakable joints.
		//public cpConstraintPostSolveFunc postSolve;
		internal Action<cpSpace> postSolve;
		//TODO: cpConstraintPostSolveFunc

		/// User definable data pointer.
		/// Generally this points to your the game object class so you can access it
		/// when given a cpConstraint reference in a callback.
		internal object userData;


		public cpConstraint(cpBody a, cpBody b)
		{
			/// The first body connected to this constraint.
			this.a = a;
			/// The second body connected to this constraint.
			this.b = b;

			this.space = null;

			this.next_a = null;
			this.next_b = null;

			/// The maximum force that this constraint is allowed to use.
			this.maxForce = cp.Infinity;
			/// The rate at which joint error is corrected.
			/// Defaults to pow(1 - 0.1, 60) meaning that it will
			/// correct 10% of the error every 1/60th of a second.
			this.errorBias = cp.cpfpow(1f - 0.1f, 60f);
			/// The maximum rate at which joint error is corrected.
			this.maxBias = cp.Infinity;


			this.collideBodies = true;

			//Not clear
			preSolve = DefaultPreSolve;
			postSolve = DefaultPostSolve;

		}
		public cpSpace GetSpace()
		{
			return this.space;
		}

		public cpBody GetBodyA()
		{
			return this.a;
		}

		public cpBody GetBodyB()
		{
			return this.b;
		}

		public float GetMaxForce()
		{
			return maxForce;
		}

		public void SetMaxForce(float maxForce)
		{
			cp.AssertHard(maxForce >= 0.0f, "maxForce must be positive.");
			ActivateBodies();// cpConstraintActivateBodies(constraint);
			this.maxForce = maxForce;
		}

		public float GetErrorBias()
		{
			return this.errorBias;
		}

		public void SetErrorBias(float errorBias)
		{
			cp.AssertHard(errorBias >= 0.0f, "errorBias must be positive.");
			ActivateBodies();
			this.errorBias = errorBias;
		}

		public float GetMaxBias()
		{
			return this.maxBias;
		}

		public void SetMaxBias(float maxBias)
		{
			cp.AssertHard(maxBias >= 0.0f, "errorBias must be positive.");
			ActivateBodies();
			this.maxBias = maxBias;
		}

		public bool GetCollideBodies()
		{
			return collideBodies;
		}

		public void SetCollideBodies(bool value)
		{
			ActivateBodies();
			this.collideBodies = value;
		}


		public Action<cpSpace> GetPreSolveFunc()
		{
			return preSolve;
		}

		public void SetPreSolveFunc(Action<cpSpace> preSolveFunc)
		{
			this.preSolve = preSolveFunc;
		}

		public Action<cpSpace> GetPostSolveFunc()
		{
			return postSolve;
		}

		public void SetPostSolveFunc(Action<cpSpace> postSolve)
		{
			this.postSolve = postSolve;
		}

		public object GetUserData()
		{
			return userData;
		}

		public void SetUserData(object userData)
		{
			this.userData = userData;
		}

		public virtual float GetImpulse()
		{
			return 0;
		}

		public void ActivateBodies()
		{
			if (this.a != null)
				this.a.Activate();
			if (this.b != null)
				this.b.Activate();
		}

		/// /////////////////////////////////////////////////////////////



		/// These methods are overridden by the constraint itself.
		#region overriden methods

		public virtual void PreStep(float dt) { }
		public virtual void ApplyCachedImpulse(float dt_coef) { }
		public virtual void ApplyImpulse(float dt)
		{
		}

		//Function called before the solver runs. This can be overridden by the user
		//to customize the constraint.
		//Animate your joint anchors, update your motor torque, etc.
		public virtual void DefaultPreSolve(cpSpace space)
		{

		}

		//Function called after the solver runs. This can be overridden by the user
		//to customize the constraint.
		//Use the applied impulse to perform effects like breakable joints.
		public virtual void DefaultPostSolve(cpSpace space)
		{

		}

		#endregion

		public virtual cpConstraint Next(cpBody body)
		{
			return (this.a == body ? this.next_a : this.next_b);
		}


		//public virtual void Draw(cpDebugDraw m_debugDraw)
		//{

		//}


		#region OverRideMethods

		public virtual cpVect GetAnchorA()
		{
			throw new NotImplementedException();
		}

		public virtual cpVect GetAnchorB()
		{
			throw new NotImplementedException();
		}

		public virtual float GetMin()
		{
			throw new NotImplementedException();
		}

		public virtual float GetMax()
		{
			throw new NotImplementedException();
		}

		public virtual void SetMax(float max)
		{
			throw new NotImplementedException();
		}

		public virtual void SetAnchorB(cpVect anchr2)
		{
			throw new NotImplementedException();
		}

		public virtual void SetAnchorA(cpVect anchr1)
		{
			throw new NotImplementedException();
		}

		public virtual void SetMin(float min)
		{
			throw new NotImplementedException();
		}

		public virtual float GetDist()
		{
			throw new NotImplementedException();
		}

		public virtual void SetDist(float distance)
		{
			throw new NotImplementedException();
		}

		public virtual float GetRestLength()
		{
			throw new NotImplementedException();
		}

		public virtual void SetRestLength(float restLength)
		{
			throw new NotImplementedException();
		}

		public virtual float GetStiffness()
		{
			throw new NotImplementedException();
		}

		public virtual float GetDamping()
		{
			throw new NotImplementedException();
		}

		public virtual void SetStiffness(float stiffness)
		{
			throw new NotImplementedException();
		}

		public virtual void SetDamping(float damping)
		{
			throw new NotImplementedException();
		}

		public virtual cpVect GetGrooveA()
		{
			throw new NotImplementedException();
		}

		public virtual void SetGrooveA(cpVect grooveA)
		{
			throw new NotImplementedException();
		}

		public virtual cpVect GetGrooveB()
		{
			throw new NotImplementedException();
		}

		public virtual void SetGrooveB(cpVect grooveB)
		{
			throw new NotImplementedException();
		}

		public virtual float GetRestAngle()
		{
			throw new NotImplementedException();
		}

		public virtual void SetRestAngle(float restAngle)
		{
			throw new NotImplementedException();
		}

		public virtual float GetRatchet()
		{
			throw new NotImplementedException();
		}

		public virtual void SetRatchet(float ratchet)
		{
			throw new NotImplementedException();
		}

		public virtual float GetPhase()
		{
			throw new NotImplementedException();
		}

		public virtual float GetAngle()
		{
			throw new NotImplementedException();
		}

		public virtual void SetAngle(float angle)
		{
			throw new NotImplementedException();
		}

		public virtual void SetPhase(float phase)
		{
			throw new NotImplementedException();
		}

		public virtual float GetRatio()
		{
			throw new NotImplementedException();
		}

		public virtual void SetRatio(float ratchet)
		{
			throw new NotImplementedException();
		}

		public virtual float GetRate()
		{
			throw new NotImplementedException();
		}

		public virtual void SetRate(float rate)
		{
			throw new NotImplementedException();
		}

		#endregion


	}



}
