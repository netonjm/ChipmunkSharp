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
namespace ChipmunkSharp.Constraints
{
    public class cpConstraint
    {

        //cpConstraintClass klass;

        /// The first body connected to this constraint.
        public cpBody a;
        /// The second body connected to this constraint.
        public cpBody b;

        public cpSpace space;

        public cpConstraint next_a;
        public cpConstraint next_b;

        /// The maximum force that this constraint is allowed to use.
        /// Defaults to infinity.
        public float maxForce;
        /// The rate at which joint error is corrected.
        /// Defaults to pow(1.0 - 0.1, 60.0) meaning that it will
        /// correct 10% of the error every 1/60th of a second.
        public float errorBias;
        /// The maximum rate at which joint error is corrected.
        /// Defaults to infinity.
        public float maxBias;

        /// Function called before the solver runs.
        /// Animate your joint anchors, update your motor torque, etc.
        //public cpConstraintPreSolveFunc preSolve;
        //TODO: cpConstraintPreSolveFunc

        /// Function called after the solver runs.
        /// Use the applied impulse to perform effects like breakable joints.
        //public cpConstraintPostSolveFunc postSolve;
        //TODO: cpConstraintPostSolveFunc

        /// User definable data pointer.
        /// Generally this points to your the game object class so you can access it
        /// when given a cpConstraint reference in a callback.
        //public cpDataPointer data;

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
            this.errorBias = (float)Math.Pow(1 - 0.1, 60);
            /// The maximum rate at which joint error is corrected.
            this.maxBias = cp.Infinity;
        }

        public void activateBodies()
        {
            if (a != null)
                a.activate();

            if (b != null)
                b.activate();
        }

        /// These methods are overridden by the constraint itself.
        #region overriden methods

        public virtual void PreStep(float dt) { }
        public virtual void ApplyCachedImpulse(float dt_coef) { }
        public virtual void ApplyImpulse(float dt)
        {
        }
        public virtual float GetImpulse()
        {
            return 0;
        }

        /// Function called before the solver runs. This can be overridden by the user
        /// to customize the constraint.
        /// Animate your joint anchors, update your motor torque, etc.
        public virtual void PreSolve(cpSpace space)
        {

        }

        /// Function called after the solver runs. This can be overridden by the user
        /// to customize the constraint.
        /// Use the applied impulse to perform effects like breakable joints.
        public virtual void postSolve(cpSpace space)
        {

        }

        #endregion

        public virtual cpConstraint next(cpBody body)
        {
            return (this.a == body ? this.next_a : this.next_b);
        }


        //    typedef void (*cpConstraintPreStepImpl)(cpConstraint *constraint, cpFloat dt);
        //typedef void (*cpConstraintApplyCachedImpulseImpl)(cpConstraint *constraint, cpFloat dt_coef);
        //typedef void (*cpConstraintApplyImpulseImpl)(cpConstraint *constraint, cpFloat dt);
        //typedef cpFloat (*cpConstraintGetImpulseImpl)(cpConstraint *constraint);



        public virtual cpVect getAnchr1()
        {
            throw new NotImplementedException();
        }

        public virtual cpVect getAnchr2()
        {
            throw new NotImplementedException();
        }

        public virtual float getMin()
        {
            throw new NotImplementedException();
        }

        public virtual float getMax()
        {
            throw new NotImplementedException();
        }

        public virtual void setMax(float max)
        {
            throw new NotImplementedException();
        }

        public virtual void setAnchr2(cpVect anchr2)
        {
            throw new NotImplementedException();
        }

        public virtual void setAnchr1(cpVect anchr1)
        {
            throw new NotImplementedException();
        }

        public virtual void setMin(float min)
        {
            throw new NotImplementedException();
        }

        public virtual float getDist()
        {
            throw new NotImplementedException();
        }

        public virtual void setDist(float distance)
        {
            throw new NotImplementedException();
        }

        public virtual float getRestLength()
        {
            throw new NotImplementedException();
        }

        public virtual void setRestLength(float restLength)
        {
            throw new NotImplementedException();
        }

        public virtual float getStiffness()
        {
            throw new NotImplementedException();
        }

        public virtual float getDamping()
        {
            throw new NotImplementedException();
        }

        public virtual void setStiffness(float stiffness)
        {
            throw new NotImplementedException();
        }

        public virtual void setDamping(float damping)
        {
            throw new NotImplementedException();
        }

        public virtual cpVect getGrooveA()
        {
            throw new NotImplementedException();
        }

        public virtual void setGrooveA(cpVect grooveA)
        {
            throw new NotImplementedException();
        }

        public virtual cpVect getGrooveB()
        {
            throw new NotImplementedException();
        }

        public virtual void setGrooveB(cpVect grooveB)
        {
            throw new NotImplementedException();
        }

        public virtual float getRestAngle()
        {
            throw new NotImplementedException();
        }

        public virtual void setRestAngle(float restAngle)
        {
            throw new NotImplementedException();
        }

        public virtual float getRatchet()
        {
            throw new NotImplementedException();
        }

        public virtual void setRatchet(float ratchet)
        {
            throw new NotImplementedException();
        }

        public virtual float getPhase()
        {
            throw new NotImplementedException();
        }

        public virtual float getAngle()
        {
            throw new NotImplementedException();
        }

        public virtual void setAngle(float angle)
        {
            throw new NotImplementedException();
        }

        public virtual void setPhase(float phase)
        {
            throw new NotImplementedException();
        }

        public virtual float getRatio()
        {
            throw new NotImplementedException();
        }

        public virtual void setRatio(float ratchet)
        {
            throw new NotImplementedException();
        }

        public virtual float getRate()
        {
            throw new NotImplementedException();
        }

        public virtual void setRate(float rate)
        {
            throw new NotImplementedException();
        }

        public virtual void Draw(cpDebugDraw m_debugDraw)
        {

        }



    }



    /// @defgroup cpConstraint cpConstraint
    /// @{

    // struct cpConstraintClass cpConstraintClass;

    delegate void cpConstraintPreStepImpl(cpConstraint constraint, float dt);
    delegate void cpConstraintApplyCachedImpulseImpl(cpConstraint constraint, float dt_coef);
    delegate void cpConstraintApplyImpulseImpl(cpConstraint constraint, float dt);
    delegate float cpConstraintGetImpulseImpl(cpConstraint constraint);


    struct cpConstraintClass
    {

        cpConstraintPreStepImpl preStep;
        cpConstraintApplyCachedImpulseImpl applyCachedImpulse;
        cpConstraintApplyImpulseImpl applyImpulse;
        cpConstraintGetImpulseImpl getImpulse;

        void cpConstraintDestroy(cpConstraint constraint) { }

        void cpConstraintFree(cpConstraint constraint)
        {
            if (constraint != null)
            {

                cpConstraintDestroy(constraint);
                //cpfree(constraint);
            }
        }

        // *** declared in util.h TODO move declaration to chipmunk_private.h


    }
}
