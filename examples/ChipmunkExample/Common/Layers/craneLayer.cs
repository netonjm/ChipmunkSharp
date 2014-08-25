using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class craneLayer : ChipmunkDemoLayer
	{

		enum COLLISION_TYPES
		{
			HOOK_SENSOR = 1,
			CRATE = 2,
		};

		cpBody dollyBody = null;
		// Constraint used as a servo motor to move the dolly back and forth.
		cpConstraint dollyServo = null;

		// Constraint used as a winch motor to lift the load.
		cpConstraint winchServo = null;

		// Temporary joint used to hold the hook to the load.
		cpConstraint hookJoint = null;


		protected override void AddedToScene()
		{
			base.AddedToScene();

			SetSubTitle("Control the crane by moving the mouse. Right click to release.");
			Position = new CCPoint(240, 170);

			space.SetIterations(30);
			space.gravity = new cpVect(0, -100);
			space.damping = 0.8f;

			cpBody staticBody = space.StaticBody;
			cpShape shape;

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(320, -240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);

			//cpShapeSetFilter(shape, NOT_GRABBABLE_FILTER);

			// Add a body for the dolly.
			dollyBody = space.AddBody(new cpBody(10, cp.Infinity));
			dollyBody.SetPosition(new cpVect(0, 100));

			// Add a block so you can see it.
			space.AddShape(cpPolyShape.BoxShape(dollyBody, 30, 30,0.0f));

			// Add a groove joint for it to move back and forth on.
			space.AddConstraint(new cpGrooveJoint(staticBody, dollyBody, new cpVect(-250, 100), new cpVect(250, 100), cpVect.Zero));

			// Add a pivot joint to act as a servo motor controlling it's position
			// By updating the anchor points of the pivot joint, you can move the dolly.
			dollyServo = space.AddConstraint(new cpPivotJoint(staticBody, dollyBody, dollyBody.Position));
			// Max force the dolly servo can generate.
			dollyServo.maxForce = 10000;
			// Max speed of the dolly servo
			dollyServo.maxBias = 100;
			// You can also change the error bias to control how it slows down.
			//cpConstraintSetErrorBias(dollyServo, 0.2);


			// Add the crane hook.
			cpBody hookBody = space.AddBody(new cpBody(1, cp.Infinity));
			hookBody.SetPosition(new cpVect(0, 50));

			// Add a sensor shape for it. This will be used to figure out when the hook touches a box.
			shape = space.AddShape(new cpCircleShape(hookBody, 10, cpVect.Zero));
			shape.SetSensor(true);// cpTrue);
			shape.SetCollisionType(((int)COLLISION_TYPES.HOOK_SENSOR).ToString());
			//cpShapeSetCollisionType(shape, HOOK_SENSOR);

			// Add a slide joint to act as a winch motor
			// By updating the max length of the joint you can make it pull up the load.

			winchServo = space.AddConstraint(new cpSlideJoint(dollyBody, hookBody, cpVect.Zero, cpVect.Zero, 0, cp.Infinity));
			// Max force the dolly servo can generate.
			winchServo.maxForce = 30000;
			// Max speed of the dolly servo

			winchServo.maxBias = 60;

			// TODO: cleanup
			// Finally a box to play with
			cpBody boxBody = space.AddBody(new cpBody(30, cp.MomentForBox(30, 50, 50)));

			boxBody.SetPosition(new cpVect(200, -200));

			// Add a block so you can see it.
			shape = space.AddShape(cpPolyShape.BoxShape(boxBody, 50, 50,0.0f));
			shape.SetFriction(0.7f);


			shape.SetCollisionType(((int)COLLISION_TYPES.CRATE).ToString());

			space.AddCollisionHandler(
				((int)COLLISION_TYPES.HOOK_SENSOR).ToString(),
				((int)COLLISION_TYPES.CRATE).ToString(),
				(a, s, b) => HookCrate(a, s), null, null, null
				);

			Schedule();
		}

		public override void Update(float dt)
		{
			base.Update(dt);

			if (!CCMouse.Instance.HasPosition)
			{
				// Set the first anchor point (the one attached to the static body) of the dolly servo to the mouse's x position.
				dollyServo.SetAnchorA(new cpVect(CCMouse.Instance.Position.x, 100));

				// Set the max length of the winch servo to match the mouse's height.
				winchServo.SetMax(cp.cpfmax(100 - CCMouse.Instance.Position.y, 50));

				if (hookJoint != null && CCMouse.Instance.rightclick)
				{
					space.RemoveConstraint(hookJoint);
					//cpConstraintFree(hookJoint);
					hookJoint = null;
				}

			}


			space.Step(dt);
		}


		void AttachHook(cpBody hook, cpBody crate)
		{
			hookJoint = space.AddConstraint(new cpPivotJoint(hook, crate, hook.Position));
		}


		bool HookCrate(cpArbiter arb, cpSpace space)
		{
			if (hookJoint == null)
			{
				// Get pointers to the two bodies in the collision pair and define local variables for them.
				// Their order matches the order of the collision types passed
				// to the collision handler this function was defined for
				cpBody hook = arb.body_a;
				cpBody crate = arb.body_b;


				// additions and removals can't be done in a normal callback.
				// Schedule a post step callback to do it.
				// Use the hook as the key and pass along the arbiter.
				//cpSpaceAddPostStepCallback(space, (cpPostStepFunc)AttachHook, hook, crate);

				space.AddPostStepCallback(() => AttachHook(hook, crate));

			}

			return true; // return value is ignored for sensor callbacks anyway
		}


	


	}
}
