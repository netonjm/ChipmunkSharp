using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class tankLayer : ChipmunkDemoLayer
	{

		cpBody tankBody, tankControlBody;


		cpBody add_box(float size, float mass)
		{
			float radius = cpVect.cpvlength(new cpVect(size, size));

			cpBody body = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, size, size)));
			body.SetPosition(new cpVect((float)CCRandom.Float_0_1() * (640 - 2 * radius) - (320 - radius), (float)CCRandom.Float_0_1() * (480 - 2 * radius) - (240 - radius)));


			cpShape shape = space.AddShape(cpPolyShape.BoxShape(body, size, size, 0));
			shape.SetElasticity(0);
			shape.SetFriction(0.7f);

			return body;
		}

		public override void OnEnter()
		{
			base.OnEnter();


			SetSubTitle("Use the mouse to drive the tank, it will follow the cursor.");

			//Position = new CCPoint(240, 170);

			space.SetIterations(10);
			space.SetSleepTimeThreshold(0.5f);

			cpBody staticBody = space.GetStaticBody();
			cpShape shape;

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(-320, 240), 0));
			shape.SetElasticity(1);
			shape.SetFriction(1);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(320, -240), new cpVect(320, 240), 0));
			shape.SetElasticity(1);
			shape.SetFriction(1);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(320, -240), 0));
			shape.SetElasticity(1);
			shape.SetFriction(1);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, 240), new cpVect(320, 240), 0));
			shape.SetElasticity(1);
			shape.SetFriction(1);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			for (int i = 0; i < 50; i++)
			{
				cpBody body = add_box(20, 1);

				cpConstraint pivot = space.AddConstraint(new cpPivotJoint(staticBody, body, cpVect.Zero, cpVect.Zero));
				pivot.SetMaxBias(0); // disable joint correction
				pivot.SetMaxForce(1000); // emulate linear friction

				cpConstraint gear = space.AddConstraint(new cpGearJoint(staticBody, body, 0, 1));
				gear.SetMaxBias(0); // disable joint correction
				gear.SetMaxForce(5000); // emulate linear friction

			}

			// We joint the tank to the control body and control the tank indirectly by modifying the control body.
			tankControlBody = space.AddBody(cpBody.NewKinematic());
			tankBody = add_box(30, 10);

			cpConstraint pivot2 = space.AddConstraint(new cpPivotJoint(tankControlBody, tankBody, cpVect.Zero, cpVect.Zero));
			pivot2.SetMaxBias(0); // disable joint correction
			pivot2.SetMaxForce(10000); // emulate linear friction


			cpConstraint gears = space.AddConstraint(new cpGearJoint(tankControlBody, tankBody, 0, 1));

			gears.SetErrorBias(0);// attempt to fully correct the joint each step
			gears.SetMaxBias(1.2f); // but limit it's angular correction rate
			gears.SetMaxForce(5000);// emulate angular friction

			Schedule();

		}



		public override void Update(float dt)
		{
			base.Update(dt);


			if (CCMouse.Instance.HasPosition)
			{

				// turn the control body based on the angle relative to the actual body
				cpVect mouseDelta = cpVect.cpvsub(CCMouse.Instance.Position, tankBody.GetPosition());
				float turn = cpVect.cpvtoangle(cpVect.cpvunrotate(tankBody.GetRotation(), mouseDelta));
				tankControlBody.SetAngle(tankBody.GetAngle() - turn);

				// drive the tank towards the mouse
				if (cpVect.cpvnear(CCMouse.Instance.Position, tankBody.GetPosition(), 30))
				{
					tankControlBody.SetVelocity(cpVect.Zero); // stop
				}
				else
				{
					float direction = (cpVect.cpvdot(mouseDelta, tankBody.GetRotation()) > 0 ? 1 : -1);
					tankControlBody.SetVelocity(cpVect.cpvrotate(tankBody.GetRotation(), new cpVect(30 * direction, 0)));
				}

			}


			space.Step(dt);
		}





	}



}
