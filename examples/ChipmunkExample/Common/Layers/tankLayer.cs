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
			body.SetPosition(new cpVect(CCRandom.Float_0_1() * (640 - 2 * radius) - (320 - radius), CCRandom.Float_0_1() * (480 - 2 * radius) - (240 - radius)));


			cpShape shape = space.AddShape(cpPolyShape.BoxShape(body, size, size, 0.0f));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.7f);

			return body;
		}


		protected override void AddedToScene()
		{
			base.AddedToScene();


			SetSubTitle("Use the mouse to drive the tank, it will follow the cursor.");

			Position = new CCPoint(240, 170);

			space.SetIterations(10);
			space.sleepTimeThreshold = 0.5f;

			cpBody staticBody = space.StaticBody;
			cpShape shape;

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(0, 0), new cpVect(0, 480), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(640, 0), new cpVect(640, 480), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(0, 0), new cpVect(640, 0), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			//cpShapeSetFilter(shape, NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(0, 480), new cpVect(640, 480), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			//cpShapeSetFilter(shape, NOT_GRABBABLE_FILTER);

			for (int i = 0; i < 50; i++)
			{
				cpBody body = add_box(20, 1);

				cpConstraint pivot = space.AddConstraint(new cpPivotJoint(staticBody, body, cpVect.Zero, cpVect.Zero));
				pivot.maxBias = 0; // disable joint correction
				pivot.maxForce = 1000.0f; // emulate linear friction

				cpConstraint gear = space.AddConstraint(new cpGearJoint(staticBody, body, 0.0f, 1.0f));
				gear.maxBias = 0; // disable joint correction
				gear.maxForce = 5000.0f; // emulate linear friction

			}

			// We joint the tank to the control body and control the tank indirectly by modifying the control body.
			tankControlBody = space.AddBody(new cpBody(100, 101));
			tankBody = add_box(30f, 10f);

			cpConstraint pivot2 = space.AddConstraint(new cpPivotJoint(tankControlBody, tankBody, cpVect.Zero, cpVect.Zero));
			pivot2.maxBias = 0; // disable joint correction
			pivot2.maxForce = 10000.0f; // emulate linear friction


			cpConstraint gears = space.AddConstraint(new cpGearJoint(tankControlBody, tankBody, 0.0f, 1.0f));

			gears.errorBias = 0;// attempt to fully correct the joint each step
			gears.maxBias = 1.2f; // but limit it's angular correction rate
			gears.maxForce = 5000.0f;// emulate angular friction

			Schedule();
		}

		public override void Update(float dt)
		{
			base.Update(dt);


			if (CCMouse.Instance.HasPosition)
			{


				// turn the control body based on the angle relative to the actual body
				cpVect mouseDelta = cpVect.cpvsub(CCMouse.Instance.Position, tankBody.GetPos());
				float turn = cpVect.ToAngle(cpVect.cpvunrotate(tankBody.Rotation, mouseDelta));
				tankControlBody.SetAngle(tankBody.Angle - turn);

				// drive the tank towards the mouse
				if (cpVect.cpvnear(CCMouse.Instance.Position, tankBody.Position, 30.0f))
				{
					tankControlBody.SetVelocity(cpVect.Zero); // stop
				}
				else
				{
					float direction = (cpVect.cpvdot(mouseDelta, tankBody.Rotation) > 0.0f ? 1.0f : -1.0f);
					tankControlBody.SetVelocity(cpVect.cpvrotate(tankBody.Rotation, new cpVect(30.0f * direction, 0.0f)));
				}

			}


			space.Step(dt);
		}





	}



}
