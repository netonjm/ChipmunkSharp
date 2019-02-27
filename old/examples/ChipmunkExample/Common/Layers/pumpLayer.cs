using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class pumpLayer : ChipmunkDemoLayer
	{


		int numBalls = 5;
		cpConstraint motor;
		cpBody[] balls;


		public override void Update(float dt)
		{
			base.Update(dt);

			float coef = (2.0f + ChipmunkDemoKeyboard.y) / 3.0f;
			float rate = ChipmunkDemoKeyboard.x * 30.0f * coef;

			motor.SetRate(rate);
			motor.SetMaxForce(rate > 0 ? 1000000.0f : 0.0f);

			space.Step(dt);

			for (int i = 0; i < numBalls; i++)
			{
				cpBody ball = balls[i];
				cpVect pos = ball.GetPosition();

				if (pos.x > 320.0f)
				{
					ball.SetVelocity(cpVect.Zero);
					ball.SetPosition(new cpVect(-224.0f, 200.0f));
				}
			}


		}


		public cpBody add_ball(cpSpace space, cpVect pos)
		{
			cpBody body = space.AddBody(new cpBody(1.0f, cp.MomentForCircle(1.0f, 30, 0, cpVect.Zero)));
			body.SetPosition(pos);

			cpShape shape = space.AddShape(new cpCircleShape(body, 30, cpVect.Zero));


			shape.SetElasticity(0.0f);
			shape.SetFriction(0.5f);


			return body;
		}




		public override void OnEnter()
		{
			base.OnEnter();

			SetSubTitle("Use the arrow keys to control the machine.");


			space.SetGravity(new cpVect(0, -600));

			cpBody staticBody = space.GetStaticBody();
			cpShape shape;

			// beveling all of the line segments slightly helps prevent things from getting stuck on cracks
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-256, 16), new cpVect(-256, 300), 2.0f));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.5f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-256, 16), new cpVect(-192, 0), 2.0f));
			shape.SetElasticity(0.0f);
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.5f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-192, 0), new cpVect(-192, -64), 2.0f));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.5f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-128, -64), new cpVect(-128, 144), 2.0f));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.5f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-192, 80), new cpVect(-192, 176), 2.0f));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.5f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-192, 176), new cpVect(-128, 240), 2.0f));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.5f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-128, 144), new cpVect(192, 64), 2.0f));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.5f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			cpVect[] verts = new cpVect[]{
		new cpVect(-30,-80),
		new cpVect(-30, 80),
		new cpVect( 30, 64),
		new cpVect( 30,-80),
	};

			cpBody plunger = space.AddBody(new cpBody(1.0f, cp.Infinity));
			plunger.SetPosition(new cpVect(-160, -80));

			shape = space.AddShape(new cpPolyShape(plunger, 4, verts, cpTransform.Identity, 0));
			shape.SetElasticity(1.0f);
			shape.SetFriction(0.5f);
			shape.SetFilter(new cpShapeFilter(cp.NO_GROUP, 1, 1));
			balls = new cpBody[numBalls];
			// add balls to hopper
			for (int i = 0; i < numBalls; i++)
				balls[i] = add_ball(space, new cpVect(-224 + i, 80 + 64 * i));

			// add small gear
			cpBody smallGear = space.AddBody(new cpBody(10.0f, cp.MomentForCircle(10.0f, 80, 0, cpVect.Zero)));
			smallGear.SetPosition(new cpVect(-160, -160));
			smallGear.SetAngle(-cp.M_PI_2);

			shape = space.AddShape(new cpCircleShape(smallGear, 80.0f, cpVect.Zero));
			shape.SetFilter(cpShape.FILTER_NONE);

			space.AddConstraint(new cpPivotJoint(staticBody, smallGear, new cpVect(-160, -160), cpVect.Zero));

			// add big gear
			cpBody bigGear = space.AddBody(new cpBody(40.0f, cp.MomentForCircle(40.0f, 160, 0, cpVect.Zero)));
			bigGear.SetPosition(new cpVect(80, -160));
			bigGear.SetAngle(cp.M_PI_2);

			shape = space.AddShape(new cpCircleShape(bigGear, 160.0f, cpVect.Zero));
			shape.SetFilter(cpShape.FILTER_NONE);

			space.AddConstraint(new cpPivotJoint(staticBody, bigGear, new cpVect(80, -160), cpVect.Zero));

			// connect the plunger to the small gear.
			space.AddConstraint(new cpPinJoint(smallGear, plunger, new cpVect(80, 0), new cpVect(0, 0)));
			// connect the gears.
			space.AddConstraint(new cpGearJoint(smallGear, bigGear, -cp.M_PI_2, -2.0f));


			// feeder mechanism
			float bottom = -300.0f;
			float top = 32.0f;
			cpBody feeder = space.AddBody(new cpBody(1.0f, cp.MomentForSegment(1.0f, new cpVect(-224.0f, bottom), new cpVect(-224.0f, top), 0.0f)));
			feeder.SetPosition(new cpVect(-224, (bottom + top) / 2.0f));

			float len = top - bottom;
			shape = space.AddShape(new cpSegmentShape(feeder, new cpVect(0.0f, len / 2.0f), new cpVect(0.0f, -len / 2.0f), 20.0f));
			shape.SetFilter(GRAB_FILTER);

			space.AddConstraint(new cpPivotJoint(staticBody, feeder, new cpVect(-224.0f, bottom), new cpVect(0.0f, -len / 2.0f)));
			cpVect anchr = feeder.WorldToLocal(new cpVect(-224.0f, -160.0f));
			space.AddConstraint(new cpPinJoint(feeder, smallGear, anchr, new cpVect(0.0f, 80.0f)));

			// motorize the second gear
			motor = space.AddConstraint(new cpSimpleMotor(staticBody, bigGear, 3.0f));


			Schedule();

		}


	}
}
