using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class jointsLayer : ChipmunkDemoLayer
	{

		cpVect boxOffset;

		private void label(string p)
		{
			CCLabelTtf tmp = GetDefaultFontTtf(p);
			tmp.Position = boxOffset.ToCCPoint();
			tmp.Scale = 0.5f;
			tmp.Position = new CCPoint(tmp.PositionX + 70, tmp.PositionY + 100);

			AddChild(tmp);
		}

		private cpBody addLever(cpVect pos)
		{
			var mass = 1;
			var a = new cpVect(0, 15);
			var b = new cpVect(0, -15);

			var body = space.AddBody(new cpBody(mass, cp.MomentForSegment(mass, a, b, 0.0f)));
			body.SetPosition(cpVect.cpvadd(pos, cpVect.cpvadd(boxOffset, new cpVect(0, -15))));

			var shape = space.AddShape(new cpSegmentShape(body, a, b, 5));
			shape.SetElasticity(0);
			shape.SetFriction(0.7f);

			return body;
		}

		private cpBody addBar(cpVect pos)
		{
			var mass = 2;
			var a = new cpVect(0, 30);
			var b = new cpVect(0, -30);

			var body = space.AddBody(new cpBody(mass, cp.MomentForSegment(mass, a, b, 0.0f)));
			body.SetPosition(cpVect.cpvadd(pos, boxOffset));

			var shape = space.AddShape(new cpSegmentShape(body, a, b, 5));
			shape.SetElasticity(0);
			shape.SetFriction(0.7f);
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));

			return body;
		}

		public cpBody addBall(cpVect pos)
		{
			var radius = 15;
			var mass = 1;
			var body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0, radius, new cpVect(0, 0))));
			body.SetPosition(cpVect.cpvadd(pos, boxOffset));

			var shape = space.AddShape(new cpCircleShape(body, radius, new cpVect(0, 0)));
			shape.SetElasticity(0);
			shape.SetFriction(0.7f);
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));

			return body;
		}

		public cpBody addWheel(cpVect pos)
		{
			var radius = 15;
			var mass = 1;
			var body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0, radius, new cpVect(0, 0))));
			body.SetPosition(cpVect.cpvadd(pos, boxOffset));

			var shape = space.AddShape(new cpCircleShape(body, radius, new cpVect(0, 0)));
			shape.SetElasticity(0);
			shape.SetFriction(0.7f);
			//shape.group = 1; // use a group to keep the car parts from colliding
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));


			return body;
		}

		public cpBody addChassis(cpVect pos)
		{
			var mass = 5;
			var width = 80;
			var height = 30;

			var body = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, width, height)));
			body.SetPosition(cpVect.cpvadd(pos, boxOffset));

			var shape = space.AddShape(cpPolyShape.BoxShape(body, width, height, 0.0f));
			shape.SetElasticity(0);
			shape.SetFriction(0.7f);
			//shape.group = 1; // use a group to keep the car parts from colliding
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));

			return body;
		}

		public override void Update(float dt)
		{
			base.Update(dt);
			space.Step(dt);
		}

		protected override void Draw()
		{
			base.Draw();

		}


		public override void OnEnter()
		{
			base.OnEnter();


			Position = new CCPoint(100, 100);

			space.SetIterations(10);
			space.SetGravity(new cpVect(0, -100));
			space.SetSleepTimeThreshold(0.5f);

			cpBody staticBody = space.GetStaticBody();// staticBody;
			cpShape shape;

			for (var y = 480; y >= 0; y -= 120)
			{
				shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(0, y), new cpVect(640, y), 0));
				shape.SetElasticity(1);
				shape.SetFriction(1);
				shape.SetFilter(NOT_GRABBABLE_FILTER);

			}

			for (var x = 0; x <= 640; x += 160)
			{
				shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(x, 0), new cpVect(x, 480), 0));
				shape.SetElasticity(1);
				shape.SetFriction(1);
				shape.SetFilter(NOT_GRABBABLE_FILTER);
			}

			cpBody body1, body2;

			var posA = new cpVect(50, 60);
			var posB = new cpVect(110, 60);

			var POS_A = new Func<cpVect>(() => { return cpVect.cpvadd(boxOffset, posA); });
			var POS_B = new Func<cpVect>(() => { return cpVect.cpvadd(boxOffset, posB); });

			// Keeps the anchor points the same distance apart from when the joint was created.
			boxOffset = new cpVect(0, 0);
			label("Pin Joint");
			body1 = addBall(posA);
			body2 = addBall(posB);
			body2.SetAngle((float)Math.PI);
			space.AddConstraint(new cpPinJoint(body1, body2, new cpVect(15, 0), new cpVect(15, 0)));

			// Slide Joints - Like pin joints but with a min/max distance.
			// Can be used for a cheap approximation of a rope.
			boxOffset = new cpVect(160, 0);
			label("Slide Joint");
			body1 = addBall(posA);
			body2 = addBall(posB);
			body2.SetAngle((float)Math.PI);
			space.AddConstraint(new cpSlideJoint(body1, body2, new cpVect(15, 0), new cpVect(15, 0), 20, 40));

			// Pivot Joints - Holds the two anchor points together. Like a swivel.
			boxOffset = new cpVect(320, 0);
			label("Pivot Joint");
			body1 = addBall(posA);
			body2 = addBall(posB);
			body2.SetAngle((float)Math.PI);
			// Alternately, specify two anchor points using cp.PivotJoint(a, b, anch1, anch2)
			space.AddConstraint(new cpPivotJoint(body1, body2, cpVect.cpvadd(boxOffset, new cpVect(80, 60))));

			// Groove Joints - Like a pivot joint, but one of the anchors is a line segment that the pivot can slide in
			boxOffset = new cpVect(480, 0);
			label("Groove Joint");
			body1 = addBall(posA);
			body2 = addBall(posB);
			space.AddConstraint(new cpGrooveJoint(body1, body2, new cpVect(30, 30), new cpVect(30, -30), new cpVect(-30, 0)));

			// Damped Springs
			boxOffset = new cpVect(0, 120);
			label("Damped Spring");
			body1 = addBall(posA);
			body2 = addBall(posB);
			body2.SetAngle((float)Math.PI);
			space.AddConstraint(new cpDampedSpring(body1, body2, new cpVect(15, 0), new cpVect(15, 0), 20, 5, 0.3f));

			// Damped Rotary Springs
			boxOffset = new cpVect(160, 120);
			label("Damped Rotary Spring");
			body1 = addBar(posA);
			body2 = addBar(posB);
			// Add some pin joints to hold the circles in place.
			space.AddConstraint(new cpPivotJoint(body1, staticBody, POS_A()));
			space.AddConstraint(new cpPivotJoint(body2, staticBody, POS_B()));
			space.AddConstraint(new cpDampedRotarySpring(body1, body2, 0, 3000, 60));

			// Rotary Limit Joint
			boxOffset = new cpVect(320, 120);
			label("Rotary Limit Joint");
			body1 = addLever(posA);
			body2 = addLever(posB);
			// Add some pin joints to hold the circles in place.
			space.AddConstraint(new cpPivotJoint(body1, staticBody, POS_A()));
			space.AddConstraint(new cpPivotJoint(body2, staticBody, POS_B()));
			// Hold their rotation within 90 degrees of each other.
			space.AddConstraint(new cpRotaryLimitJoint(body1, body2, -(float)Math.PI / 2, (float)Math.PI / 2));

			// Ratchet Joint - A rotary ratchet, like a socket wrench
			boxOffset = new cpVect(480, 120);
			label("Ratchet Joint");
			body1 = addLever(posA);
			body2 = addLever(posB);
			// Add some pin joints to hold the circles in place.
			space.AddConstraint(new cpPivotJoint(body1, staticBody, POS_A()));
			space.AddConstraint(new cpPivotJoint(body2, staticBody, POS_B()));
			// Ratchet every 90 degrees
			space.AddConstraint(new cpRatchetJoint(body1, body2, 0, (float)Math.PI / 2f));

			// Gear Joint - Maintain a specific angular velocity ratio
			boxOffset = new cpVect(0, 240);
			label("Gear Joint");
			body1 = addBar(posA);
			body2 = addBar(posB);
			// Add some pin joints to hold the circles in place.
			space.AddConstraint(new cpPivotJoint(body1, staticBody, POS_A()));
			space.AddConstraint(new cpPivotJoint(body2, staticBody, POS_B()));
			// Force one to sping 2x as fast as the other
			space.AddConstraint(new cpGearJoint(body1, body2, 0, 2));

			// Simple Motor - Maintain a specific angular relative velocity
			boxOffset = new cpVect(160, 240);
			label("Simple Motor");
			body1 = addBar(posA);
			body2 = addBar(posB);
			// Add some pin joints to hold the circles in place.
			space.AddConstraint(new cpPivotJoint(body1, staticBody, POS_A()));
			space.AddConstraint(new cpPivotJoint(body2, staticBody, POS_B()));
			// Make them spin at 1/2 revolution per second in relation to each other.
			space.AddConstraint(new cpSimpleMotor(body1, body2, (float)Math.PI));

			// Make a car with some nice soft suspension
			boxOffset = new cpVect(320, 240);
			var wheel1 = addWheel(posA);
			var wheel2 = addWheel(posB);
			var chassis = addChassis(new cpVect(80, 100));

			space.AddConstraint(new cpGrooveJoint(chassis, wheel1, new cpVect(-30, -10), new cpVect(-30, -40), new cpVect(0, 0)));
			space.AddConstraint(new cpGrooveJoint(chassis, wheel2, new cpVect(30, -10), new cpVect(30, -40), new cpVect(0, 0)));

			space.AddConstraint(new cpDampedSpring(chassis, wheel1, new cpVect(-30, 0), new cpVect(0, 0), 50, 20, 10));
			space.AddConstraint(new cpDampedSpring(chassis, wheel2, new cpVect(30, 0), new cpVect(0, 0), 50, 20, 10));

			Schedule();

		}


	}
}
