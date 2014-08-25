using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class theoJansenLayer : ChipmunkDemoLayer
	{


		static cpConstraint motor;

		static float seg_radius = 3.0f;

		public void make_leg(float side, float offset, cpBody chassis, cpBody crank, cpVect anchor)
		{
			cpVect a, b;
			cpShape shape;

			float leg_mass = 1.0f;

			// make leg
			a = cpVect.Zero;
			b = new cpVect(0.0f, side);

			cpBody upper_leg = space.AddBody(new cpBody(leg_mass, cp.MomentForSegment(leg_mass, a, b, 0.0f)));

			upper_leg.SetPosition(new cpVect(offset, 0.0f));

			shape = space.AddShape(new cpSegmentShape(upper_leg, a, b, seg_radius));
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));

			space.AddConstraint(new cpPivotJoint(chassis, upper_leg, new cpVect(offset, 0.0f), cpVect.Zero));


			//shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));

			// lower leg
			a = cpVect.Zero;
			b = new cpVect(0.0f, -1.0f * side);

			cpBody lower_leg = space.AddBody(new cpBody(leg_mass, cp.MomentForSegment(leg_mass, a, b, 0.0f)));
			lower_leg.SetPosition(new cpVect(offset, -side));

			shape = space.AddShape(new cpSegmentShape(lower_leg, a, b, seg_radius));
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));
			shape = space.AddShape(new cpCircleShape(lower_leg, seg_radius * 2.0f, b));
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));

			shape.SetElasticity(0.0f);
			shape.SetFriction(1.0f);

			space.AddConstraint(new cpPinJoint(chassis, lower_leg, new cpVect(offset, 0.0f), cpVect.Zero));

			space.AddConstraint(new cpGearJoint(upper_leg, lower_leg, 0.0f, 1.0f));

			cpPinJoint constraint;
			float diag = cp.cpfsqrt(side * side + offset * offset);

			constraint = space.AddConstraint(new cpPinJoint(crank, upper_leg, anchor, new cpVect(0.0f, side))) as cpPinJoint;

			constraint.SetDist(diag);

			constraint = space.AddConstraint(new cpPinJoint(crank, lower_leg, anchor, cpVect.Zero)) as cpPinJoint;
			constraint.SetDist(diag);
		}

		public override void OnEnter()
		{
			base.OnEnter();

		
			SetSubTitle("Use the arrow keys to control the machine.");

			space.SetIterations(20);
			space.SetGravity(new cpVect(0, -500));

			cpBody staticBody = space.GetStaticBody();
			cpShape shape;
			cpVect a, b;

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, 240), new cpVect(320, 240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(-320, 240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(320, -240), new cpVect(320, 240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(320, -240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			float offset = 30.0f;

			// make chassis
			float chassis_mass = 2.0f;
			a = new cpVect(-offset, 0.0f);
			b = new cpVect(offset, 0.0f);

			cpBody chassis = space.AddBody(new cpBody(chassis_mass, cp.MomentForSegment(chassis_mass, a, b, 0.0f)));

			shape = space.AddShape(new cpSegmentShape(chassis, a, b, seg_radius));
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));
			// make crank
			float crank_mass = 1.0f;
			float crank_radius = 13.0f;
			cpBody crank = space.AddBody(new cpBody(crank_mass, cp.MomentForCircle(crank_mass, crank_radius, 0.0f, cpVect.Zero)));

			shape = space.AddShape(new cpCircleShape(crank, crank_radius, cpVect.Zero));
			shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));

			space.AddConstraint(new cpPivotJoint(chassis, crank, cpVect.Zero, cpVect.Zero));

			float side = 30.0f;

			int num_legs = 2;
			for (int i = 0; i < num_legs; i++)
			{
				make_leg(side, offset, chassis, crank, cpVect.cpvmult(cpVect.cpvforangle((float)(2 * i + 0) / (float)num_legs * ((float)Math.PI)), crank_radius));
				make_leg(side, -offset, chassis, crank, cpVect.cpvmult(cpVect.cpvforangle((float)(2 * i + 1) / (float)num_legs * ((float)Math.PI)), crank_radius));
			}

			motor = space.AddConstraint(new cpSimpleMotor(chassis, crank, 6.0f));

			Schedule();

		}


		public override void Update(float dt)
		{
			base.Update(dt);

			float coef = (2.0f + ChipmunkDemoKeyboard.y) / 3.0f;
			float rate = ChipmunkDemoKeyboard.x * 10.0f * coef;

			motor.SetRate(rate);
			motor.SetMaxForce(rate != 0 ? 100000.0f : 0.0f);

			space.Step(dt);
		}


	}
}
