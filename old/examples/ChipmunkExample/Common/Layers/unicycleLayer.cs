using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class unicycleLayer : ChipmunkDemoLayer
	{





		static cpBody balance_body;
		static float balance_sin = 0.0f;
		//static cpfloat last_v = 0.0;

		static cpBody wheel_body;
		static cpConstraint motor;

		public override void OnEnter()
		{
			base.OnEnter();

			SetSubTitle("This unicycle is completely driven and balanced by a single cpSimpleMotor.\nMove the mouse to make the unicycle follow it.");

			space.SetIterations(30);
			space.SetGravity(new cpVect(0, -500));

			{
				cpShape shape = null;
				cpBody staticBody = space.GetStaticBody();

				shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-3200, -240), new cpVect(3200, -240), 0.0f));
				shape.SetElasticity(1.0f);
				shape.SetFriction(1.0f);
				shape.SetFilter(NOT_GRABBABLE_FILTER);

				shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(0, -200), new cpVect(240, -240), 0.0f));
				shape.SetElasticity(1.0f);
				shape.SetFriction(1.0f);
				shape.SetFilter(NOT_GRABBABLE_FILTER);

				shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-240, -240), new cpVect(0, -200), 0.0f));
				shape.SetElasticity(1.0f);
				shape.SetFriction(1.0f);
				shape.SetFilter(NOT_GRABBABLE_FILTER);
			}


			{
				float radius = 20.0f;
				float mass = 1.0f;

				float moment = cp.MomentForCircle(mass, 0.0f, radius, cpVect.Zero);

				wheel_body = space.AddBody(new cpBody(mass, moment));
				wheel_body.SetPosition(new cpVect(0.0f, -160.0f + radius));

				cpShape shape = space.AddShape(new cpCircleShape(wheel_body, radius, cpVect.Zero));
				shape.SetFriction(0.7f);
				shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));
			}

			{
				float cog_offset = 30.0f;

				cpBB bb1 = new cpBB(-5.0f, 0.0f - cog_offset, 5.0f, cog_offset * 1.2f - cog_offset);
				cpBB bb2 = new cpBB(-25.0f, bb1.t, 25.0f, bb1.t + 10.0f);

				float mass = 3.0f;
				float moment = cp.MomentForBox2(mass, bb1) + cp.MomentForBox2(mass, bb2);

				balance_body = space.AddBody(new cpBody(mass, moment));
				balance_body.SetPosition(new cpVect(0.0f, wheel_body.GetPosition().y + cog_offset));

				cpShape shape = null;

				shape = space.AddShape(cpPolyShape.BoxShape2(balance_body, bb1, 0.0f));
				shape.SetFriction(1.0f);
				shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));

				shape = space.AddShape(cpPolyShape.BoxShape2(balance_body, bb2, 0.0f));
				shape.SetFriction(1.0f);
				shape.SetFilter(new cpShapeFilter(1, cp.ALL_CATEGORIES, cp.ALL_CATEGORIES));
			}

			cpVect anchorA = balance_body.WorldToLocal(wheel_body.GetPosition());
			cpVect groove_a = cpVect.cpvadd(anchorA, new cpVect(0.0f, 30.0f));
			cpVect groove_b = cpVect.cpvadd(anchorA, new cpVect(0.0f, -10.0f));
			space.AddConstraint(new cpGrooveJoint(balance_body, wheel_body, groove_a, groove_b, cpVect.Zero));
			space.AddConstraint(new cpDampedSpring(balance_body, wheel_body, anchorA, cpVect.Zero, 0.0f, 6.0e2f, 30.0f));

			motor = space.AddConstraint(new cpSimpleMotor(wheel_body, balance_body, 0.0f));
			motor.SetPreSolveFunc((s) => motor_preSolve(motor, s));

			{
				float width = 100.0f;
				float height = 20.0f;
				float mass = 3.0f;

				cpBody boxBody = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, width, height)));
				boxBody.SetPosition(new cpVect(200, -100));

				cpShape shape = space.AddShape(cpPolyShape.BoxShape(boxBody, width, height, 0.0f));
				shape.SetFriction(0.7f);
			}


			Schedule();

		}



		void motor_preSolve(cpConstraint motor, cpSpace space)
		{
			float dt = space.GetCurrentTimeStep();

			float target_x = CCMouse.Instance.Position.x;

			paint = new shape
			{
				point1 = new cpVect(target_x, -1000.0f),
				point2 = new cpVect(target_x, 1000.0f),
			};

			float max_v = 500.0f;
			float target_v = cp.cpfclamp(cp.bias_coef(0.5f, dt / 1.2f) * (target_x - balance_body.GetPosition().x) / dt, -max_v, max_v);
			float error_v = (target_v - balance_body.GetVelocity().x);
			float target_sin = 3.0e-3f * cp.bias_coef(0.1f, dt) * error_v / dt;

			float max_sin = cp.cpfsin(0.6f);
			balance_sin = cp.cpfclamp(balance_sin - 6.0e-5f * cp.bias_coef(0.2f, dt) * error_v / dt, -max_sin, max_sin);
			float target_a = (float)Math.Asin(cp.cpfclamp(-target_sin + balance_sin, -max_sin, max_sin));
			float angular_diff = (float)Math.Asin(cpVect.cpvcross(balance_body.GetRotation(), cpVect.cpvforangle(target_a)));
			float target_w = cp.bias_coef(0.1f, dt / 0.4f) * (angular_diff) / dt;

			float max_rate = 50.0f;
			float rate = cp.cpfclamp(wheel_body.GetAngularVelocity() + balance_body.GetAngularVelocity() - target_w, -max_rate, max_rate);
			motor.SetRate(cp.cpfclamp(rate, -max_rate, max_rate));
			motor.SetMaxForce(8.0e4f);
		}

		shape paint;

		class shape
		{
			public cpVect point1;
			public cpVect point2;
		}

		protected override void Draw()
		{
			base.Draw();
			if (paint != null)
			{
				m_debugDraw.DrawSegment(paint.point1, paint.point2, 1, cpColor.Red);
			}
		}

		public override void Update(float dt)
		{
			base.Update(dt);
			space.Step(dt);
		}

	}
}
