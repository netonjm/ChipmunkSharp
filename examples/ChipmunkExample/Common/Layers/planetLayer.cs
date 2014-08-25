using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class planetLayer : ChipmunkDemoLayer
	{


		cpBody planetBody;

		float gravityStrength = 5.0e6f;

		protected override void AddedToScene()
		{
			base.AddedToScene();

			Position = new CCPoint(240, 170);

			space.SetIterations(20);

			planetBody = space.AddBody(new cpBody(50, 200));// cpSpaceAddBody(space, cpBodyNewKinematic());
			planetBody.SetAngularVelocity(0.2f);

			for (int i = 0; i < 30; i++)
			{
				add_box();
			}

			cpShape shape = space.AddShape(new cpCircleShape(planetBody, 70.0f, cpVect.Zero));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			//cpShapeSetFilter(shape, NOT_GRABBABLE_FILTER);

			Schedule();
		}

		static cpVect rand_pos(float radius)
		{
			cpVect v;
			do
			{
				v = new cpVect(CCRandom.Float_0_1() * (640 - 2 * radius) - (320 - radius), CCRandom.Float_0_1() * (480 - 2 * radius) - (240 - radius));
			} while (cpVect.cpvlength(v) < 85.0f);
			return v;
		}

		void planetGravityVelocityFunc(cpBody body, cpVect gravity, float damping, float dt)
		{
			// Gravitational acceleration is proportional to the inverse square of
			// distance, and directed toward the origin. The central planet is assumed
			// to be massive enough that it affects the satellites but not vice versa.
			//cpVect p = cpBodyGetPosition(body);
			float sqdist = cpVect.cpvlengthsq(body.Position);
			cpVect g = cpVect.cpvmult(body.Position, -gravityStrength / (sqdist * cp.cpfsqrt(sqdist)));
			body.VelocityFunc(g, damping, dt);
		}

		void add_box()
		{
			const float size = 10.0f;
			const float mass = 1.0f;

			float[] verts = new float[] {
		-size,-size,
		-size, size,
		 size, size,
		 size,-size,
	};

			float radius = cpVect.cpvlength(new cpVect(size, size));
			cpVect pos = rand_pos(radius);

			cpBody body = space.AddBody(new cpBody(mass, cp.momentForPoly(mass, verts, cpVect.Zero)));
			body.velocity_func = (s, f1, f2) => planetGravityVelocityFunc(body, s, f1, f2);

			body.SetPosition(pos);

			// Set the box's velocity to put it into a circular orbit from its
			// starting position.
			float r = cpVect.cpvlength(pos);
			float v = cp.cpfsqrt(gravityStrength / r) / r;
			body.SetVelocity(cpVect.cpvmult(cpVect.cpvperp(pos), v));

			// Set the box's angular velocity to match its orbital period and
			// align its initial angle with its position.
			body.SetAngularVelocity(v);
			body.SetAngle(cp.cpfatan2(pos.y, pos.x));

			cpShape shape = space.AddShape(new cpPolyShape(body, verts, 0.0f)); //cpTransformIdentity
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.7f);
		}

		public override void Update(float dt)
		{
			base.Update(dt);
			space.Step(dt);
		}




	}
}
