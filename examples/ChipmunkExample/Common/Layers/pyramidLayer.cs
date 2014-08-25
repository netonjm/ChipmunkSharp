using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class pyramidLayer : ChipmunkDemoLayer
	{

		public override void OnEnter()
		{
			base.OnEnter();

			space.SetIterations(30);
			space.SetGravity(new cpVect(0, -100));
			space.SetSleepTimeThreshold(0.5f);
			space.SetCollisionSlop(0.5f);

			cpBody body, staticBody = space.GetStaticBody();
			cpShape shape;

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

			// Add lots of boxes.
			for (var i = 0; i < 10; i++)
			{
				for (var j = 0; j <= i; j++)
				{
					body = space.AddBody(new cpBody(1, cp.MomentForBox(1, 30, 30)));
					body.SetPosition(new cpVect(j * 32 - i * 16 + 0, 300 - i * 32));

					shape = space.AddShape(cpPolyShape.BoxShape(body, 30, 30, 0.5f));
					shape.SetElasticity(0);
					shape.SetFriction(0.8f);
				}
			}

			// Add a ball to make things more interesting
			float radius = 15f;
			body = space.AddBody(new cpBody(10, cp.MomentForCircle(10f, 0, radius, cpVect.Zero)));
			body.SetPosition(new cpVect(0, -240 + radius + 5));

			shape = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.9f);

			Schedule();

		}


		public override void Update(float dt)
		{
			base.Update(dt);

			this.space.Step(dt);
		}




	}
}
