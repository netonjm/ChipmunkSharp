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

		protected override void AddedToScene()
		{
			base.AddedToScene();

			//space.iterations = 30;
			space.gravity = new cpVect(0, -100);
			space.sleepTimeThreshold = 0.5f;
			space.collisionSlop = 0.5f;

			cpBody body, staticBody = space.StaticBody;
			cpShape shape;

			this.addFloor();
			this.addWalls();

			// Add lots of boxes.
			for (var i = 0; i < 14; i++)
			{
				for (var j = 0; j <= i; j++)
				{
					body = space.AddBody(new cpBody(1, cp.MomentForBox(1, 30, 30)));
					body.SetPosition(new cpVect(j * 32 - i * 16 + 320, 540 - i * 32));

					shape = space.AddShape(cpPolyShape.BoxShape(body, 30, 30, 0.0f));
					shape.SetElasticity(0);
					shape.SetFriction(0.8f);
				}
			}

			// Add a ball to make things more interesting
			var radius = 15;
			body = space.AddBody(new cpBody(10, cp.momentForCircle(10, 0, radius, new cpVect(0, 0))));
			body.SetPosition(new cpVect(320, radius + 5));

			shape = space.AddShape(new cpCircleShape(body, radius, new cpVect(0, 0)));
			shape.SetElasticity(0);
			shape.SetFriction(0.9f);


			Schedule();

		}

		public override void OnEnter()
		{
			base.OnEnter();
		}


		public override void Update(float dt)
		{
			base.Update(dt);
			var steps = 3;
			dt /= steps;
			for (var i = 0; i < 3; i++)
			{
				this.space.Step(dt);
			}
		}




	}
}
