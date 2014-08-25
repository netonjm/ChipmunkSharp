using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{

	class ballLayer : ChipmunkDemoLayer
	{




		protected override void AddedToScene()
		{
			base.AddedToScene();

			//SetScale(windowSize);

			//PositionX += (windowSize.Width - 640) * .5f;  //new CCPoint(150, 150);

			space.iterations = 60;
			space.gravity = new cpVect(0, -500);
			space.sleepTimeThreshold = 0.5f;
			space.collisionSlop = 0.5f;
			space.sleepTimeThreshold = 0.5f;

			this.addFloor();
			this.addWalls();

			float width = 50;
			float height = 60;
			float mass = width * height * 1 / 1000;


			var rock = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, width, height)));
			rock.Position = new cpVect(500, 100);
			rock.SetAngle(1);

			cpPolyShape shape = space.AddShape(cpPolyShape.BoxShape(rock, width, height,0.0f)) as cpPolyShape;

			shape.SetFriction(0.3f);
			shape.SetElasticity(0.3f);


			for (var i = 1; i <= 12; i++)
			{
				float radius = 20;
				mass = 3;

				var body = space.AddBody(new cpBody(mass, cp.momentForCircle(mass, 0, radius, cpVect.Zero)));

				body.Position = new cpVect(200 + i, (2 * radius + 5) * i);

				cpCircleShape circle = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero)) as cpCircleShape;
				circle.SetElasticity(0.8f);
				circle.SetFriction(1f);
			}

			var ramp = space.AddShape(new cpSegmentShape(space.StaticBody, new cpVect(100, 100), new cpVect(300, 200), 10));

			ramp.SetElasticity(1f);
			ramp.SetFriction(1f);

			ramp.SetGroup(cp.NOT_GRABABLE_MASK);

			Schedule();
		}

		public override void OnEnter()
		{
			base.OnEnter();
			//Position = new CCPoint((640 - Director.WindowSizeInPixels.Width) * .5f, (480 - Director.WindowSizeInPixels.Height) * .5f);
		}

		public override void Update(float dt)
		{
			base.Update(dt);
			space.Step(dt);
		}



	}
}
