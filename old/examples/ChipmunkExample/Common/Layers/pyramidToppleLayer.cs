using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using CocosSharp;
using ChipmunkSharp;

namespace ChipmunkExample
{
	class pyramidToppleLayer : ChipmunkDemoLayer
	{

		int WIDTH = 4;
		int HEIGHT = 20;

		public override void OnEnter()
		{
			base.OnEnter();

			space.SetIterations(30);
			space.SetGravity(new cpVect(0, -300));
			space.SetSleepTimeThreshold(0.5f);
			space.SetCollisionSlop(0.5f);


			cpShape shape = space.AddShape(new cpSegmentShape(space.GetStaticBody(), new cpVect(-600, -240), new cpVect(600, -240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);


			// Add the dominoes.
			var n = 7;
			for (var i = 0; i < n; i++)
			{
				for (var j = 0; j < (n - i); j++)
				{
					var offset = new cpVect((j - (n - 1 - i) * 0.5f) * 1.5f * HEIGHT, (i + 0.5f) * (HEIGHT + 2 * WIDTH) - WIDTH - 240);
					add_domino(offset, false);
					add_domino(cpVect.cpvadd(offset, new cpVect(0, (HEIGHT + WIDTH) / 2)), true);

					if (j == 0)
					{
						add_domino(cpVect.cpvadd(offset, new cpVect(0.5f * (WIDTH - HEIGHT), HEIGHT + WIDTH)), false);
					}

					if (j != n - i - 1)
					{
						add_domino(cpVect.cpvadd(offset, new cpVect(HEIGHT * 0.75f, (HEIGHT + 3 * WIDTH) / 2)), true);
					}
					else
					{
						add_domino(cpVect.cpvadd(offset, new cpVect(0.5f * (HEIGHT - WIDTH), HEIGHT + WIDTH)), false);
					}
				}
			}

			Schedule();


		}


		public override void Update(float dt)
		{
			base.Update(dt);

			this.space.Step(dt);
		}


		public void add_domino(cpVect pos, bool flipped)
		{


			float mass = 1f;
			float radius = 0.5f;
			float moment = cp.MomentForBox(mass, WIDTH, HEIGHT);

			var body = space.AddBody(new cpBody(mass, moment));
			body.SetPosition(pos);

			var shape = (flipped ? cpPolyShape.BoxShape(body, HEIGHT, WIDTH, 0.0f) : cpPolyShape.BoxShape(body, WIDTH - radius * 2.0f, HEIGHT, radius));
			space.AddShape(shape);
			shape.SetElasticity(0f);
			shape.SetFriction(0.6f);

		}





	}
}
