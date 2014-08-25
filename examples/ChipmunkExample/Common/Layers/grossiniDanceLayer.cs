using System;
using CocosSharp;
using System.Collections.Generic;
using ChipmunkSharp;

namespace ChipmunkExample
{
	public class grossiniDanceLayer : ChipmunkDemoLayer
	{


		float prevX = 0f;
		float prevY = 0f;

		int parentnode = 1;
		cpShape[] walls = new cpShape[4];
		CCSpriteBatchNode spriteTexture;

		protected override void AddedToScene()
		{
			base.AddedToScene();

			space.gravity = new cpVect(0, -300);

			walls[0] = new cpSegmentShape(space.StaticBody, new cpVect(0, 0), new cpVect(Width, 0), 0f);  //new cpSegmentShape(space.staticBody, new cpVect(0, 0), new cpVect(s.Width, 0), 0f);
			walls[1] = new cpSegmentShape(space.StaticBody, new cpVect(0, Height), new cpVect(Width, Height), 0f); // tmp.Init();
			walls[2] = new cpSegmentShape(space.StaticBody, new cpVect(0, 0), new cpVect(0, Height), 0f);
			walls[3] = new cpSegmentShape(space.StaticBody, new cpVect(Width, 0), new cpVect(Width, Height), 0f);

			foreach (var shape in walls)
			{
				shape.SetElasticity(2f);
				shape.SetFriction(1f);
				space.AddStaticShape(shape);
			}

			spriteTexture = new CCSpriteBatchNode("grossini_dance_atlas", 100);
			AddChild(spriteTexture, 0, parentnode);

			CCEventListenerAccelerometer tAcel = new CCEventListenerAccelerometer();

			tAcel.OnAccelerate = (acceleration) =>
			{
				var filterFactor = .5f;
				var accelX = (float)acceleration.Acceleration.X * filterFactor + (1 - filterFactor) * prevX;
				var accelY = (float)acceleration.Acceleration.Y * filterFactor + (1 - filterFactor) * prevY;
				prevX = accelX;
				prevY = accelY;
				space.gravity = new cpVect(300f * accelY, -300f * accelX);
			};

			AddEventListener(tAcel, this);

			addGrossiniAtPosition(windowSize.Center);

			Schedule();

		}


		public override void OnTouchesEnded(List<CCTouch> touches, CCEvent arg2)
		{
			base.OnTouchesEnded(touches, arg2);

			foreach (CCTouch touch in touches)
				addGrossiniAtPosition(CCMouse.Instance.Position.ToCCPoint());

		}

		public override void OnEnter()
		{
			base.OnEnter();
			//Position = new CCPoint((640 - Director.WindowSizeInPixels.Width) * .5f, (480 - Director.WindowSizeInPixels.Height) * .5f);
		}

		public CCPhysicsSprite addGrossiniAtPosition(CCPoint location)
		{
			int posx, posy;

			posx = (int)(CCRandom.NextDouble() * 200.0f);
			posy = (int)(CCRandom.NextDouble() * 200.0f);

			posx = (posx % 4) * 85;
			posy = (posy % 3) * 121;

			CCPhysicsSprite sp = new CCPhysicsSprite(spriteTexture.Texture, new CCRect(posx, posy, 85, 121));

			cpBB verts = new cpBB(-24, -54, 24, 54);

			var body = new cpBody(1f, cp.MomentForBox2(1f, new cpBB(-24, -54, 24, 54))); //);
			body.SetPosition(new cpVect(posx, posy));
			space.AddBody(body);

			var shape = cpPolyShape.BoxShape2(body, verts,0.0f);
			shape.e = .5f;
			shape.u = .5f;
			space.AddShape(shape);

			sp.Body = body;

			AddChild(sp);
			sp.Position = location;


			return sp;
		}

		public override void Update(float dt)
		{
			base.Update(dt);
			space.Step(dt);
		}




	}
}

