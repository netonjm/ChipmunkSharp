using System;
using CocosSharp;
using System.Collections.Generic;
using ChipmunkSharp;

namespace ChipmunkExample
{
	public class grossiniDanceLayer : ChipmunkDemoLayer
	{

		float prevX = 0;
		float prevY = 0;

		int parentnode = 1;
		cpShape[] walls = new cpShape[4];
		CCSpriteBatchNode spriteTexture;


		public override void OnTouchesBegan(List<CCTouch> touches, CCEvent e)
		{
			base.OnTouchesBegan(touches, e);

			foreach (CCTouch touch in touches)
			{
				addGrossiniAtPosition(CCMouse.Instance.Position.ToCCPoint());
			}
		}

		//public override void OnTouchesEnded(List<CCTouch> touches, CCEvent arg2)
		//{
		//	base.OnTouchesEnded(touches, arg2);

		//	foreach (CCTouch touch in touches)
		//		addGrossiniAtPosition(CCMouse.Instance.Position.ToCCPoint());

		//}

		public override void OnEnter()
		{
			base.OnEnter();


			space.SetGravity(new cpVect(0, -300));

			var staticBody = space.GetStaticBody();

			walls[0] = new cpSegmentShape(staticBody, new cpVect(-Width * .5f, -Height * .5f), new cpVect(Width * .5f, -Height * .5f), 0);  //new cpSegmentShape(space.staticBody, new cpVect(0, 0), new cpVect(s.Width, 0), 0f);
			walls[1] = new cpSegmentShape(staticBody, new cpVect(-Width * .5f, Height * .5f), new cpVect(Width * .5f, Height * .5f), 0); // tmp.Init();
			walls[2] = new cpSegmentShape(staticBody, new cpVect(-Width * .5f, -Height * .5f), new cpVect(-Width * .5f, Height * .5f), 0);
			walls[3] = new cpSegmentShape(staticBody, new cpVect(Width * .5f, -Height * .5f), new cpVect(Width * .5f, Height * .5f), 0);

			foreach (var shape in walls)
			{
				shape.SetElasticity(2);
				shape.SetFriction(1);
				space.AddShape(shape);
			}

			spriteTexture = new CCSpriteBatchNode("grossini_dance_atlas", 100);
			AddChild(spriteTexture, 0, parentnode);

			//CCEventListenerAccelerometer tAcel = new CCEventListenerAccelerometer();

			//tAcel.OnAccelerate = (acceleration) =>
			//{
			//	var filterFactor = .5f;
			//	var accelX = (float)acceleration.Acceleration.X * filterFactor + (1 - filterFactor) * prevX;
			//	var accelY = (float)acceleration.Acceleration.Y * filterFactor + (1 - filterFactor) * prevY;
			//	prevX = accelX;
			//	prevY = accelY;
			//	space.gravity = new cpVect(300f * accelY, -300f * accelX);
			//};

			//AddEventListener(tAcel, this);

			addGrossiniAtPosition(CCPoint.Zero);

			Schedule();

		}

		public CCPhysicsSprite addGrossiniAtPosition(CCPoint location)
		{
			int posx, posy;

			posx = CCRandom.Next() * 200;
			posy = CCRandom.Next() * 200;

			posx = (Math.Abs(posx) % 4) * 85;
			posy = (Math.Abs(posy) % 3) * 121;

			CCPhysicsSprite sp = new CCPhysicsSprite(spriteTexture.Texture, new CCRect(posx, posy, 85, 121));

			cpBB verts = new cpBB(-24, -54, 24, 54);

			var body = new cpBody(1, cp.MomentForBox2(1, new cpBB(-24, -54, 24, 54))); //);
			body.SetPosition(new cpVect(posx, posy));
			space.AddBody(body);

			var shape = cpPolyShape.BoxShape2(body, verts, 0);
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

