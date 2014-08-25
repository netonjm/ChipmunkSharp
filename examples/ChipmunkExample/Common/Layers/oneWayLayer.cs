using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{



	class oneWayLayer : ChipmunkDemoLayer
	{
		struct OneWayPlatform
		{
			public cpVect n; // direction objects may pass through
		}

		enum CollisionTypes
		{
			COLLISION_TYPE_ONE_WAY = 1,
		};

		OneWayPlatform platformInstance;

		protected override void AddedToScene()
		{
			base.AddedToScene();

			Position = new CCPoint(240, 170);


			platformInstance = new OneWayPlatform();

			SetSubTitle("One way platforms are trivial in Chipmunk using a very simple collision callback.");

			space.SetIterations(10);
			space.gravity = new cpVect(0, -100);

			cpBody body, staticBody = space.StaticBody;
			cpShape shape;

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(-320, 240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			//cpShapeSetFilter(shape, NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(320, -240), new cpVect(320, 240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);

			//cpShapeSetFilter(shape, NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(320, -240), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			//cpShapeSetFilter(shape, NOT_GRABBABLE_FILTER);

			// Add our one way segment
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-160, -100), new cpVect(160, -100), 10.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);

			shape.SetCollisionType(COLLISION_TYPE_ONE_WAY);
			//shape. cpShapeSetFilter(shape, NOT_GRABBABLE_FILTER);

			// We'll use the data pointer for the OneWayPlatform struct
			platformInstance.n = new cpVect(0, 1); // let objects pass upwards
			shape.userData = platformInstance;


			// Add a ball to test it out
			float radius = 15.0f;
			body = space.AddBody(new cpBody(10.0f, cp.momentForCircle(10.0f, 0.0f, radius, cpVect.Zero)));
			body.SetPosition(new cpVect(0, -200));
			body.SetVelocity(new cpVect(0, 170));

			shape = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.9f);
			shape.SetCollisionType("2");


			//CollisionHandler  handler = space.add
			//cpCollisionHandler* handler = cpSpaceAddWildcardHandler(space, COLLISION_TYPE_ONE_WAY);
			//handler->preSolveFunc = PreSolve;


			Schedule();
		}

		static bool preSolve(cpArbiter arb, cpSpace space)
		{
			cpShape a, b;
			arb.GetShapes(out a, out b);

			OneWayPlatform platform = (OneWayPlatform)a.userData;// (OneWayPlatform*)cpShapeGetUserData(a);

			if (cpVect.cpvdot(arb.GetNormal(0), platform.n) < 0)
			{
				arb.Ignore();// cpArbiterIgnore(arb);
				return false;
			}

			return true;
		}


		public override void Update(float dt)
		{
			base.Update(dt);

			space.Step(dt);
		}

	


		public string COLLISION_TYPE_ONE_WAY { get; set; }
	}
}
