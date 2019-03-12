namespace ChipmunkSharp.Example
{
	class BallLayer : ChipmunkDemoLayer
	{
		public BallLayer (PhysicsDrawView debugDraw, cpSpace space) : base (debugDraw, space)
		{
			//PositionX += (windowSize.Width - 640) * .5d;  //new CCPoint(150, 150);
			space.SetIterations (60);
			space.SetGravity (new cpVect (0, -500));
			space.SetSleepTimeThreshold (0.5f);
			space.SetCollisionSlop (0.5f);
			space.SetSleepTimeThreshold (0.5f);

			AddFloor ();
			AddWalls ();

			float width = 50;
			float height = 60;
			float mass = width * height * 1f / 1000f;

			var rock = space.AddBody (new cpBody (mass, cp.MomentForBox (mass, width, height)));
			rock.SetPosition (new cpVect (100, 200));
			rock.SetAngle (1);

			cpPolyShape shape = space.AddShape (cpPolyShape.BoxShape (rock, width, height, 0.0f)) as cpPolyShape;
			shape.SetFriction (0.3f);
			shape.SetElasticity (0.3f);
			shape.SetFilter (NOT_GRABBABLE_FILTER); //The box cannot be dragg
			for (var i = 0; i <= 6; i++) {
				float radius = 20f;
				mass = 3;
				var body = space.AddBody (new cpBody (mass, cp.MomentForCircle (mass, 0f, radius, cpVect.Zero)));
				body.SetPosition (new cpVect (i +100, ((2 * radius + 5) * 1) + 100));

				cpCircleShape circle = space.AddShape (new cpCircleShape (body, radius, cpVect.Zero)) as cpCircleShape;
				circle.SetElasticity (0.8f);
				circle.SetFriction (1);
			}

			var ramp = space.AddShape (new cpSegmentShape (space.GetStaticBody (), new cpVect (20, 20), new cpVect (300, 22), 10));
			ramp.SetElasticity (1f);
			ramp.SetFriction (1f);
			ramp.SetFilter (NOT_GRABBABLE_FILTER);
		}

		public override void Update (float dt)
		{
			space.Step (1f / 60f);
			m_debugDraw.DebugDraw ();
		}
	}
}
