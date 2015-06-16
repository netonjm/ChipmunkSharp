using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class contactGraphLayer : ChipmunkDemoLayer
	{
        struct CrushingContext
        {
            public float magnitudeSum;
            public cpVect vectorSum;

            public CrushingContext(float magnitudeSum, cpVect vectorSum)
            {
                // TODO: Complete member initialization
                this.magnitudeSum = magnitudeSum;
                this.vectorSum = vectorSum;
            }
        };

		bool isTest = false;

		// static body that we will be making into a scale
		public cpBody scaleStaticBody;
		public cpBody ballBody;
		public cpShape shape;
        List<cpBB> draw = new List<cpBB>();

		public void ScaleIterator(cpBody body, cpArbiter arb, ref cpVect sum)
		{
			sum = cpVect.cpvadd(sum, arb.TotalImpulse());
		}

		public void BallIterator(cpBody body, cpArbiter arb, ref int count)
		{
			cpShape ball, other;
			arb.GetShapes(out ball, out other);
			draw.Add(other.bb);
			count++;
		}

		static void EstimateCrushing(cpBody body, cpArbiter arb, ref CrushingContext context)
		{
			cpVect j = arb.TotalImpulse();
			context.magnitudeSum += (float)cpVect.cpvlength(j);
			context.vectorSum = cpVect.cpvadd(context.vectorSum, j);
		}

		public override void OnEnter()
		{
			base.OnEnter();
			space.SetIterations(30);
			space.SetGravity(new cpVect(0f, -300f));
			space.SetCollisionSlop(0.5f);
			space.SetSleepTimeThreshold(1.0f);

			cpBody body, staticBody = space.GetStaticBody();

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320f, -240f), new cpVect(-320f, 240f), 0.0f));

			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(320f, -240f), new cpVect(320f, 240f), 0.0f));
			shape.SetElasticity(1f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320f, -240f), new cpVect(320f, -240f), 0.0f));
			shape.SetElasticity(1f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			//RAMP
			scaleStaticBody = space.AddBody(cpBody.NewStatic());
			shape = space.AddShape(new cpSegmentShape(scaleStaticBody, new cpVect(-240, -180), new cpVect(-140, -180), 4.0f));

			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			if (!isTest)
			{
				// add some boxes to stack on the scale
				for (int i = 0; i < 5; i++)
				{
					body = space.AddBody(new cpBody(1.0f, cp.MomentForBox(1.0f, 30.0f, 30.0f)));
					body.SetPosition(new cpVect(0f, i * 32f - 220f));

					shape = space.AddShape(cpPolyShape.BoxShape(body, 30.0f, 30.0f, 0.0f));
					shape.SetElasticity(0.0f);
					shape.SetFriction(0.8f);
				}
			}

			//Add a ball that we'll track which objects are beneath it.
			float radius = 15.0f;
			ballBody = space.AddBody(new cpBody(10.0f, cp.MomentForCircle(10.0f, 0.0f, radius, cpVect.Zero)));
			ballBody.SetPosition(new cpVect(120, -240 + radius + 5));

			shape = space.AddShape(new cpCircleShape(ballBody, radius, cpVect.Zero));
			shape.SetElasticity(0.0f);
			shape.SetFriction(0.9f);
			Schedule();
		}

		protected override void Draw()
		{
			base.Draw();
			foreach (var item in draw)
				m_debugDraw.Draw(item, cpColor.Blue);
			draw.Clear();
		}

		public override void Update(float dt)
		{
			base.Update(dt);
			space.Step(dt);
			SetSubTitle("Place objects on the scale to weigh them. The ball marks the shapes it's sitting on.\n");

			// Sum the total impulse applied to the scale from all collision pairs in the contact graph.
			// If your compiler supports blocks, your life is a little easier.
			// You can use the "Block" versions of the functions without needing the callbacks above.
			cpVect impulseSum = cpVect.Zero;
			scaleStaticBody.EachArbiter(
				(a, o) => ScaleIterator(scaleStaticBody, a, ref impulseSum)
				, null);

			// Force is the impulse divided by the timestep.
			float force = cpVect.cpvlength(impulseSum) / dt;

			// Weight can be found similarly from the gravity vector.
			cpVect g = space.GetGravity();// cpSpaceGetGravity();
			float weight = cpVect.cpvdot(g, impulseSum) / (cpVect.cpvlengthsq(g) * dt);

			SetSubTitle(string.Format("Total force: {0}, Total weight: {1}. ", force, weight));
			
            // Highlight and count the number of shapes the ball is touching.
			int count = 0;
			ballBody.EachArbiter((a, o) => BallIterator(ballBody, a, ref count), null);
			SetSubTitle(string.Format("The ball is touching {0} shapes.", count));

			CrushingContext crush = new CrushingContext(0.0f, cpVect.Zero);
			ballBody.EachArbiter((a, o) => EstimateCrushing(ballBody, a, ref crush), null);
			float crushForce = (crush.magnitudeSum - cpVect.cpvlength(crush.vectorSum)) * dt;

			if (crushForce > 10.0f)
				SetSubTitle(string.Format("The ball is being crushed. (f: {0})", crushForce));
			else
				SetSubTitle(string.Format("The ball is not being crushed. (f: {0})", crushForce));
		
            draw.Clear();
		}
	}
}
