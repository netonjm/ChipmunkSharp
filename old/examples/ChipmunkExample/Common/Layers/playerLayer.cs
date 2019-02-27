using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class playerLayer : ChipmunkDemoLayer
	{


		const float PLAYER_VELOCITY = 500f;

		const float PLAYER_GROUND_ACCEL_TIME = 0.1f;
		const float PLAYER_GROUND_ACCEL = (PLAYER_VELOCITY / PLAYER_GROUND_ACCEL_TIME);

		const float PLAYER_AIR_ACCEL_TIME = 0.25f;
		const float PLAYER_AIR_ACCEL = (PLAYER_VELOCITY / PLAYER_AIR_ACCEL_TIME);

		const float JUMP_HEIGHT = 50.0f;
		const float JUMP_BOOST_HEIGHT = 55.0f;
		const float FALL_VELOCITY = 900f;
		const float GRAVITY = 2000f;

		cpBody playerBody = null;
		cpShape playerShape = null;

		float remainingBoost = 0;
		bool grounded = false;
		bool lastJumpState = false;


		public void SelectPlayerGroundNormal(cpArbiter arb, ref cpVect groundNormal)
		{
			cpVect n = cpVect.cpvneg(arb.GetNormal());

			if (n.y > groundNormal.y)
			{
				groundNormal = n;
			}
		}

		public void playerUpdateVelocity(cpBody body, cpVect gravity, float damping, float dt)
		{
			bool jumpState = (CCMouse.Instance.dblclick);

			// Grab the grounding normal from last frame
			cpVect groundNormal = cpVect.Zero;

			playerBody.EachArbiter(
				(arbiter, o) => SelectPlayerGroundNormal(arbiter, ref groundNormal)
				, null);

			grounded = (groundNormal.y > 0.0f);
			if (groundNormal.y < 0.0f) remainingBoost = 0f;

			// Do a normal-ish update
			bool boost = (jumpState && remainingBoost > 0.0f);
			cpVect g = (boost ? cpVect.Zero : gravity);
			body.UpdateVelocity(g, damping, dt);

			// Target horizontal speed for air/ground control
			float target_vx = PLAYER_VELOCITY * ChipmunkDemoKeyboard.x;

			// Update the surface velocity and friction
			// Note that the "feet" move in the opposite direction of the player.
			cpVect surface_v = new cpVect(-target_vx, 0.0f);
			playerShape.surfaceV = surface_v;
			playerShape.u = (grounded ? PLAYER_GROUND_ACCEL / GRAVITY : 0f);

			// Apply air control if not grounded
			if (!grounded)
			{
				// Smoothly accelerate the velocity
				playerBody.v.x = cp.cpflerpconst(playerBody.v.x, target_vx, PLAYER_AIR_ACCEL * dt);
			}

			body.v.y = cp.cpfclamp(body.v.y, -FALL_VELOCITY, cp.Infinity);

		}



		public override void Update(float dt)
		{
			base.Update(dt);

			bool jumpState = (CCMouse.Instance.dblclick);

			// If the jump key was just pressed this frame, jump!
			if (jumpState && !lastJumpState && grounded)
			{
				float jump_v = cp.cpfsqrt(2 * JUMP_HEIGHT * GRAVITY);
				playerBody.v = cpVect.cpvadd(playerBody.v, new cpVect(0, jump_v));

				remainingBoost = JUMP_BOOST_HEIGHT / jump_v;
			}

			// Step the space


			space.Step(dt);

			remainingBoost -= dt;
			lastJumpState = jumpState;

		}





		public playerLayer()
		{

		}

		public override void OnEnter()
		{
			base.OnEnter();

			space.SetIterations(10);
			space.SetGravity(new cpVect(0, -GRAVITY));
			//	space.sleepTimeThreshold = 1000;

			cpBody body, staticBody = space.GetStaticBody();
			cpShape shape;

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(-320, 240), 0.0f));
			shape.SetElasticity(1.0f); shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(320, -240), new cpVect(320, 240), 0.0f));
			shape.SetElasticity(1.0f); shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(320, -240), 0.0f));
			shape.SetElasticity(1.0f); shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, 240), new cpVect(320, 240), 0.0f));
			shape.SetElasticity(1.0f); shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			// Set up the player
			body = space.AddBody(new cpBody(1.0f, cp.Infinity));
			body.p = new cpVect(0, -200);

			body.SetVelocityUpdateFunc(
				(gravity, damping, dt) =>
				playerUpdateVelocity(body, gravity, damping, dt)
				);

			playerBody = body;

			shape = space.AddShape(cpPolyShape.BoxShape2(body, new cpBB(-15, -27.5f, 15, 27.5f), 10f));

			shape.e = 0.0f; shape.u = 0.0f;
			shape.type = 1;
			playerShape = shape;

			// Add some boxes to jump on
			for (int i = 0; i < 6; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					body = space.AddBody(new cpBody(4.0f, cp.Infinity));
					body.p = new cpVect(100 + j * 60, -200 + i * 60);

					shape = space.AddShape(cpPolyShape.BoxShape(body, 50, 50, 0));
					shape.e = 0.0f;
					shape.u = 0.7f;
				}
			}

			Schedule();

		}


	}
}
