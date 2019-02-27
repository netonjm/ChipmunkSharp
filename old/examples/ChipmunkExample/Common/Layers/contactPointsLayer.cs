using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class contactPointsLayer : ChipmunkDemoLayer
	{

		public ulong COLLISION_TYPE_ONE_WAY = 1;

		static bool NeverCollide(cpArbiter arb, cpSpace space, object data) {
			return false;
		}

		public override void OnEnter()
		{
			base.OnEnter();
			space.SetIterations(5);
			space.SetDamping(0.1f);

			//space.defaultHandler.
			cpCollisionHandler handler = space.AddWildcardHandler(COLLISION_TYPE_ONE_WAY);
			handler.preSolveFunc = NeverCollide;
			//space.def SetDefaultCollisionHandler(space, NeverCollide, NULL, NULL, NULL, NULL);

			{
				float mass = 1.0f;
				float length = 100.0f;
				cpVect a = new cpVect(-length / 2.0f, 0.0f), b = new cpVect(length / 2.0f, 0.0f);

				cpBody body = space.AddBody(new cpBody(mass, cp.MomentForSegment(mass, a, b, 0.0f)));
				body.SetPosition(new cpVect(-160.0f, -80.0f));

				space.AddShape(new cpSegmentShape(body, a, b, 30.0f));
			}
			{
				float mass = 1.0f;
				float length = 100.0f;
				cpVect a = new cpVect(-length / 2.0f, 0.0f), b = new cpVect(length / 2.0f, 0.0f);
				cpBody body = space.AddBody(new cpBody(mass, cp.MomentForSegment(mass, a, b, 0.0f)));
				body.SetPosition(new cpVect(-160.0f, 80.0f));
				space.AddShape(new cpSegmentShape(body, a, b, 20.0f));
			}
			{
				float mass = 1.0f;
				int NUM_VERTS = 5;
				cpVect[] verts = new cpVect[NUM_VERTS];
				for (int i = 0; i < NUM_VERTS; i++)
				{
					float angle = -2 * cp.M_PI * i / NUM_VERTS;
					verts[i] = new cpVect(40 * cp.cpfcos(angle), 40 * cp.cpfsin(angle));
				}

				cpBody body = space.AddBody(new cpBody(mass, cp.MomentForPoly(mass, NUM_VERTS, verts, cpVect.Zero, 0.0f)));
				body.SetPosition(new cpVect(-0.0f, -80.0f));
				space.AddShape(new cpPolyShape(body, NUM_VERTS, verts, 0.0f));
			}

			{
				float mass = 1.0f;
				int NUM_VERTS = 4;

				cpVect[] verts = new cpVect[NUM_VERTS];
				for (int i = 0; i < NUM_VERTS; i++)
				{
					float angle = -2 * cp.M_PI * i / NUM_VERTS;
					verts[i] = new cpVect(60 * cp.cpfcos(angle), 60 * cp.cpfsin(angle));
				}

				cpBody body = space.AddBody(new cpBody(mass, cp.MomentForPoly(mass, NUM_VERTS, verts, cpVect.Zero, 0.0f)));
				body.SetPosition(new cpVect(-0.0f, 80.0f));
				space.AddShape(new cpPolyShape(body, NUM_VERTS, verts, 0));
			}

			{
				float mass = 1.0f;
				float r = 60.0f;

				cpBody body = space.AddBody(new cpBody(mass, cp.Infinity));
				body.SetPosition(new cpVect(160, -80));
				space.AddShape(new cpCircleShape(body, r, cpVect.Zero));
			}

			{
				float mass = 1.0f;
				float r = 40.0f;

				cpBody body = space.AddBody(new cpBody(mass, cp.Infinity));
				body.SetPosition(new cpVect(160, 80));
				space.AddShape(new cpCircleShape(body, r, cpVect.Zero));
			}

			Schedule();
		}

		public override void Update(float dt)
		{
			base.Update(dt);
			space.Step(dt);
		}
	}
}
