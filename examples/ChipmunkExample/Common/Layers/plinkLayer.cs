using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class plinkLayer : ChipmunkDemoLayer
	{


		int NUM_VERTS = 5;
		float pentagon_mass = 0.0f;
		float pentagon_moment = 0.0f;


		public override void OnEnter()
		{
			base.OnEnter();


			SetSubTitle("Right click to make pentagons static/dynamic.");
			space.SetIterations(5);
			space.SetGravity(new cpVect(0, -100));

			cpBody body, staticBody = space.GetStaticBody();
			cpShape shape;

			// Vertexes for a triangle shape.
			cpVect[] tris = new cpVect[] {
				new cpVect(-15,-15),
				new cpVect( 0,10),
				new cpVect(  15, -15),
				
			};


			// Create the static triangles.
			for (int i = 0; i < 9; i++)
			{
				for (int j = 0; j < 6; j++)
				{
					float stagger = (j % 2) * 40;
					cpVect offset = new cpVect(i * 80 - 320 + stagger, j * 70 - 240);

					shape = space.AddShape(new cpPolyShape(staticBody, 3, tris, cpTransform.Translate(offset), 0));

					shape.SetElasticity(1.0f);
					shape.SetFriction(1.0f);
					shape.SetFilter(NOT_GRABBABLE_FILTER);
				}
			}



			cpVect[] verts = new cpVect[NUM_VERTS];
			for (int i = 0; i < NUM_VERTS; i++)
			{
				float angle = -2 * cp.M_PI * i / NUM_VERTS;
				verts[i] = cpVect.cpv(10 * cp.cpfcos(angle), 10 * cp.cpfsin(angle));
			}

			pentagon_mass = 1;
			pentagon_moment = cp.MomentForPoly(1.0f, NUM_VERTS, verts, cpVect.Zero, 0.0f);

			// Add lots of pentagons.
			for (int i = 0; i < 100; i++)
			{
				body = space.AddBody(new cpBody(pentagon_mass, pentagon_moment));
				body.SetPosition(
					new cpVect(
						RandomHelper.next(-300, 300),
						RandomHelper.next(350, 1000)));

				shape = space.AddShape(new cpPolyShape(body, NUM_VERTS, verts, cpTransform.Identity, 0.0f));
				shape.SetElasticity(0.0f);
				shape.SetFriction(0.4f);
			}

			Schedule();

		}

		void eachBody(cpBody body)
		{
			cpVect pos = body.GetPosition();
			if (pos.y < -400 || cp.cpfabs(pos.x) > 340)
			{

				body.SetPosition(new cpVect(
						RandomHelper.next(-300, 300),
						RandomHelper.next(350, 900)));
			}
		}

		public override void OnTouchesEnded(List<CCTouch> touches, CCEvent arg2)
		{
			base.OnTouchesEnded(touches, arg2);
		}

		public override void Update(float dt)
		{
			base.Update(dt);

			if (CCMouse.Instance.rightclick)
			{
				cpPointQueryInfo info = null;
				cpShape nearest = space.PointQueryNearest(CCMouse.Instance.Position, 0.0f, GRAB_FILTER, ref info);
				if (nearest != null)
				{
					cpBody body = nearest.GetBody();// cpShapeGetBody();
					if (body.bodyType == cpBodyType.STATIC)
					{
						body.SetBodyType(cpBodyType.DYNAMIC);
						body.SetMass(pentagon_mass);
						body.SetMoment(pentagon_moment);
					}
					else if (body.bodyType == cpBodyType.DYNAMIC)
					{
						body.SetBodyType(cpBodyType.STATIC);

					}
				}
			}



			space.EachBody(eachBody, null);

			space.Step(dt);
		}


	}
}
