using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class gjkLayer : ChipmunkDemoLayer
	{

		cpShape shape1, shape2;

		public override void OnEnter()
		{
			base.OnEnter();


			space.SetIterations(5);
			space.SetDamping(0.1f);

			float mass = 1.0f;

			{
				float size = 100.0f;

				cpBody body = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, size, size)));
				body.SetPosition(new cpVect(100.0f, 50.0f));

				shape1 = space.AddShape(cpPolyShape.BoxShape(body, size, size, 0.0f));
				//shape1.SetGroup(1);
			}
			{
				float size = 100.0f;

				cpBody body = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, size, size)));
				body.SetPosition(new cpVect(120.0f, -40.0f));
				body.SetAngle(1e-2f);

				shape2 = space.AddShape(cpPolyShape.BoxShape(body, size, size, 0.0f));
				//shape2.SetGroup(1);
			}


			Schedule();

		}



		protected override void Draw()
		{
			base.Draw();

			//ChipmunkDemoDefaultDrawImpl(space);
			//ContactPoint[] arr = new ContactPoint[cpArbiter.CP_MAX_CONTACTS_PER_ARBITER];
			//	cpCollideShapes(shape1, shape2, (cpCollisionID[]){0}, arr);
			//cpCollisionInfo info = cpCollideShapes(shape2, shape1, 0x00000000, arr);
			List<cpContact> contacts = new List<cpContact>();
			cpCollision.cpCollide(shape1, shape2, 0, ref contacts);
			int collisions = contacts.Count;
			string description = "";


			for (int i = 0; i < collisions; i++)
			{
				description += contacts[0].ToString();
			}



			SetSubTitle(string.Format("{0} collitions: ({1})", collisions, description));
		}

		public override void Update(float dt)
		{
			base.Update(dt);

			space.Step(dt);
		}


	}
}
