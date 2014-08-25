using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class sliceLayer : ChipmunkDemoLayer
	{

		float DENSITY = (1f / 10000f);


		public void ClipPoly(cpSpace space, cpShape shp, cpVect n, float dist)
		{

			cpPolyShape shape = (cpPolyShape)shp;

			cpBody body = shape.GetBody();


			int count = shape.Count;
			int clippedCount = 0;

			cpVect[] clipped = new cpVect[count + 1];

			for (int i = 0, j = count - 1; i < count; j = i, i++)
			{
				cpVect a = body.LocalToWorld(shape.GetVert(j));
				float a_dist = cpVect.cpvdot(a, n) - dist;

				if (a_dist < 0)
				{
					clipped[clippedCount] = a;
					clippedCount++;
				}

				cpVect b = body.LocalToWorld(shape.GetVert(i));
				float b_dist = cpVect.cpvdot(b, n) - dist;

				if (a_dist * b_dist < 0)
				{
					float t = cp.cpfabs(a_dist) / (cp.cpfabs(a_dist) + cp.cpfabs(b_dist));

					clipped[clippedCount] = cpVect.cpvlerp(a, b, t);
					clippedCount++;
				}
			}

			cpVect centroid = cp.CentroidForPoly(clippedCount, clipped);
			float mass = cp.AreaForPoly(clippedCount, clipped, 0) * DENSITY;
			float moment = cp.MomentForPoly(mass, clippedCount, clipped, cpVect.cpvneg(centroid), 0);

			cpBody new_body = space.AddBody(new cpBody(mass, moment));
			new_body.SetPosition(centroid);
			new_body.SetVelocity(body.GetVelocityAtWorldPoint(centroid));
			new_body.SetAngularVelocity(body.GetAngularVelocity());

			cpTransform transform = cpTransform.Translate(cpVect.cpvneg(centroid));
			cpShape new_shape = space.AddShape(new cpPolyShape(new_body, clippedCount, clipped, transform, 0));
			// Copy whatever properties you have set on the original shape that are important
			new_shape.SetFriction(shape.GetFriction());
		}

		public void SliceShapePostStep(cpSpace space, cpShape shape, SliceContext context)
		{
			cpVect a = context.a;
			cpVect b = context.b;

			// Clipping plane normal and distance.
			cpVect n = cpVect.cpvnormalize(cpVect.cpvperp(cpVect.cpvsub(b, a)));
			float dist = cpVect.cpvdot(a, n);

			ClipPoly(space, shape, n, dist);
			ClipPoly(space, shape, cpVect.cpvneg(n), -dist);

			cpBody body = shape.GetBody();
			space.RemoveShape(shape);
			space.RemoveBody(body);
		}

		public void SliceQuery(cpShape shape, float t, cpVect n, SliceContext context)
		{
			cpVect a = context.a;
			cpVect b = context.b;

			// Check that the slice was complete by checking that the endpoints aren't in the sliced shape.

			cpPointQueryInfo inf1 = null;
			cpPointQueryInfo inf2 = null;

			if (shape.PointQuery(a, ref inf1) > 0 && shape.PointQuery(b, ref inf2) > 0)
			{
				// Can't modify the space during a query.
				// Must make a post-step callback to do the actual slicing.
				context.space.AddPostStepCallback(
					(s, o1, o2) => SliceShapePostStep(s, (cpShape)o1, (SliceContext)o2),
					shape, context);
			}
		}



		// Context structs are annoying, use blocks or closures instead if your compiler supports them.
		public struct SliceContext
		{
			public cpVect a, b;
			public cpSpace space;


			public SliceContext(cpVect a, cpVect b, cpSpace space)
			{
				// TODO: Complete member initialization
				this.a = a;
				this.b = b;
				this.space = space;
			}
		};


		public override void OnEnter()
		{
			base.OnEnter();

			SetSubTitle("Right click and drag to slice up the block.");


			space.SetIterations(30);
			space.SetGravity(new cpVect(0, -500));
			space.SetSleepTimeThreshold(0.5f);
			space.SetCollisionSlop(0.5f);

			cpBody body, staticBody = space.GetStaticBody();
			cpShape shape;

			// Create segments around the edge of the screen.
			shape = space.AddShape(new cpSegmentShape(staticBody,
				new cpVect(-1000, -240), new cpVect(1000, -240), 0));

			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetFilter(NOT_GRABBABLE_FILTER);

			float width = 200;
			float height = 300;
			float mass = width * height * DENSITY;
			float moment = cp.MomentForBox(mass, width, height);

			body = space.AddBody(new cpBody(mass, moment));

			shape = space.AddShape(cpPolyShape.BoxShape(body, width, height, 0));
			shape.SetFriction(0.6f);

			Schedule();
		}

		bool write;
		cpVect sliceStart;

		public override void Update(float dt)
		{
			base.Update(dt);

			space.Step(dt);


			bool lastClickState = false;
			sliceStart = cpVect.Zero;

			// Annoying state tracking code that you wouldn't need
			// in a real event driven system.
			if (CCMouse.Instance.rightclick != lastClickState)
			{
				if (CCMouse.Instance.rightclick)
				{
					// MouseDown
					sliceStart = CCMouse.Instance.Position;
				}
				else
				{
					// MouseUp
					SliceContext context = new SliceContext(sliceStart, CCMouse.Instance.Position, space);

					space.SegmentQuery(sliceStart,
						CCMouse.Instance.Position, 0, GRAB_FILTER, (shape, v1, v2, d, o) =>
						SliceQuery(shape, d, v1, (SliceContext)o),
						context);
				}

				lastClickState = CCMouse.Instance.rightclick;
			}

			write = CCMouse.Instance.rightclick;



		}




		protected override void Draw()
		{
			base.Draw();

			if (write)
			{
				m_debugDraw.DrawSegment(sliceStart, CCMouse.Instance.Position, 1, cpColor.Red);
			}

		}


	}
}
