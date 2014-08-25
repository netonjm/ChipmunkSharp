using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ChipmunkSharp;
using Microsoft.Xna.Framework.Input;
namespace ChipmunkExample
{
	class convexLayer : ChipmunkDemoLayer
	{

		float DENSITY = 1f / 10000f;

		cpPolyShape shape;

		protected override void AddedToScene()
		{
			base.AddedToScene();

			Position = new CCPoint(50, 20);

			space.iterations = 30;
			space.gravity = new cpVect(0, -500);
			space.sleepTimeThreshold = 0.5f;
			space.collisionSlop = 0.5f;

			cpBody body, staticBody = space.StaticBody;

			// Create segments around the edge of the screen.
			this.addFloor();

			var width = 50;
			var height = 70;
			var mass = width * height * DENSITY;
			var moment = cp.MomentForBox(mass, width, height);

			body = space.AddBody(new cpBody(mass, moment));
			body.SetPosition(new cpVect(320, height / 2));

			this.shape = space.AddShape(cpPolyShape.BoxShape(body, width, height, 0.0f)) as cpPolyShape;
			this.shape.SetFriction(0.6f);

			Schedule();

		}

		public override void Update(float dt)
		{
			base.Update(dt);

			var tolerance = 2;

			//var mouse = new cpVect(mouseState.X, mouseState.Y);

			if (!CCMouse.Instance.HasPosition)
				return;

			var info = this.shape.NearestPointQuery(CCMouse.Instance.Position);

			if (CCMouse.Instance.rightclick && info.distance > tolerance)
			{
				var body = shape.body;
				var count = shape.GetNumVerts();

				// Allocate the space for the new vertexes on the stack.
				float[] verts = new float[(count + 1) * 2];

				for (var i = 0; i < count; i++)
				{
					verts[i * 2] = shape.verts[i * 2];
					verts[i * 2 + 1] = shape.verts[i * 2 + 1];
				}

				var end = body.WorldToLocal(CCMouse.Instance.Position);
				verts[count * 2] = end.x;
				verts[count * 2 + 1] = end.y;

				// This function builds a convex hull for the vertexes.
				// Because the result array is NULL, it will reduce the input array instead.
				cp.convexHull(verts, null, tolerance);

				// Figure out how much to shift the body by.
				var centroid = cp.CentroidForPoly(verts);

				// Recalculate the body properties to match the updated shape.
				var mass = cp.areaForPoly(verts) * DENSITY;
				body.SetMass(mass);
				body.SetMoment(cp.momentForPoly(mass, verts, centroid.Neg()));
				body.SetPosition(body.LocalToWorld(centroid));

				// Use the setter function from chipmunk_unsafe.h.
				// You could also remove and recreate the shape if you wanted.
				shape.SetVerts(verts, centroid.Neg());
			}

			var steps = 1;
			dt = dt / steps;

			for (var i = 0; i < steps; i++)
			{
				this.space.Step(dt);
			}

		}

		protected override void Draw()
		{
			base.Draw();
		}



	}
}
