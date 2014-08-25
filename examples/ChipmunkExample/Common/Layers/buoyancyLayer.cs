using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class buoyancyLayer : ChipmunkDemoLayer
	{

		float FLUID_DENSITY = 0.00014f;
		float FLUID_DRAG = 2.0f;

		protected override void AddedToScene()
		{
			base.AddedToScene();

			space.iterations = 30;
			space.gravity = new cpVect(0, -500);
			//	cpSpaceSetDamping(space, 0.5);
			space.sleepTimeThreshold = 0.5f;
			space.collisionSlop = 0.5f;

			var staticBody = space.StaticBody;

			// Create segments around the edge of the screen.
			var shape = space.AddShape(new cpSegmentShape(new cpBody(), new cpVect(0, 0), new cpVect(0f, 480f), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetGroup(cp.NOT_GRABABLE_MASK);

			shape = space.AddShape(new cpSegmentShape(new cpBody(), new cpVect(640f, 0), new cpVect(640, 480), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetGroup(cp.NOT_GRABABLE_MASK);

			shape = space.AddShape(new cpSegmentShape(new cpBody(), new cpVect(0, 0), new cpVect(640, 0), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetGroup(cp.NOT_GRABABLE_MASK);

			shape = space.AddShape(new cpSegmentShape(new cpBody(), new cpVect(0, 480), new cpVect(640, 480), 0.0f));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetGroup(cp.NOT_GRABABLE_MASK);

			// {
			// Add the edges of the bucket
			var bb = new cpBB(20, 40, 420, 240);
			var radius = 5.0f;

			shape = space.AddShape(new cpSegmentShape(new cpBody(), new cpVect(bb.l, bb.b), new cpVect(bb.l, bb.t), radius));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetGroup(cp.NOT_GRABABLE_MASK);

			shape = space.AddShape(new cpSegmentShape(new cpBody(), new cpVect(bb.r, bb.b), new cpVect(bb.r, bb.t), radius));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetGroup(cp.NOT_GRABABLE_MASK);

			shape = space.AddShape(new cpSegmentShape(new cpBody(), new cpVect(bb.l, bb.b), new cpVect(bb.r, bb.b), radius));
			shape.SetElasticity(1.0f);
			shape.SetFriction(1.0f);
			shape.SetGroup(cp.NOT_GRABABLE_MASK);

			// Add the sensor for the water.
			shape = space.AddShape(cpPolyShape.BoxShape2(new cpBody(), bb, 0.0f));
			shape.SetSensor(true);
			shape.SetCollisionType("1");
			// }


			// {
			float width = 200.0f;
			float height = 50.0f;
			float mass = 0.3f * FLUID_DENSITY * width * height;
			var moment = cp.MomentForBox(mass, width, height);

			cpBody body = space.AddBody(new cpBody(mass, moment));
			body.SetPosition(new cpVect(270, 140));
			body.SetVelocity(new cpVect(0, -100));
			body.SetAngularVelocity(1);

			shape = space.AddShape(cpPolyShape.BoxShape(body, width, height,0.0f));
			shape.SetFriction(0.8f);
			// }

			// {
			width = 40.0f;
			height = width * 2;
			mass = 0.3f * FLUID_DENSITY * width * height;
			moment = cp.MomentForBox(mass, width, height);

			body = space.AddBody(new cpBody(mass, moment));
			body.SetPosition(new cpVect(120, 190));
			body.SetVelocity(new cpVect(0, -100));
			body.SetAngularVelocity(1);

			shape = space.AddShape(cpPolyShape.BoxShape(body, width, height,0.0f));
			shape.SetFriction(0.8f);
			// }

			space.AddCollisionHandler("1", "0",
			   null,(a,s,o) => waterPreSolve(a,s) ,
			   null,
				null
				  );

			Schedule();

		}

		public override void OnEnter()
		{
			base.OnEnter();
			//Position = new CCPoint((640 - Director.WindowSizeInPixels.Width) * .5f, (480 - Director.WindowSizeInPixels.Height) * .5f);
		}

		public bool waterPreSolve(cpArbiter arb, cpSpace space)
		{
			var shapes = arb.GetShapes();
			var water = shapes[0];
			var poly = shapes[1] as cpPolyShape;

			var body = poly.body;

			// Get the top of the water sensor bounding box to use as the water level.
			var level = water.bb.t;

			// Clip the polygon against the water level
			var count = poly.GetNumVerts();

			List<float> clipped = new List<float>();

			var j = count - 1;
			for (var i = 0; i < count; i++)
			{
				var a = body.LocalToWorld(poly.GetVert(j));
				var b = body.LocalToWorld(poly.GetVert(i));

				if (a.y < level)
				{
					clipped.Add(a.x);
					clipped.Add(a.y);
				}

				var a_level = a.y - level;
				var b_level = b.y - level;

				if (a_level * b_level < 0.0)
				{
					var t = Math.Abs(a_level) / (Math.Abs(a_level) + Math.Abs(b_level));

					var v = cpVect.vlerp(a, b, t);
					clipped.Add(v.x);
					clipped.Add(v.y);
				}
				j = i;
			}


			var clippedArray = clipped.ToArray();

			// Calculate buoyancy from the clipped polygon area
			var clippedArea = cp.areaForPoly(clippedArray);

			var displacedMass = clippedArea * FLUID_DENSITY;
			var centroid = cp.CentroidForPoly(clippedArray);
			var r = centroid.Sub(body.GetPos()); //  cp.v.sub(centroid, );

			var dt = space.GetCurrentTimeStep();
			var g = space.gravity;

			// Apply the buoyancy force as an impulse.
			body.ApplyImpulse(cpVect.Multiply(g, -displacedMass * dt), r);

			// Apply linear damping for the fluid drag.
			var v_centroid = cpVect.Add(body.GetVel(), cpVect.Multiply(cpVect.Perp(r), body.w));
			var k = 1; //k_scalar_body(body, r, cp.v.normalize_safe(v_centroid));
			float damping = clippedArea * FLUID_DRAG * FLUID_DENSITY;
			float v_coef = (float)Math.Exp(-damping * dt * k); // linear drag
			//	var v_coef = 1.0/(1.0 + damping*dt*cp.v.len(v_centroid)*k); // quadratic drag
			body.ApplyImpulse(cpVect.Multiply(cpVect.Sub(cpVect.Multiply(v_centroid, v_coef), v_centroid), 1.0f / k), r);

			// Apply angular damping for the fluid drag.
			float w_damping = cp.momentForPoly(FLUID_DRAG * FLUID_DENSITY * clippedArea, clippedArray, body.Position.Neg());
			body.w *= (float)Math.Exp(-w_damping * dt * (1 / body.i));

			return true;
		}

		public override void Update(float dt)
		{
			base.Update(dt);
			var steps = 1;
			dt = dt / steps;

			for (var i = 0; i < steps; i++)
			{
				this.space.Step(dt);
			}
		}



	}
}
