using ChipmunkSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
    class buoyancyLayer : CSBaseLayer
    {

        float FLUID_DENSITY = 0.00014f;
        float FLUID_DRAG = 2.0f;
        protected override void PostLoadContent()
        {

            space.iterations = 30;
            space.gravity = new cpVect(0, -500);
            //	cpSpaceSetDamping(space, 0.5);
            space.sleepTimeThreshold = 0.5f;
            space.collisionSlop = 0.5f;

            var staticBody = space.staticBody;

            // Create segments around the edge of the screen.
            var shape = space.addShape(new cpSegmentShape(new cpBody(), new cpVect(0, 0), new cpVect(0f, 480f), 0.0f));
            shape.setElasticity(1.0f);
            shape.setFriction(1.0f);
            shape.setLayers(cp.NOT_GRABABLE_MASK);

            shape = space.addShape(new cpSegmentShape(new cpBody(), new cpVect(640f, 0), new cpVect(640, 480), 0.0f));
            shape.setElasticity(1.0f);
            shape.setFriction(1.0f);
            shape.setLayers(cp.NOT_GRABABLE_MASK);

            shape = space.addShape(new cpSegmentShape(new cpBody(), new cpVect(0, 0), new cpVect(640, 0), 0.0f));
            shape.setElasticity(1.0f);
            shape.setFriction(1.0f);
            shape.setLayers(cp.NOT_GRABABLE_MASK);

            shape = space.addShape(new cpSegmentShape(new cpBody(), new cpVect(0, 480), new cpVect(640, 480), 0.0f));
            shape.setElasticity(1.0f);
            shape.setFriction(1.0f);
            shape.setLayers(cp.NOT_GRABABLE_MASK);

            // {
            // Add the edges of the bucket
            var bb = new cpBB(20, 40, 420, 240);
            var radius = 5.0f;

            shape = space.addShape(new cpSegmentShape(new cpBody(), new cpVect(bb.l, bb.b), new cpVect(bb.l, bb.t), radius));
            shape.setElasticity(1.0f);
            shape.setFriction(1.0f);
            shape.setLayers(cp.NOT_GRABABLE_MASK);

            shape = space.addShape(new cpSegmentShape(new cpBody(), new cpVect(bb.r, bb.b), new cpVect(bb.r, bb.t), radius));
            shape.setElasticity(1.0f);
            shape.setFriction(1.0f);
            shape.setLayers(cp.NOT_GRABABLE_MASK);

            shape = space.addShape(new cpSegmentShape(new cpBody(), new cpVect(bb.l, bb.b), new cpVect(bb.r, bb.b), radius));
            shape.setElasticity(1.0f);
            shape.setFriction(1.0f);
            shape.setLayers(cp.NOT_GRABABLE_MASK);

            // Add the sensor for the water.
            shape = space.addShape(cp.BoxShape2(new cpBody(), bb));
            shape.setSensor(true);
            shape.setCollisionType("1");
            // }


            // {
            float width = 200.0f;
            float height = 50.0f;
            float mass = 0.3f * FLUID_DENSITY * width * height;
            var moment = cp.momentForBox(mass, width, height);

            cpBody body = space.addBody(new cpBody(mass, moment));
            body.setPos(new cpVect(270, 140));
            body.setVel(new cpVect(0, -100));
            body.setAngVel(1);

            shape = space.addShape(cp.BoxShape(body, width, height));
            shape.setFriction(0.8f);
            // }

            // {
            width = 40.0f;
            height = width * 2;
            mass = 0.3f * FLUID_DENSITY * width * height;
            moment = cp.momentForBox(mass, width, height);

            body = space.addBody(new cpBody(mass, moment));
            body.setPos(new cpVect(120, 190));
            body.setVel(new cpVect(0, -100));
            body.setAngVel(1);

            shape = space.addShape(cp.BoxShape(body, width, height));
            shape.setFriction(0.8f);
            // }

            space.addCollisionHandler("1", "0",
               null,
              waterPreSolve,
               null,
                null
                  );
        }

        public bool waterPreSolve(cpArbiter arb, cpSpace space)
        {
            var shapes = arb.getShapes();
            var water = shapes[0];
            var poly = shapes[1] as cpPolyShape;

            var body = poly.body;

            // Get the top of the water sensor bounding box to use as the water level.
            var level = water.getBB().t;

            // Clip the polygon against the water level
            var count = poly.getNumVerts();

            List<float> clipped = new List<float>();

            var j = count - 1;
            for (var i = 0; i < count; i++)
            {
                var a = body.local2World(poly.getVert(j));
                var b = body.local2World(poly.getVert(i));

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
            var centroid = cp.centroidForPoly(clippedArray);
            var r = centroid.Sub(body.getPos()); //  cp.v.sub(centroid, );

            var dt = space.getCurrentTimeStep();
            var g = space.gravity;

            // Apply the buoyancy force as an impulse.
            body.applyImpulse(cpVect.Multiply(g, -displacedMass * dt), r);

            // Apply linear damping for the fluid drag.
            var v_centroid = cpVect.Add(body.getVel(), cpVect.Multiply(cpVect.Perp(r), body.w));
            var k = 1; //k_scalar_body(body, r, cp.v.normalize_safe(v_centroid));
            float damping = clippedArea * FLUID_DRAG * FLUID_DENSITY;
            float v_coef = (float)Math.Exp(-damping * dt * k); // linear drag
            //	var v_coef = 1.0/(1.0 + damping*dt*cp.v.len(v_centroid)*k); // quadratic drag
            body.applyImpulse(cpVect.Multiply(cpVect.Sub(cpVect.Multiply(v_centroid, v_coef), v_centroid), 1.0f / k), r);

            // Apply angular damping for the fluid drag.
            float w_damping = cp.momentForPoly(FLUID_DRAG * FLUID_DENSITY * clippedArea, clippedArray, body.Position.Neg());
            body.w *= (float)Math.Exp(-w_damping * dt * (1 / body.i));

            return true;
        }


        #region Update and Draw

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(float dt)
        {
            base.Update(dt);
            space.step(dt);
        }

        protected override void Draw()
        {
            base.Draw();
            space.DrawDebugData();
        }

        #endregion
    }
}
