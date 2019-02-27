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

        static float k_scalar_body(cpBody body, cpVect point, cpVect n)
        {
            float rcn = cpVect.cpvcross(cpVect.cpvsub(point, body.GetPosition()), n);
            return 1.0f / body.GetMass() + rcn * rcn / body.GetMoment();
        }

        public bool waterPreSolve(cpArbiter arb, cpSpace space, object o)
        {
            cpShape obj1, obj2;
            arb.GetShapes(out obj1, out obj2);
            cpPolyShape water = obj1 as cpPolyShape;
            cpPolyShape poly = obj2 as cpPolyShape;
            cpBody body = poly.GetBody();

            float level = water.GetBB().t;// cpShapeGetBB().t;

            int count = poly.Count; //cpPolyShapeGetCount(poly.g);
            int clippedCount = 0;

            cpVect[] clipped = new cpVect[10];

            for (int i = 0, j = count - 1; i < count; j = i, i++)
            {
                cpVect a = body.LocalToWorld(poly.GetVert(j));
                cpVect b = body.LocalToWorld(poly.GetVert(i));

                if (a.y < level)
                {
                    clipped[clippedCount] = a;
                    clippedCount++;
                }

                float a_level = a.y - level;
                float b_level = b.y - level;

                if (a_level * b_level < 0.0f)
                {
                    float t = cp.cpfabs(a_level) / (cp.cpfabs(a_level) + cp.cpfabs(b_level));

                    clipped[clippedCount] = cpVect.cpvlerp(a, b, t);
                    clippedCount++;
                }
            }

            // Calculate buoyancy from the clipped polygon area
            float clippedArea = cp.AreaForPoly(clippedCount, clipped, 0.0f);
            float displacedMass = clippedArea * FLUID_DENSITY;
            cpVect centroid = cp.CentroidForPoly(clippedCount, clipped);

            //ChipmunkDebugDrawPolygon(clippedCount, clipped, 0.0f, RGBAColor(0, 0, 1, 1), RGBAColor(0, 0, 1, 0.1f));
            //ChipmunkDebugDrawDot(5, centroid, RGBAColor(0, 0, 1, 1));

            float dt = space.GetCurrentTimeStep();
            cpVect g = space.GetGravity();

            // Apply the buoyancy force as an impulse.
            body.ApplyImpulseAtWorldPoint(cpVect.cpvmult(g, -displacedMass * dt), centroid);

            // Apply linear damping for the fluid drag.
            cpVect v_centroid = body.GetVelocityAtWorldPoint(centroid);
            float k = k_scalar_body(body, centroid, cpVect.cpvnormalize(v_centroid));
            float damping = clippedArea * FLUID_DRAG * FLUID_DENSITY;
            float v_coef = cp.cpfexp(-damping * dt * k); // linear drag
            //	cpfloat v_coef = 1.0/(1.0 + damping*dt*cpvlength(v_centroid)*k); // quadratic drag
            body.ApplyImpulseAtWorldPoint(cpVect.cpvmult(cpVect.cpvsub(cpVect.cpvmult(v_centroid, v_coef), v_centroid), 1.0f / k), centroid);

            // Apply angular damping for the fluid drag.
            cpVect cog = body.LocalToWorld(body.GetCenterOfGravity());
            float w_damping = cp.MomentForPoly(FLUID_DRAG * FLUID_DENSITY * clippedArea, clippedCount, clipped, cpVect.cpvneg(cog), 0.0f);
            body.SetAngularVelocity(body.GetAngularVelocity() * cp.cpfexp(-w_damping * dt / body.GetMoment()));
            return true;
        }

        public override void OnEnter()
        {
            base.OnEnter();
            space.SetIterations(30);
            space.SetGravity(new cpVect(0, -500));
            space.SetSleepTimeThreshold(0.5f);
            space.SetCollisionSlop(0.5f);
            var staticBody = space.GetStaticBody();

            // Create segments around the edge of the screen.
            var shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(-320, 240), 0.0f));
            shape.SetElasticity(1.0f);
            shape.SetFriction(1.0f);
            shape.SetFilter(NOT_GRABBABLE_FILTER);

            shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(320, -240), new cpVect(320, 240), 0.0f));
            shape.SetElasticity(1.0f);
            shape.SetFriction(1.0f);
            shape.SetFilter(NOT_GRABBABLE_FILTER);

            shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(320, -240), 0.0f));
            shape.SetElasticity(1.0f);
            shape.SetFriction(1.0f);
            shape.SetFilter(NOT_GRABBABLE_FILTER);

            shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, 240), new cpVect(320, 240), 0.0f));
            shape.SetElasticity(1.0f);
            shape.SetFriction(1.0f);
            shape.SetFilter(NOT_GRABBABLE_FILTER);

            {
                // Add the edges of the bucket
                var bb = new cpBB(-300, -200, 100, 0);
                var radius = 5.0f;

                shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(bb.l, bb.b), new cpVect(bb.l, bb.t), radius));
                shape.SetElasticity(1.0f);
                shape.SetFriction(1.0f);
                shape.SetFilter(NOT_GRABBABLE_FILTER);

                shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(bb.r, bb.b), new cpVect(bb.r, bb.t), radius));
                shape.SetElasticity(1.0f);
                shape.SetFriction(1.0f);
                shape.SetFilter(NOT_GRABBABLE_FILTER);

                shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(bb.l, bb.b), new cpVect(bb.r, bb.b), radius));
                shape.SetElasticity(1.0f);
                shape.SetFriction(1.0f);
                shape.SetFilter(NOT_GRABBABLE_FILTER);

                // Add the sensor for the water.
                shape = space.AddShape(cpPolyShape.BoxShape2(staticBody, bb, 0.0f));
                shape.SetSensor(true);
                shape.SetCollisionType(1);
            }
            {
                float width = 200.0f;
                float height = 50.0f;
                float mass = 0.3f * FLUID_DENSITY * width * height;
                var moment = cp.MomentForBox(mass, width, height);

                cpBody body = space.AddBody(new cpBody(mass, moment));
                body.SetPosition(new cpVect(-50, -100));
                body.SetVelocity(new cpVect(0, -100));
                body.SetAngularVelocity(1);

                shape = space.AddShape(cpPolyShape.BoxShape(body, width, height, 0.0f));
                shape.SetFriction(0.8f);
            }

            {
                float width = 40.0f;
                float height = width * 2;
                float mass = 0.3f * FLUID_DENSITY * width * height;
                float moment = cp.MomentForBox(mass, width, height);

                cpBody body = space.AddBody(new cpBody(mass, moment));
                body.SetPosition(new cpVect(-200, -50));
                body.SetVelocity(new cpVect(0, -100));
                body.SetAngularVelocity(1);

                shape = space.AddShape(cpPolyShape.BoxShape(body, width, height, 0.0f));
                shape.SetFriction(0.8f);
            }

            cpCollisionHandler handler = space.AddCollisionHandler(1, 0);
            handler.preSolveFunc = waterPreSolve;
            Schedule();
        }

        public override void Update(float dt)
        {
            //base.Update(dt);
            this.space.Step(dt);
        }
    }
}
