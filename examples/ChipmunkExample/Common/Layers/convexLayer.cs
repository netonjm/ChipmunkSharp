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
        cpShape shape;

        public override void OnEnter()
        {
            base.OnEnter();
            space.SetIterations(30);
            space.SetGravity(new cpVect(0, -500));
            space.SetSleepTimeThreshold(0.5f);
            space.SetCollisionSlop(0.5f);

            cpBody body, staticBody = space.GetStaticBody();
            // Create segments around the edge of the screen.
            //this.addFloor();

            this.shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(320, -240), 0f)) as cpSegmentShape;
            this.shape.SetFriction(1.0f);
            this.shape.SetElasticity(0.6f);
            this.shape.SetFilter(NOT_GRABBABLE_FILTER);
            var width = 50;
            var height = 70;
            var mass = width * height * DENSITY;
            var moment = cp.MomentForBox(mass, width, height);

            body = space.AddBody(new cpBody(mass, moment));
            body.SetPosition(new cpVect(0, 0));

            this.shape = space.AddShape(cpPolyShape.BoxShape(body, width, height, 0.0f));
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
            cpPointQueryInfo info = null;
            //CCMouse.Instance.UpdatePosition()
            var d = this.shape.PointQuery(CCMouse.Instance.Position, ref info);
            var actualShape = (shape as cpPolyShape);

            if ((CCMouse.Instance.rightclick || CCMouse.Instance.dblclick) && d > tolerance)
            {
                var body = actualShape.body;
                var count = actualShape.Count;

                // Allocate the space for the new vertexes on the stack.
                cpVect[] verts = new cpVect[count + 1]; // * sizeof(cpVect));
                for (int i = 0; i < count; i++)
                    verts[i] = actualShape.GetVert(i);
                verts[count] = body.WorldToLocal(CCMouse.Instance.Position);

                // This function builds a convex hull for the vertexes.
                // Because the result array is NULL, it will reduce the input array instead.
                int hullCount = cp.ConvexHull(count + 1, verts, ref verts, null, tolerance);

                // Figure out how much to shift the body by.
                var centroid = cp.CentroidForPoly(hullCount, verts);

                // Recalculate the body properties to match the updated shape.
                var mass = cp.AreaForPoly(hullCount, verts, 0.0f) * DENSITY;
                body.SetMass(mass);
                body.SetMoment(cp.MomentForPoly(mass, hullCount, verts, cpVect.cpvneg(centroid), 0.0f));
                body.SetPosition(body.LocalToWorld(centroid));

                // Use the setter function from chipmunk_unsafe.h.
                // You could also remove and recreate the shape if you wanted.
                actualShape.SetVerts(hullCount, verts,
                    cpTransform.Translate(cpVect.cpvneg(centroid)));
            }

            var steps = 1;
            dt = dt / steps;

            for (var i = 0; i < steps; i++)
                this.space.Step(dt);
        }
    }
}
