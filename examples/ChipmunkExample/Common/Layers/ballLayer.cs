using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
    class ballLayer : ChipmunkDemoLayer
    {
        protected override void AddedToScene()
        {
            base.AddedToScene();

            //PositionX += (windowSize.Width - 640) * .5d;  //new CCPoint(150, 150);
            space.SetIterations(60);
            space.SetGravity(new cpVect(0, -500));
            space.SetSleepTimeThreshold(0.5f);
            space.SetCollisionSlop(0.5f);
            space.SetSleepTimeThreshold(0.5f);

            this.addFloor();
            this.addWalls();
            float width = 50;
            float height = 60;
            float mass = width * height * 1f / 1000f;

            var rock = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, width, height)));
            rock.SetPosition(new cpVect(200, 0));
            rock.SetAngle(1);

            cpPolyShape shape = space.AddShape(cpPolyShape.BoxShape(rock, width, height, 0.0f)) as cpPolyShape;
            shape.SetFriction(0.3f);
            shape.SetElasticity(0.3f);
            //shape.SetFilter(NOT_GRABBABLE_FILTER); //The box cannot be dragg

            for (var i = 1; i <= 6; i++)
            {
                float radius = 20f;
                mass = 3;
                var body = space.AddBody(new cpBody(mass, cp.MomentForCircle(mass, 0f, radius, cpVect.Zero)));
                body.SetPosition(new cpVect(i, (2 * radius + 5) * 1));

                cpCircleShape circle = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero)) as cpCircleShape;
                circle.SetElasticity(0.8f);
                circle.SetFriction(1);
            }

            var ramp = space.AddShape(new cpSegmentShape(space.GetStaticBody(), new cpVect(0, 0), new cpVect(300, 200), 10));
            ramp.SetElasticity(1f);
            ramp.SetFriction(1f);
            ramp.SetFilter(NOT_GRABBABLE_FILTER);

            Schedule();
        }

        public override void Update(float dt)
        {
            base.Update(dt);
            space.Step(dt);
        }
    }
}
