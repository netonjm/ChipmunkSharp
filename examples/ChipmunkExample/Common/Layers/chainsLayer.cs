using ChipmunkSharp;
using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
    class chainsLayer : ChipmunkDemoLayer
    {
        int CHAIN_COUNT = 8;
        int LINK_COUNT = 10;

        void BreakablejointPostStepRemove(cpConstraint joint)
        {
            space.RemoveConstraint(joint);
        }

        void BreakableJointPostSolve(cpConstraint joint)
        {
            float dt = space.GetCurrentTimeStep();

            // Convert the impulse to a force by dividing it by the timestep.
            float force = joint.GetImpulse() / dt;
            float maxForce = joint.GetMaxForce();// maxForce;

            // If the force is almost as big as the joint's max force, break it.
            if (force > 0.9 * maxForce)
            {
                space.AddPostStepCallback(
                    (s, o1, o2) => BreakablejointPostStepRemove(o1 as cpConstraint),
                    joint, null
                    );
            }
        }

        public override void OnEnter()
        {
            base.OnEnter();
            space.SetIterations(30);
            space.SetGravity(new cpVect(0, -100));
            space.SetSleepTimeThreshold(0.5f);

            cpBody body, staticBody = space.GetStaticBody();
            cpShape shape;

            // Create segments around the edge of the screen.
            shape = space.AddShape(new cpSegmentShape(staticBody, new cpVect(-320, -240), new cpVect(-320, 240), 0.0f));
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

            float mass = 1;
            float width = 20;
            float height = 30;
            float spacing = width * 0.3f;

            // Add lots of boxes.
            for (int i = 0; i < CHAIN_COUNT; i++)
            {
                cpBody prev = null;

                for (int j = 0; j < LINK_COUNT; j++)
                {
                    cpVect pos = new cpVect(40 * (i - (CHAIN_COUNT - 1) / 2.0f), 240 - (j + 0.5f) * height - (j + 1) * spacing);
                    body = space.AddBody(new cpBody(mass, cp.MomentForBox(mass, width, height)));
                    body.SetPosition(pos);

                    shape = space.AddShape(new cpSegmentShape(body, new cpVect(0, (height - width) / 2.0f), new cpVect(0, (width - height) / 2.0f), width / 2.0f));
                    shape.SetFriction(0.8f);

                    float breakingForce = 80000;

                    cpConstraint constraint = null;
                    if (prev == null)
                        constraint = space.AddConstraint(new cpSlideJoint(body, staticBody, new cpVect(0, height / 2), new cpVect(pos.x, 240), 0, spacing));
                    else
                        constraint = space.AddConstraint(new cpSlideJoint(body, prev, new cpVect(0, height / 2), new cpVect(0, -height / 2), 0, spacing));

                    constraint.SetMaxForce(breakingForce);
                    constraint.SetPostSolveFunc(s => BreakableJointPostSolve(constraint));
                    constraint.SetCollideBodies(false);

                    //cpConstraintSetPostSolveFunc(constraint, BreakableJointPostSolve);
                    prev = body;
                }
            }

            float radius = 15.0f;
            body = space.AddBody(new cpBody(10.0f, cp.MomentForCircle(10.0f, 0.0f, radius, cpVect.Zero)));
            body.SetPosition(new cpVect(0, -240 + radius + 5));
            body.SetVelocity(new cpVect(0, 300));

            shape = space.AddShape(new cpCircleShape(body, radius, cpVect.Zero));
            shape.SetElasticity(0.0f);
            shape.SetFriction(0.9f); ;

            Schedule();
        }

        public override void Update(float dt)
        {
            base.Update(dt);
            space.Step(dt);
        }
    }
}
