using ChipmunkSharp;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace ChipmunkExample
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class BallTest : CSBaseLayer
    {

        #region Initialization
        public BallTest()
        {
        }

        protected override void PostLoadContent()
        {
            base.PostLoadContent();

            space.iterations = 60;
            space.gravity = new cpVect(0, -500);
            space.sleepTimeThreshold = 0.5f;
            space.collisionSlop = 0.5f;
            space.sleepTimeThreshold = 0.5f;

            this.addFloor();
            this.addWalls();

            float width = 50;
            float height = 60;
            float mass = width * height * 1 / 1000;

            cpBody body = new cpBody(mass, cp.momentForBox(mass, width, height));

            var rock = space.addBody(new cpBody(mass, cp.momentForBox(mass, width, height)));
            rock.Position = new cpVect(500, 100);
            rock.setAngle(1);

            cpPolyShape shape = space.addShape(cp.BoxShape(rock, width, height)) as cpPolyShape;

            shape.setFriction(0.3f);
            shape.setElasticity(0.3f);


            for (var i = 1; i <= 10; i++)
            {
                float radius = 20;
                mass = 3;

                body = space.addBody(new cpBody(mass, cp.momentForCircle(mass, 0, radius, cpVect.Zero)));

                body.Position = new cpVect(200 + i, (2 * radius + 5) * i);

                cpCircleShape circle = space.addShape(new cpCircleShape(body, radius, cpVect.Zero)) as cpCircleShape;
                circle.setElasticity(0.8f);
                circle.setFriction(1f);
            }

            var ramp = space.addShape(new cpSegmentShape(space.staticBody, new cpVect(100, 100), new cpVect(300, 200), 10));

            ramp.setElasticity(1f);
            ramp.setFriction(1f);

            ramp.layers = cp.NOT_GRABABLE_MASK;
        }


        #endregion

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
