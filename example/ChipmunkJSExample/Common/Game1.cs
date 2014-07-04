using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using ChipmunkSharp;
using System.Collections.Generic;
using System;

namespace ChipmunkExample
{
    /// <summary>
    /// This is the main type for your game
    /// </summary>
    public class Game1 : Game
    {
        #region Constants

        // this constant controls the number of stars that will be created when the game
        // starts up.
        const int NumStars = 500;

        // what percentage of those stars will be "big" stars? the default is 20%.
        const float PercentBigStars = .2f;

        // how bright will stars be?  somewhere between these two values.
        const byte MinimumStarBrightness = 56;
        const byte MaximumStarBrightness = 255;

        // how big is the ship?
        const float ShipSizeX = 10f;
        const float ShipSizeY = 15f;
        const float ShipCutoutSize = 5f;

        // the radius of the sun.
        const float SunSize = 30f;

        #endregion

        #region Fields

        GraphicsDeviceManager graphics;

        // PrimitiveBatch is the new class introduced in this sample. We'll use it to
        // draw everything in this sample, including the stars, ships, and sun.
        PrimitiveBatch m_debugDraw;

        // these two lists, stars, and starColors, keep track of the positions and
        // colors of all the stars that we will randomly generate during the initialize
        // phase.

        cpSpace space;
        int GRABABLE_MASK_BIT = 1;
        int NOT_GRABABLE_MASK = 0;

        #endregion

        #region Initialization
        public Game1()
        {

            this.IsMouseVisible = true;

            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";

#if WINDOWS_PHONE || IPHONE
            TargetElapsedTime = TimeSpan.FromTicks(333333);

            graphics.PreferredBackBufferWidth = 480;
            graphics.PreferredBackBufferHeight = 800;
            graphics.IsFullScreen = true;
#else
            // set the backbuffer size to something that will work well on both xbox
            // and windows.
            graphics.PreferredBackBufferWidth = 853;
            graphics.PreferredBackBufferHeight = 480;
#endif
        }

        protected override void Initialize()
        {
            base.Initialize();
        }


        /// <summary>
        /// Load your graphics content.
        /// </summary>
        protected override void LoadContent()
        {
            m_debugDraw = new PrimitiveBatch(graphics.GraphicsDevice);

            m_debugDraw.AppendFlags(ChipmunkDrawFlags.e_shapeBit | ChipmunkDrawFlags.e_aabbBit | ChipmunkDrawFlags.e_centerOfMassBit | ChipmunkDrawFlags.e_jointBit | ChipmunkDrawFlags.e_pairBit);

            m_debugDraw.DrawColor = cpColor.Blue;

            space = new cpSpace();
            space.SetDebugDraw(m_debugDraw);
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

            cpBody body = new cpBody(mass, cpEnvironment.momentForBox(mass, width, height));
            var rock = space.AddBody(body);

            rock.Position = new cpVect(500, 100);
            rock.SetAngle(1);

            cpShape shape = space.AddShape(cpPolyShape.cpBoxShapeNew(rock, width, height));

            shape.SetFriction(0.3f);
            shape.SetElasticity(0.3f);

            for (var i = 1; i <= 10; i++)
            {
                var radius = 20;
                mass = 3;

                body = new cpBody(mass, cpEnvironment.momentForCircle(mass, 0, radius, cpVect.ZERO));
                space.AddBody(body);

                body.Position = new cpVect(200 + i, (2 * radius + 5) * i);
                var circle = space.AddShape(cpCircleShape.cpCircleShapeNew(body, radius, cpVect.ZERO));
                circle.SetElasticity(0.8f);
                circle.SetFriction(1f);
            }

            var ramp = space.AddShape(cpSegmentShape.cpSegmentShapeNew(space.staticBody, new cpVect(100, 100), new cpVect(300, 200), 10));

            ramp.SetElasticity(1f);
            ramp.SetFriction(1f);

            ramp.layers = NOT_GRABABLE_MASK;

        }

        public void addFloor()
        {
            var fl = cpSegmentShape.cpSegmentShapeNew(space.staticBody, cpVect.ZERO, new cpVect(640, 0), 0);
            var floor = space.AddShape(fl);
            floor.SetElasticity(1);
            floor.SetFriction(1);
            floor.layers = NOT_GRABABLE_MASK;
        }

        public void addWalls()
        {
            var wall1 = space.AddShape(cpSegmentShape.cpSegmentShapeNew(space.staticBody, cpVect.ZERO, new cpVect(0, 480), 0));
            wall1.SetElasticity(1);
            wall1.SetFriction(1);
            wall1.layers = NOT_GRABABLE_MASK;

            var wall2 = space.AddShape(cpSegmentShape.cpSegmentShapeNew(space.staticBody, new cpVect(640, 0), new cpVect(640, 480), 0));
            wall2.SetElasticity(1);
            wall2.SetFriction(1);
            wall2.layers = NOT_GRABABLE_MASK;
        }


        #endregion

        #region Update and Draw

        List<Vector2> pointsClicked = new List<Vector2>();
        public bool pressed;

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {

            base.Update(gameTime);

            var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

            space.Step(dt);

            // Allows the game to exit
            if ((GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                || Keyboard.GetState().IsKeyDown(Keys.Escape))
                this.Exit();


            var mouse = Mouse.GetState();
            if (mouse.LeftButton == ButtonState.Pressed)
            {
                if (!pressed)
                {
                    pressed = true;
                    lock (pointsClicked)
                        pointsClicked.Add(new Vector2(mouse.X, mouse.Y));
                }
            }

            if (mouse.LeftButton == ButtonState.Released)
            {
                pressed = false;
            }

        }

        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            base.Draw(gameTime);
            graphics.GraphicsDevice.Clear(Color.Black);

            int screenWidth = graphics.GraphicsDevice.Viewport.Width;
            int screenHeight = graphics.GraphicsDevice.Viewport.Height;

            m_debugDraw.Begin(PrimitiveType.LineList);
            space.DrawDebugData();

            // and we're done.
            m_debugDraw.End();

          
        }


        public List<Vector2> GetSafePointList(List<Vector2> points)
        {
            List<Vector2> safePointsClicked = new List<Vector2>();
            lock (points)
            {
                foreach (var item in points)
                    safePointsClicked.Add(item);
            }

            return safePointsClicked;
        }

        #endregion

    }
}
// DrawStars is called to do exactly what its name says: draw the stars.
//private void DrawStars()
//{
//    // stars are drawn as a list of points, so begin the primitiveBatch.
//    primitiveBatch.Begin(PrimitiveType.TriangleList);

//    lock (stars)
//    {
//        // loop through all of the stars, and tell primitive batch to draw them.
//        // each star is a very small triangle.
//        for (int i = 0; i < stars.Count; i++)
//        {
//            primitiveBatch.AddVertex(stars[i], starColors[i]);
//            primitiveBatch.AddVertex(stars[i] + Vector2.UnitX, starColors[i]);
//            primitiveBatch.AddVertex(stars[i] + Vector2.UnitY, starColors[i]);
//        }
//    }

//private void CreateStars()
//{
//    // since every star will be put in a random place and have a random color, 
//    // a random number generator might come in handy.
//    Random random = new Random();

//    // where can we put the stars?
//    int screenWidth = graphics.GraphicsDevice.Viewport.Width;
//    int screenHeight = graphics.GraphicsDevice.Viewport.Height;


//    for (int i = 0; i < NumStars; i++)
//    {
//        // pick a random spot...
//        Vector2 where = new Vector2(
//            random.Next(0, screenWidth),
//            random.Next(0, screenHeight));

//        // ...and a random color. it's safe to cast random.Next to a byte,
//        // because MinimumStarBrightness and MaximumStarBrightness are both
//        // bytes.
//        byte greyValue =
//            (byte)random.Next(MinimumStarBrightness, MaximumStarBrightness);
//        Color color = new Color(greyValue, greyValue, greyValue);

//        // if the random number was greater than the percentage chance for a big
//        // star, this is just a normal star.
//        if ((float)random.NextDouble() > PercentBigStars)
//        {
//            starColors.Add(color);
//            stars.Add(where);
//        }
//        else
//        {
//            // if this star is randomly selected to be a "big" star, we actually
//            // add four points and colors to stars and starColors. big stars are
//            // a block of four points, instead of just one point.
//            for (int j = 0; j < 4; j++)
//            {
//                starColors.Add(color);
//            }

//            stars.Add(where);
//            stars.Add(where + Vector2.UnitX);
//            stars.Add(where + Vector2.UnitY);
//            stars.Add(where + Vector2.One);
//        }
//    }
//}

//    // and then tell it that we're done.
//    primitiveBatch.End();
//}


//private void DrawSun(Vector2 where)
//     {

//         // the sun is made from 4 lines in a circle.
//         primitiveBatch.Begin(PrimitiveType.LineList);

//         // draw the vertical and horizontal lines
//         primitiveBatch.AddVertex(where + new Vector2(0, SunSize), Color.White);
//         primitiveBatch.AddVertex(where + new Vector2(0, -SunSize), Color.White);

//         primitiveBatch.AddVertex(where + new Vector2(SunSize, 0), Color.White);
//         primitiveBatch.AddVertex(where + new Vector2(-SunSize, 0), Color.White);

//         // to know where to draw the diagonal lines, we need to use trig.
//         // cosine of pi / 4 tells us what the x coordinate of a circle's radius is
//         // at 45 degrees. the y coordinate normally would come from sin, but sin and
//         // cos 45 are the same, so we can reuse cos for both x and y.
//         float sunSizeDiagonal = (float)Math.Cos(MathHelper.PiOver4);

//         // since that trig tells us the x and y for a unit circle, which has a
//         // radius of 1, we need scale that result by the sun's radius.
//         sunSizeDiagonal *= SunSize;

//         primitiveBatch.AddVertex(
//             where + new Vector2(-sunSizeDiagonal, sunSizeDiagonal), Color.Gray);
//         primitiveBatch.AddVertex(
//             where + new Vector2(sunSizeDiagonal, -sunSizeDiagonal), Color.Gray);

//         primitiveBatch.AddVertex(
//             where + new Vector2(sunSizeDiagonal, sunSizeDiagonal), Color.Gray);
//         primitiveBatch.AddVertex(
//             where + new Vector2(-sunSizeDiagonal, -sunSizeDiagonal), Color.Gray);

//         primitiveBatch.End();
//     }
//// called to draw the spacewars ship at a point on the screen.
//private void DrawShip(Vector2 where)
//{
//    // tell the primitive batch to start drawing lines
//    primitiveBatch.Begin(PrimitiveType.LineList);

//    // from the nose, down the left hand side
//    primitiveBatch.AddVertex(
//        where + new Vector2(0f, -ShipSizeY), Color.White);
//    primitiveBatch.AddVertex(
//        where + new Vector2(-ShipSizeX, ShipSizeY), Color.White);

//    // to the right and up, into the cutout
//    primitiveBatch.AddVertex(
//        where + new Vector2(-ShipSizeX, ShipSizeY), Color.White);
//    primitiveBatch.AddVertex(
//        where + new Vector2(0f, ShipSizeY - ShipCutoutSize), Color.White);

//    // to the right and down, out of the cutout
//    primitiveBatch.AddVertex(
//        where + new Vector2(0f, ShipSizeY - ShipCutoutSize), Color.White);
//    primitiveBatch.AddVertex(
//        where + new Vector2(ShipSizeX, ShipSizeY), Color.White);

//    // and back up to the nose, where we started.
//    primitiveBatch.AddVertex(
//        where + new Vector2(ShipSizeX, ShipSizeY), Color.White);
//    primitiveBatch.AddVertex(
//        where + new Vector2(0f, -ShipSizeY), Color.White);

//    // and we're done.
//    primitiveBatch.End();
//}

//// called to draw the spacewars sun.