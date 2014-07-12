using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ChipmunkSharp;
using Microsoft.Xna.Framework.Input;
using ChipmunkSharp.Constraints;
using Microsoft.Xna.Framework;

namespace ChipmunkExample
{
    public class CSBaseLayer : Game
    {

        #region Fields

        List<Vector2> pointsClicked = new List<Vector2>();

        public GraphicsDeviceManager graphics;

        // PrimitiveBatch is the new class introduced in this sample. We'll use it to
        // draw everything in this sample, including the stars, ships, and sun.
        public PrimitiveBatch m_debugDraw;

        // these two lists, stars, and starColors, keep track of the positions and
        // colors of all the stars that we will randomly generate during the initialize
        // phase.

        public cpSpace space;

        #endregion

        #region Initialization
        public CSBaseLayer()
        {
            Window.Title = "ChipmunkSharp Framework";

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
            m_debugDraw.AppendFlags(cpDrawFlags.Shape | cpDrawFlags.AABB | cpDrawFlags.CenterOfMass | cpDrawFlags.Joint | cpDrawFlags.Pair);
            m_debugDraw.DrawColor = cpColor.Blue;

            space = new cpSpace();
            space.SetDebugDraw(m_debugDraw);

            PostLoadContent();
        }

        protected virtual void PostLoadContent()
        {

        }


        public void addFloor()
        {
            // var space = this.space;
            var floor = space.addShape(new cpSegmentShape(space.staticBody, new cpVect(0, 0), new cpVect(640, 0), 0f));
            floor.setElasticity(1);
            floor.setFriction(1);
            floor.setLayers(cp.NOT_GRABABLE_MASK);
        }

        public void addWalls()
        {

            var space = this.space;
            var wall1 = space.addShape(new cpSegmentShape(space.staticBody, new cpVect(0, 0), new cpVect(0, 480), 0));
            wall1.setElasticity(1);
            wall1.setFriction(1);
            wall1.setLayers(cp.NOT_GRABABLE_MASK);

            var wall2 = space.addShape(new cpSegmentShape(space.staticBody, new cpVect(640, 0), new cpVect(640, 480), 0));
            wall2.setElasticity(1);
            wall2.setFriction(1);
            wall2.setLayers(cp.NOT_GRABABLE_MASK);

        }


        #endregion

        #region Update and Draw

        private cpVect mousePosition;
        private bool mousePressed;

        /// <summary>
        /// Allows the game to run logic such as updating the world,
        /// checking for collisions, gathering input and playing audio.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Update(GameTime gameTime)
        {
            base.Update(gameTime);

            var dt = (float)gameTime.ElapsedGameTime.TotalSeconds;

            // Allows the game to exit
            if ((GamePad.GetState(PlayerIndex.One).Buttons.Back == ButtonState.Pressed)
                || Keyboard.GetState().IsKeyDown(Keys.Escape))
                this.Exit();

            var mouse = Mouse.GetState();

            if (mouse.LeftButton == ButtonState.Released)
            {
                mousePressed = false;
                OnMouseReleased(mouse);
            }
            if (mouse.LeftButton == ButtonState.Pressed)
            {
                if (!mousePressed)
                {
                    mousePressed = true;
                    OnMousePressed(mouse);
                }
            }

            cpVect actualMousePosition = new cpVect(mouse.X, mouse.Y);

            if (mousePosition != actualMousePosition)
            {
                mousePosition = actualMousePosition;
                OnMouseMoved(mouse);
            }

            Update(dt);
        }

        public virtual void OnMousePressed(MouseState mouse)
        {

        }

        public virtual void OnMouseReleased(MouseState mouse)
        {

        }

        public virtual void OnMouseMoved(MouseState mouse)
        {

        }



        /// <summary>
        /// This is called when the game should draw itself.
        /// </summary>
        /// <param name="gameTime">Provides a snapshot of timing values.</param>
        protected override void Draw(GameTime gameTime)
        {
            base.Draw(gameTime);

            graphics.GraphicsDevice.Clear(Color.Black);

            m_debugDraw.Begin(Microsoft.Xna.Framework.Graphics.PrimitiveType.TriangleList);

            Draw();

            // and we're done.

            m_debugDraw.End();

        }

        protected virtual void Draw()
        {
        }

        protected virtual void Update(float dt)
        {

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
