using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using CocosSharp;
using ChipmunkSharp;
using Microsoft.Xna.Framework.Input;
using System.Diagnostics;

namespace ChipmunkExample
{
    public class CCMaxScene : CCScene
    {
        public CCLayer InformationLayer;
        public CCLayer ContainerLayer;
        public CCLayer BackgroundLayer;

        public CCMaxScene(CCLayer informationLayer, ChipmunkDemoLayer containerLayer)
            : base(AppDelegate.SharedWindow)
        {
            BackgroundLayer = containerLayer.BackgroundLayer = new CCLayer();
            containerLayer.InformationLayer = InformationLayer = informationLayer;
            ContainerLayer = containerLayer;
            BackgroundLayer.AddChild(ContainerLayer, 0);
            BackgroundLayer.AddChild(InformationLayer, 100);
            AddChild(BackgroundLayer);
        }
    }


    public class ChipmunkDemoLayer : CCLayer
    {
        public CCLayer InformationLayer;
        public CCLayer BackgroundLayer;
        public CCLayer ContainerLayer;

        public const int GRABBABLE_MASK_BIT = 1 << 31;

        public cpShapeFilter GRAB_FILTER = new cpShapeFilter(cp.NO_GROUP, GRABBABLE_MASK_BIT, GRABBABLE_MASK_BIT);
        public cpShapeFilter NOT_GRABBABLE_FILTER = new cpShapeFilter(cp.NO_GROUP, ~GRABBABLE_MASK_BIT, ~GRABBABLE_MASK_BIT);
        bool CollisionEnabled = true;
        public PhysicsDebugDraw m_debugDraw;

        public static Type[] bench_list = new Type[] {
			typeof(playerLayer), //player needs control with keyb
			typeof(queryLayer), //Some rare collisions 
			typeof(plinkLayer), 
			typeof(contactPointsLayer), //Pentagon and square in some moments cannot be selected, and collision fails detection
			typeof(ballLayer),	//Box cannot be selected (Query problem) ,  Ball launched with some streng to the wall transpases it (same query problem, not detect collision)
			typeof(pumpLayer), 
			typeof(craneLayer), 
			typeof(shatterLayer), //rare Colour blink
			typeof(gjkLayer), 	
			typeof(convexLayer),
			typeof(stickyLayer), 
			typeof(tankLayer), // Tank when its following the cursor seems the driver are drinked or drugged
			typeof(contactGraphLayer), 
			typeof(grossiniDanceLayer),  //Not aligned the image and Shape
			typeof(benchLayer), 
			typeof(buoyancyLayer), //Change water to transparent
			typeof(chainsLayer),
			typeof(oneWayLayer),
			typeof(planetLayer),
			typeof(pyramidLayer), //Collision on central block its wear
			typeof(pyramidToppleLayer), 
			typeof(jointsLayer), //Collision on some shapes not work
			typeof(theoJansenLayer),
			typeof(unicycleLayer),
			typeof(logoSmashLayer),
			typeof(sliceLayer),
		};

        public static CCScene ActualScene
        {
            get
            {
                ChipmunkDemoLayer demoLayer = Activator.CreateInstance(
                    bench_list[ScenePlayer.Index], new object[] { }) as ChipmunkDemoLayer;
                if (demoLayer == null)
                    demoLayer = new ballLayer();

                CCLayer InformationLayer = new CCLayer(AppDelegate.DefaultResolution);
                CCMaxScene scene = new CCMaxScene(InformationLayer, demoLayer);
                return scene;
            }
        }

        public cpVect ChipmunkDemoKeyboard = cpVect.Zero;
        static PlayerIndex ScenePlayer = new PlayerIndex(bench_list.Length);
        static PlayerIndex SubScenePlayer = new PlayerIndex();
        public float Width = 640;
        public float Height = 480;

        public float scale;

        const string DEFAULT_FONT = "weblysleekuisb-22";
        const string DEFAULT_FULL_FONT = "fonts/" + DEFAULT_FONT;
        const string DEFAULT_FONT_NAME = "WeblySleek UI Semibold";
        const int DEFAULT_FONT_SIZE = 22;

        public cpSpace space;
        CCEventListenerTouchAllAtOnce eTouch;

        CCMenu menuLayerPlayer;
        CCSprite background;
        CCSprite background_white;
        LogoNode logo;

        public CCSize windowSize { get { return AppDelegate.SharedWindow.WindowSizeInPixels; } }

        public static CCLabelTtf example;
        public static string sceneName = "";
        public static string subSceneName = "";

        public static int subSceneNumber { get { if (SubScenePlayer != null) return SubScenePlayer.Count; return 0; } }
        public static int subSceneIndex { get { if (SubScenePlayer != null) return SubScenePlayer.Index; return 0; } }

        //bool debug_poligon = true;
        bool debug_shapes = true;
        bool debug_joints = true;
        bool debug_contacts = false;
        bool debug_bb = false;

        public void RefreshDebug()
        {
            m_debugDraw.Flags = PhysicsDrawFlags.None;

            if (debug_shapes)
                m_debugDraw.Flags = PhysicsDrawFlags.Shapes;

            if (debug_joints)
                if (m_debugDraw.Flags.HasFlag(PhysicsDrawFlags.None))
                    m_debugDraw.Flags = PhysicsDrawFlags.Joints;
                else
                    m_debugDraw.AppendFlags(PhysicsDrawFlags.Joints);

            if (debug_contacts)
                if (m_debugDraw.Flags.HasFlag(PhysicsDrawFlags.None))
                    m_debugDraw.Flags = PhysicsDrawFlags.ContactPoints;
                else
                    m_debugDraw.AppendFlags(PhysicsDrawFlags.ContactPoints);


            if (debug_bb)
                if (m_debugDraw.Flags.HasFlag(PhysicsDrawFlags.None))
                    m_debugDraw.Flags = PhysicsDrawFlags.BB;
                else
                    m_debugDraw.AppendFlags(PhysicsDrawFlags.BB);
        }


        protected override void AddedToScene()
        {
            base.AddedToScene();

            //Our information layer it's on parent layer because our information text are statics
            background = new CCSprite("Background.png");
            background.Position = windowSize.Center;
            background.Scale = windowSize.Height / background.Texture.ContentSizeInPixels.Height;
            BackgroundLayer.AddChild(background, -3);
            this.Position = windowSize.Center;

            //Creation menu debug options
            var lbl1 = new CCMenuItemLabelTTF(GetDefaultFontTtf("Shapes"), (o) => { debug_shapes = !debug_shapes; RefreshDebug(); });
            lbl1.AnchorPoint = new CCPoint(0, .5f);
            var lbl2 = new CCMenuItemLabelTTF(GetDefaultFontTtf("Joins"), (o) => { debug_joints = !debug_joints; RefreshDebug(); });
            lbl2.AnchorPoint = new CCPoint(0, .5f);
            var lbl3 = new CCMenuItemLabelTTF(GetDefaultFontTtf("Contact"), (o) => { debug_contacts = !debug_contacts; RefreshDebug(); });
            lbl3.AnchorPoint = new CCPoint(0, .5f);
            var lbl4 = new CCMenuItemLabelTTF(GetDefaultFontTtf("BB"), (o) => { debug_bb = !debug_bb; RefreshDebug(); });
            lbl4.AnchorPoint = new CCPoint(0, .5f);
            CCMenu menu = new CCMenu(new CCMenuItem[] { lbl1, lbl2, lbl3, lbl4 });
            menu.AlignItemsVertically();
            menu.Position = new CCPoint(windowSize.Width * .02f, windowSize.Height * .35f);
            InformationLayer.AddChild(menu);

            background_white = new CCSprite("Background-alone.png");
            background_white.Position = windowSize.Center;
            background_white.Scale = windowSize.Height / background_white.Texture.ContentSizeInPixels.Height;
            background_white.Visible = false;
            InformationLayer.AddChild(background_white, -4);

            logo = new LogoNode();
            logo.InitialPosition = new CCPoint(new CCPoint(windowSize.Width * .90f, windowSize.Height * .05f));
            logo.Position = new CCPoint(logo.InitialPosition);

            InformationLayer.AddChild(logo, -2);

            logo.RunAction(new CCRepeatForever(
                new CCEaseSineInOut(new CCMoveBy(1.5f, new CCPoint(0, 10))),
                new CCEaseSineInOut(new CCMoveBy(1.5f, new CCPoint(0, -10))))
                );

            //Debug draw initialization
            //m_debugDraw = new CCChipmunkDebugDraw(DEFAULT_FULL_FONT);

            //Space initialization
            space = new cpSpace();
			//space.SetDebugDraw(m_debugDraw);

			space.CollisionEnabled = CollisionEnabled;
            m_debugDraw = new PhysicsDebugDraw(space);
         CollisionEnabled
            AddChild(m_debugDraw);
            RefreshDebug();

            //Title initialization
            example = GetDefaultFontTtf("Physics examples");
            example.AnchorPoint = new CCPoint(.5f, 1f);
            //example.Scale = .3f;
            InformationLayer.AddChild(example);
            SetTitle(this.GetType().Name);

            //Player initialization
            menuLayerPlayer = GetPlayerLayer();
            menuLayerPlayer.AnchorPoint = new CCPoint(0, 0);

            menuLayerPlayer.Position = new CCPoint(Window.WindowSizeInPixels.Width * .12f, Window.WindowSizeInPixels.Height * .05f);
            InformationLayer.AddChild(menuLayerPlayer);

            //Listeners initializations
            //eMouse = new CCEventListenerMouse();
            //eMouse.OnMouseMove += OnMouseMove;
            //eMouse.OnMouseDown += OnMouseDown;
            //eMouse.OnMouseUp += OnMouseUp;
            //eMouse.OnMouseScroll = e => CCMouse.Instance.OnMouseScroll(e, this);
            //AddEventListener(eMouse);

            eTouch = new CCEventListenerTouchAllAtOnce();
            eTouch.OnTouchesBegan += OnTouchesBegan;
            eTouch.OnTouchesMoved += OnTouchesMoved;
            eTouch.OnTouchesCancelled += OnTouchesCancelled;
            eTouch.OnTouchesEnded += OnTouchesEnded;
            AddEventListener(eTouch);
        }

        public static CCLabel GetDefaultFont(string defaultText)
        {
            return new CCLabel(defaultText, DEFAULT_FONT, DEFAULT_FONT_SIZE); ;
        }

        public static CCLabelTtf GetDefaultFontTtf(string defaultText)
        {
            return new CCLabelTtf(defaultText, DEFAULT_FONT, DEFAULT_FONT_SIZE); ;
        }

        public void HideBackground()
        {
            background_white.Visible = true;
            background.RunActions(new CCFadeOut(1.5f), new CCCallFunc(OnBackgroundHided));
        }

        public static void RefreshTittle()
        {
            example.Text = string.Format("{0}: {1} {2}",
                sceneName.Replace("_", " "),
                subSceneName.Replace("_", " "),
                subSceneNumber == 1 ? "" : string.Format("{0}/{1}", subSceneIndex, subSceneNumber)
            );
        }

        /// <summary>
        /// Creates a subscene player
        /// </summary>
        /// <param name="subScenes"></param>
        public void CreateSubScenePlayer(int subScenes)
        {
            var menu = GetPlayerSubLayer();
            menuLayerPlayer.AnchorPoint = new CCPoint(0, 0);
            menu.Position = new CCPoint(Window.WindowSizeInPixels.Width * .67f, Window.WindowSizeInPixels.Height * .05f);
            InformationLayer.AddChild(menu);
            SubScenePlayer.Count = subScenes - 1;
        }

        /// <summary>
        /// Clear actual space
        /// </summary>
        public void ClearSpace()
        {
            space.Clear();
        }

        /// <summary>
        /// This method is called when next scene , previous scene or refresh is pressed (needs override)
        /// </summary>
        /// <param name="index"></param>
        public virtual void ChangeActualSubScene(int index)
        {

        }

        /// <summary>
        /// Instances using reflection the example type selected
        /// </summary>
        private void LoadActualIndexScene()
        {
            Director.ReplaceScene(ActualScene);
        }

        public void RefreshSubScene()
        {
            ClearSpace();
            ChangeActualSubScene(SubScenePlayer.Index);
        }

        /// <summary>
        /// Loads next example scene
        /// </summary>
        public void NextScene()
        {
            ScenePlayer.Next();
            LoadActualIndexScene();
        }

        /// <summary>
        /// Loads previous example scene
        /// </summary>
        public void PrevScene()
        {
            ScenePlayer.Prev();
            LoadActualIndexScene();
        }

        /// <summary>
        /// Loads next subscene example
        /// </summary>
        void NextSubScene()
        {
            SubScenePlayer.Next();
            RefreshSubScene();
        }

        /// <summary>
        /// Loads previous subscene example
        /// </summary>
        void PrevSubScene()
        {
            SubScenePlayer.Prev();
            RefreshSubScene();
        }

        /// <summary>
        /// Sets the title of the scene
        /// </summary>
        /// <param name="p"></param>
        public static void SetTitle(string p)
        {
            sceneName = p;
            RefreshTittle();
        }

        /// <summary>
        /// Sets the title of the subscene
        /// </summary>
        /// <param name="p"></param>
        public static void SetSubTitle(string p)
        {
            subSceneName = p;
            RefreshTittle();
        }

        /// <summary>
        /// Adds and get a player menu control on the container selected with his predefined actions
        /// </summary>
        /// <param name="container"></param>
        /// <param name="previousAction"></param>
        /// <param name="nextAction"></param>
        /// <param name="altTitle"></param>
        /// <param name="altAction"></param>
        /// <param name="scale"></param>
        /// <returns></returns>

        /// <summary>
        /// Adds and get a player menu control on the container selected with his predefined actions
        /// </summary>
        public CCMenu GetPlayerLayer()
        {
            CCMenuItemImage itm = new CCMenuItemImage("arrow-left.png", "arrow-left-press.png", o => PrevScene());
            //itm.Position = new CCPoint(0, 0);
            itm.AnchorPoint = new CCPoint(0, 0);
            CCMenuItemImage itm2 = new CCMenuItemImage("arrow-right.png", "arrow-right-press.png", o => NextScene());
            //itm2.Position = new CCPoint(itm.PositionX + itm.ContentSize.Width + separation, itm.PositionY);
            itm2.AnchorPoint = new CCPoint(0, 0);
            CCMenuItemImage itm3 = new CCMenuItemImage("btn-reset.png", "btn-reset-press.png", o => LoadActualIndexScene());
            //itm2.Position = new CCPoint(itm2.PositionX + separation, itm.PositionY);
            itm3.AnchorPoint = new CCPoint(0, 0);

            CCMenu menu = new CCMenu(itm, itm2, itm3);
            menu.AlignItemsHorizontally(3);
            menu.AnchorPoint = new CCPoint(0, 0);
            return menu;
        }

        /// <summary>
        /// Adds and get a player menu control on the container selected with his predefined actions
        /// </summary>
        public CCMenu GetPlayerSubLayer()
        {
            CCMenuItemImage itm = new CCMenuItemImage("arrow-left.png", "arrow-left-press.png", o => PrevSubScene());
            itm.AnchorPoint = new CCPoint(0, 0);
            CCMenuItemImage itm2 = new CCMenuItemImage("arrow-right.png", "arrow-right-press.png", o => NextSubScene());
            itm2.AnchorPoint = new CCPoint(0, 0);
            CCMenuItemImage itm3 = new CCMenuItemImage("btn-clear.png", "btn-clear-press.png", o => ClearSpace());
            itm3.AnchorPoint = new CCPoint(0, 0);

            CCMenu menu = new CCMenu(itm, itm2, itm3);
            menu.AlignItemsHorizontally(3);
            menu.AnchorPoint = new CCPoint(0, 0);
            return menu;
        }

        public void addFloor()
        {
            var floor = space.AddShape(
                new cpSegmentShape(space.GetStaticBody(), new cpVect(-Width * .5f, -Height * .5f), new cpVect(Width * .5f, -Height * .5f),
                    0f));
            floor.SetElasticity(1);
            floor.SetFriction(1);
            floor.SetFilter(NOT_GRABBABLE_FILTER);
        }

        public void addWalls()
        {
            var space = this.space;
            var wall1 = space.AddShape(new cpSegmentShape(space.GetStaticBody(), new cpVect(-Width * .5f, -Height * .5f), new cpVect(-Width * .5f, Height * .5f), 0));
            wall1.SetElasticity(1);
            wall1.SetFriction(1);
            wall1.SetFilter(NOT_GRABBABLE_FILTER);

            var wall2 = space.AddShape(new cpSegmentShape(space.GetStaticBody(), new cpVect(Width * .5f, -Height * .5f), new cpVect(Width * .5f, Height * .5f), 0));
            wall2.SetElasticity(1);
            wall2.SetFriction(1);
            wall2.SetFilter(NOT_GRABBABLE_FILTER);
        }

        public override void Update(float dt)
        {
            base.Update(dt);

#if !WINDOWS_PHONE
            MouseEvents();
#endif
            Draw();
        }

#if !WINDOWS_PHONE

        public void MouseEvents()
        {
            var eMouse = Mouse.GetState();
            CCMouse.Instance.UpdatePosition(eMouse.X, eMouse.Y, this);
            CCMouse.Instance.rightclick = eMouse.RightButton == ButtonState.Pressed;
            CCMouse.Instance.leftclick = eMouse.LeftButton == ButtonState.Pressed;
            CCMouse.Instance.midclick = eMouse.MiddleButton == ButtonState.Pressed;
        }

#endif

        public virtual void OnBackgroundHided()
        {
        }

        #region Touch events

        public virtual void OnTouchesEnded(List<CCTouch> touches, CCEvent arg2)
        {

            CCMouse.Instance.OnTouchesEnded(touches, this);

            if (logo.IsMoving)
            {
                CCMouse.Instance.IsDragBlocked = logo.IsMoving = false;
                logo.RunAction(new CCEaseSineInOut(new CCMoveTo(2, logo.InitialPosition)));
            }

            if (!CCMouse.Instance.rightclick)
                if (CCMouse.Instance.HasBodyJoined)
                {
                    space.RemoveConstraint(CCMouse.Instance.mouseJoint);
                    CCMouse.Instance.mouseJoint = null;
                }
        }


        public virtual void OnTouchesMoved(List<CCTouch> touches, CCEvent e)
        {

            CCMouse.Instance.OnTouchesMoved(touches, this);

            if (CCMouse.Instance.HasPosition)
            {
                // Move mouse body toward the mouse
                CCMouse.Instance.UpdateBodyVelocity();
                CCMouse.Instance.UpdateBodyPosition();
            }

            if (logo.IsMoving)
            {
                logo.Position = new CCPoint(touches.FirstOrDefault().LocationOnScreen) - logo.MoveOffset;
            }
        }

        public virtual void OnTouchesCancelled(List<CCTouch> touches, CCEvent e)
        {
            CCMouse.Instance.OnTouchesCancelled(touches, this);
        }

        public virtual void OnTouchesBegan(List<CCTouch> touches, CCEvent e)
        {
            var touch = touches.FirstOrDefault();


            CCMouse.Instance.UpdatePositionLocation(touch.LocationOnScreen, this); //Update mouse mouse position 

            CCMouse.Instance.UpdateBodyPosition();

            CCMouse.Instance.OnTouchBegan(touch, this);

            if (!CCMouse.Instance.HasBodyJoined)
            {
                float radius = 5.0f;

                cpPointQueryInfo info = null;
                var shape = space.PointQueryNearest(
                    CCMouse.Instance.Position, radius, GRAB_FILTER, ref info);
                if (shape != null)
                {
                    cpVect nearest = (info.distance > 0.0d ? info.point : CCMouse.Instance.Position);

                    CCMouse.Instance.mouseJoint = new cpPivotJoint(CCMouse.Instance.mouseBody, shape.body, cpVect.Zero, shape.body.WorldToLocal(nearest));
                    CCMouse.Instance.mouseJoint.SetMaxForce(50000);
                    CCMouse.Instance.mouseJoint.SetErrorBias(cp.cpfpow(1f - 0.15f, 60f));
                    space.AddConstraint(CCMouse.Instance.mouseJoint);
                    return;
                }
            }

            //Arrastramos el logo
            if (logo.BoundingBox.ContainsPoint(CCMouse.Instance.PositionParentSpace))
            {
                logo.MoveOffset = touch.LocationOnScreen - logo.Position;
                CCMouse.Instance.IsDragBlocked = logo.IsMoving = true;
                return;
            }

        }

        #endregion

        protected virtual void Draw()
        {
            m_debugDraw.DebugDraw();
            if (CCMouse.Instance.HasPosition)
                m_debugDraw.DrawCircle(CCMouse.Instance.Position, 2.0f, CCMouse.Instance.mouseJoint != null ? cpColor.WhiteGreen : cpColor.WhiteRed);
        }

        public override void OnExit()
        {
            base.OnExit();
            //RemoveEventListener(eMouse);
            RemoveEventListener(eTouch);
        }
    }
}
