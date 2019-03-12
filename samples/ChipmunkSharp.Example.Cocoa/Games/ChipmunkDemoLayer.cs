using AppKit;
using CoreGraphics;

namespace ChipmunkSharp.Example
{
	class ChipmunkDemoLayer : NSView
	{
		public override bool IsFlipped => true;

		public static string subSceneName = "";

		public const int GRABBABLE_MASK_BIT = 1 << 31;

		public cpShapeFilter GRAB_FILTER = new cpShapeFilter (cp.NO_GROUP, GRABBABLE_MASK_BIT, GRABBABLE_MASK_BIT);
		public cpShapeFilter NOT_GRABBABLE_FILTER = new cpShapeFilter (cp.NO_GROUP, ~GRABBABLE_MASK_BIT, ~GRABBABLE_MASK_BIT);

		public float Width = 640;
		public float Height = 480;

		bool debug_shapes = true;
		bool debug_joints = true;
		bool debug_contacts = false;
		bool debug_bb = false;

		public float scale;

		public cpSpace space;

		public PhysicsDrawView m_debugDraw;

		public ChipmunkDemoLayer (PhysicsDrawView debugDraw, cpSpace space)
		{
			this.space = space;

			m_debugDraw = debugDraw;

			AddSubview (m_debugDraw as NSView);
		}

		public override void SetFrameSize (CGSize newSize)
		{
			base.SetFrameSize (newSize);
			m_debugDraw.Frame = Frame;
		}

		/// <summary>
		/// Clear actual space
		/// </summary>
		protected void ClearSpace ()
		{
			space.Clear ();
		}

		public virtual void Update (float dt)
		{

		}

		public void RefreshDebug ()
		{
			m_debugDraw.Flags = PhysicsDrawFlags.None;

			if (debug_shapes)
				m_debugDraw.Flags = PhysicsDrawFlags.Shapes;

			if (debug_joints)
				if (m_debugDraw.Flags.HasFlag (PhysicsDrawFlags.None))
					m_debugDraw.Flags = PhysicsDrawFlags.Joints;
				else
					m_debugDraw.AppendFlags (PhysicsDrawFlags.Joints);

			if (debug_contacts)
				if (m_debugDraw.Flags.HasFlag (PhysicsDrawFlags.None))
					m_debugDraw.Flags = PhysicsDrawFlags.ContactPoints;
				else
					m_debugDraw.AppendFlags (PhysicsDrawFlags.ContactPoints);


			if (debug_bb)
				if (m_debugDraw.Flags.HasFlag (PhysicsDrawFlags.None))
					m_debugDraw.Flags = PhysicsDrawFlags.BB;
				else
					m_debugDraw.AppendFlags (PhysicsDrawFlags.BB);
		}

		public void AddFloor ()
		{
			var floor = space.AddShape (
				new cpSegmentShape (space.GetStaticBody (), new cpVect (0, 1), new cpVect (Width, 1),
					0f));
			floor.SetElasticity (1);
			floor.SetFriction (1);
			floor.SetFilter (NOT_GRABBABLE_FILTER);
		}

		public void AddWalls ()
		{
			var space = this.space;
			var wall1 = space.AddShape (new cpSegmentShape (space.GetStaticBody (), new cpVect (1, Height), new cpVect (1, 1), 0));
			wall1.SetElasticity (1);
			wall1.SetFriction (1);
			wall1.SetFilter (NOT_GRABBABLE_FILTER);

			var wall2 = space.AddShape (new cpSegmentShape (space.GetStaticBody (), new cpVect (Width, Height), new cpVect (Width, 1), 0));
			wall2.SetElasticity (1);
			wall2.SetFriction (1);
			wall2.SetFilter (NOT_GRABBABLE_FILTER);
		}
	}
}
