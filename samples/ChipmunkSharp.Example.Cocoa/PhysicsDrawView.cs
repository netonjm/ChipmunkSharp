using System;
using AppKit;
using CoreAnimation;
using CoreGraphics;

namespace ChipmunkSharp
{
	class PhysicsDrawView : NSView
	{
		public static cpColor CONSTRAINT_COLOR = cpColor.Grey; //new cpColor(0, 1, 0, 0.5f);
		public static cpColor TRANSPARENT_COLOR = new cpColor (0, 0, 0, 0.0f);

		public PhysicsDrawFlags Flags { get; set; } = PhysicsDrawFlags.None;
		public CocoaDrawDelegate drawDelegate { get; set; }

		readonly cpSpace space;

		public PhysicsDrawView (CocoaDrawDelegate drawDelegate, cpSpace space)
		{
			WantsLayer = true;
			this.space = space;
			this.drawDelegate = drawDelegate;
			drawDelegate.ContentLayer = Layer;
		}

		/// <summary>
		/// Append flags to the current flags.
		/// </summary>
		public void AppendFlags (params PhysicsDrawFlags[] flags)
		{
			foreach (var item in flags)
				Flags |= item;
		}

		/// <summary>
		/// Clear flags from the current flags.
		/// </summary>
		public void ClearFlags (params PhysicsDrawFlags[] flags)
		{
			foreach (var item in flags)
				Flags &= ~item;
		}

		void InternalDraw ()
		{
			if (space == null)
				return;

			if (Layer.Sublayers != null) {
				foreach (var item in Layer.Sublayers) {
					item.RemoveFromSuperLayer ();
				}
			}
			
			//DrawString (15, 15, string.Format ("Step: {0}", _space.stamp));
			//DrawString (15, 50, string.Format ("Bodies : {0}/{1}", _space.dynamicBodies.Count + _space.staticBodies.Count, _space.dynamicBodies.Capacity));
			//DrawString (15, 80, string.Format ("Arbiters: {0}/{1}", _space.arbiters.Count, _space.arbiters.Capacity));

			if (Flags.HasFlag (PhysicsDrawFlags.All) || Flags.HasFlag (PhysicsDrawFlags.BB) || Flags.HasFlag (PhysicsDrawFlags.Shapes))
				space.EachShape (DrawShape);

			if (Flags.HasFlag (PhysicsDrawFlags.Joints) || Flags.HasFlag (PhysicsDrawFlags.All))
				space.EachConstraint (DrawConstraint);

			var contacts = 0;

			if (Flags.HasFlag (PhysicsDrawFlags.All) || Flags.HasFlag (PhysicsDrawFlags.ContactPoints)) {
				for (var i = 0; i < space.arbiters.Count; i++) {
					for (int j = 0; j < space.arbiters[i].contacts.Count; j++)
						drawDelegate.Draw (space.arbiters[i].contacts[j]);
					contacts += space.arbiters[i].contacts.Count;
				}
			}

			//DrawString (15, 110, "Contact points: " + contacts);
			//DrawString (15, 140, string.Format ("Nodes:{1} Leaf:{0} Pairs:{2}", cp.numLeaves, cp.numNodes, cp.numPairs));
		}

		public void DebugDraw ()
		{
			InvokeOnMainThread (InternalDraw);
		}

		public void DrawConstraint (cpConstraint constraint)
		{
			Type klass = constraint.GetType ();
			if (klass == typeof (cpPinJoint))
				Draw ((cpPinJoint)constraint);
			else if (klass == typeof (cpSlideJoint))
				Draw ((cpSlideJoint)constraint);
			else if (klass == typeof (cpPivotJoint))
				Draw ((cpPivotJoint)constraint);
			else if (klass == typeof (cpGrooveJoint))
				Draw ((cpGrooveJoint)constraint);
			else if (klass == typeof (cpDampedSpring))
				Draw ((cpDampedSpring)constraint);
			else if (klass == typeof (cpDampedRotarySpring))
				Draw ((cpDampedRotarySpring)constraint);
			else if (klass == typeof (cpSimpleMotor))
				Draw ((cpSimpleMotor)constraint);
			else {
				//printf("Cannot draw constraint\n");
			}
		}
		public void DrawShape (cpShape shape)
		{
			cpBody body = shape.body;
			cpColor color = cp.GetShapeColor (shape); // ColorForBody(body);

			switch (shape.shapeType) {
				case cpShapeType.Circle: {
						cpCircleShape circle = (cpCircleShape)shape;

						if (Flags.HasFlag (PhysicsDrawFlags.BB) || Flags.HasFlag (PhysicsDrawFlags.All))
							drawDelegate.Draw (circle.bb);

						if (Flags.HasFlag (PhysicsDrawFlags.Shapes) || Flags.HasFlag (PhysicsDrawFlags.All))
							drawDelegate.Draw (circle, color);
					}
					break;
				case cpShapeType.Segment: {
						cpSegmentShape seg = (cpSegmentShape)shape;
						if (Flags.HasFlag (PhysicsDrawFlags.BB) || Flags.HasFlag (PhysicsDrawFlags.All))
							drawDelegate.Draw (seg.bb);

						if (Flags.HasFlag (PhysicsDrawFlags.Shapes) || Flags.HasFlag (PhysicsDrawFlags.All))
							drawDelegate.Draw (seg, color);
					}
					break;
				case cpShapeType.Polygon: {
						cpPolyShape poly = (cpPolyShape)shape;
						if (Flags.HasFlag (PhysicsDrawFlags.BB) || Flags.HasFlag (PhysicsDrawFlags.All))
							drawDelegate.Draw (poly.bb);

						if (Flags.HasFlag (PhysicsDrawFlags.Shapes) || Flags.HasFlag (PhysicsDrawFlags.All)) {
							drawDelegate.Draw (poly, color);
						}
					}
					break;
				default:
					cp.AssertHard (false, "Bad assertion in DrawShape()");
					break;
			}
		}

		#region DRAW CONSTRAINT

		private void Draw (cpDampedRotarySpring constraint)
		{
			//Not used
		}

		private void Draw (cpDampedSpring constraint)
		{
			var a = constraint.a.LocalToWorld (constraint.GetAnchorA ());
			var b = constraint.b.LocalToWorld (constraint.GetAnchorB ());

			drawDelegate.DrawSpring (a, b, CONSTRAINT_COLOR);
		}

		public void Draw (cpSimpleMotor cpSimpleMotor)
		{
			//Not used
		}

		private void Draw (cpGrooveJoint constraint)
		{
			var a = constraint.a.LocalToWorld (constraint.grv_a);
			var b = constraint.a.LocalToWorld (constraint.grv_b);
			var c = constraint.b.LocalToWorld (constraint.anchorB);

			drawDelegate.DrawSegment (a, b, 1, CONSTRAINT_COLOR);
			drawDelegate.DrawCircle (c, 5f, CONSTRAINT_COLOR);
		}

		private void Draw (cpPivotJoint constraint)
		{
			cpVect a = cpTransform.Point (constraint.a.transform, constraint.GetAnchorA ());
			cpVect b = cpTransform.Point (constraint.b.transform, constraint.GetAnchorB ());

			//DrawSegment(a, b, 1, cpColor.Grey);
			drawDelegate.DrawDot (a, 3, CONSTRAINT_COLOR);
			drawDelegate.DrawDot (b, 3, CONSTRAINT_COLOR);
		}

		public void Draw (cpSlideJoint constraint)
		{
			cpVect a = cpTransform.Point (constraint.a.transform, constraint.GetAnchorA ());
			cpVect b = cpTransform.Point (constraint.b.transform, constraint.GetAnchorB ());

			drawDelegate.DrawSegment (a, b, 1, cpColor.Grey);
			drawDelegate.DrawDot (a, 5, CONSTRAINT_COLOR);
			drawDelegate.DrawDot (b, 5, CONSTRAINT_COLOR);
		}

		public void Draw (cpPinJoint constraint)
		{
			cpVect a = cpTransform.Point (constraint.a.transform, constraint.GetAnchorA ());
			cpVect b = cpTransform.Point (constraint.b.transform, constraint.GetAnchorB ());

			drawDelegate.DrawSegment (a, b, 1, cpColor.Grey);
			drawDelegate.DrawDot (a, 5, CONSTRAINT_COLOR);
			drawDelegate.DrawDot (b, 5, CONSTRAINT_COLOR);
		}

		#endregion
	}
}
