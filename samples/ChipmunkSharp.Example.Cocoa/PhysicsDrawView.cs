using System;
using AppKit;
using CoreGraphics;

namespace ChipmunkSharp
{
	class PhysicsDrawView : NSView
	{
		public PhysicsDrawFlags Flags = PhysicsDrawFlags.None;

		cpSpace _space;
		cpBody _body;

		readonly IDrawDelegate drawDelegate;

		public PhysicsDrawView (IDrawDelegate drawDelegate, cpSpace space)
		{
			this.drawDelegate = drawDelegate;
		}

		public cpBody Body {
			get { return _body; }
			set { _body = value; }
		}

		public CGPoint Position {
			get { return CocoaHelpers.cpVert2Point (_body.GetPosition ()); }
			set { _body.SetPosition (new cpVect ((float) value.X, (float)value.Y)); }
		}

		public void DebugDraw ()
		{
			//TODO: EEE

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

	}
}
