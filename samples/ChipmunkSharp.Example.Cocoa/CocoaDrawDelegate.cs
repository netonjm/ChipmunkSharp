using System;
using AppKit;
using CoreAnimation;

namespace ChipmunkSharp
{
	public class CocoaDrawDelegate
	{
		public CALayer ContentLayer { get; set; }

		public static cpColor CONSTRAINT_COLOR = cpColor.Grey; //new cpColor(0, 1, 0, 0.5f);
		public static cpColor TRANSPARENT_COLOR = new cpColor (0, 0, 0, 0.0f);

		cpVect[] springPoints = {
	new cpVect(0.00f, 0.0f),
	new cpVect(0.20f, 0.0f),
	new cpVect(0.25f, 3.0f),
	new cpVect(0.30f, -6.0f),
	new cpVect(0.35f, 6.0f),
	new cpVect(0.40f, -6.0f),
	new cpVect(0.45f, 6.0f),
	new cpVect(0.50f, -6.0f),
	new cpVect(0.55f, 6.0f),
	new cpVect(0.60f, -6.0f),
	new cpVect(0.65f, 6.0f),
	new cpVect(0.70f, -3.0f),
	new cpVect(0.75f, 6.0f),
	new cpVect(0.80f, 0.0f),
	new cpVect(1.00f, 0.0f)
		};

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

		public void DrawSpring (cpVect a, cpVect b, cpColor cpColor)
		{
			DrawDot (a, 5, CONSTRAINT_COLOR);
			DrawDot (b, 5, CONSTRAINT_COLOR);

			cpVect delta = cpVect.cpvsub (b, a);
			float cos = delta.x;
			float sin = delta.y;
			float s = 1.0f / cpVect.cpvlength (delta);

			cpVect r1 = cpVect.cpv (cos, -sin * s);
			cpVect r2 = cpVect.cpv (sin, cos * s);

			cpVect[] verts = new cpVect[springPoints.Length];
			for (int i = 0; i < springPoints.Length; i++) {
				cpVect v = springPoints[i];
				verts[i] = new cpVect (cpVect.cpvdot (v, r1) + a.x, cpVect.cpvdot (v, r2) + a.y);
			}

			for (int i = 0; i < springPoints.Length - 1; i++) {
				DrawSegment (verts[i], verts[i + 1], 1, cpColor.Grey);
			}
		}

		#region	DRAW SHAPES

		public void DrawCircle (cpVect center, float radius, cpColor color)
		{
			ContentLayer.AddSublayer (CALayerHelper.GetCircleLayer (center.ToCGPoint (), radius, color.ToCGColor ()));
		}

		public void DrawSolidCircle (cpVect center, float radius, cpColor color, cpColor borderColor, int borderWidth)
		{
			ContentLayer.AddSublayer (CALayerHelper.GetSolidCircleLayer (center.ToCGPoint (), radius, color.ToCGColor (), borderColor.ToCGColor ()));
		}

		public void DrawCircle (cpVect center, float radius, float angle, int segments, cpColor color)
		{
			
		}

		public void DrawDot (cpVect pos, float radius, cpColor color)
		{
			var backgroundColor = color.ToCGColor ();
			var background = CALayerHelper.GetSolidCircleLayer (pos.ToCGPoint (), radius, backgroundColor, backgroundColor);
			ContentLayer.AddSublayer (background);

			var pointColor = (color * 0.5f).ToCGColor ();
			var pointLayer = CALayerHelper.GetSolidCircleLayer (pos.ToCGPoint (), 2, pointColor, pointColor);
			ContentLayer.AddSublayer (pointLayer);
			pointLayer.CenterIn (background);
		}

		public void DrawPolygon (cpVect[] verts, cpColor fillColor, cpColor borderColor, float borderWidth = 0)
		{
			ContentLayer.AddSublayer (CALayerHelper.GetPolygon (verts.ToCGPoints (), fillColor.ToCGColor (), borderColor.ToCGColor (), borderWidth));
		}

		public void DrawRect (Rectangle rect, cpColor color)
		{
			DrawPolygon (rect.ToCpVect (), color, color + 2, 2);
		}

		public void DrawSegment (cpVect from, cpVect to, float radius, cpColor color)
		{
			var backgroundColor = color.ToCGColor ();
			ContentLayer.AddSublayer (CALayerHelper.DrawSegment (from.ToCGPoint (), to.ToCGPoint (), radius, backgroundColor, backgroundColor));
		}

		public void Draw (cpPolyShape poly, cpColor color)
		{
			cpColor fill = new cpColor (color);
			fill.a = cp.cpflerp (color.a, 1.0f, 0.5f);
			DrawPolygon (poly.GetVertices (), fill, color, poly.GetRadius ());
		}

		public void Draw (cpBB bb)
		{
			Draw (bb, cpColor.CyanBlue);
		}

		public void Draw (cpBB bb, cpColor color)
		{
			DrawPolygon (new cpVect[] {

						new cpVect(bb.r, bb.b),
					new cpVect(bb.r, bb.t),
					new cpVect(bb.l, bb.t),
					new cpVect(bb.l, bb.b)

				}, TRANSPARENT_COLOR, color, 1);
		}

		public void Draw (cpContact contact)
		{
			DrawDot (contact.r1, 0.5f, cpColor.Red);
			DrawDot (contact.r2, 0.5f, cpColor.Red);
		}

		public void Draw (cpCircleShape circle, cpColor color)
		{
			cpVect center = circle.tc;
			float radius = circle.r;
			cpVect To = cpVect.cpvadd (cpVect.cpvmult (circle.body.GetRotation (), circle.r), (circle.tc));
			DrawCircle (center, cp.cpfmax (radius, 1.0f), color);
			DrawSegment (center, To, 0.5f, cpColor.Grey);
		}

		public void Draw (cpSegmentShape seg, cpColor color)
		{
			DrawFatSegment (seg.ta, seg.tb, seg.r, color);
		}

		public void DrawFatSegment (cpVect ta, cpVect tb, float r, cpColor color)
		{
			cpColor fill = new cpColor (color);
			fill.a = cp.cpflerp (color.a, 1.0f, 0.5f);

			DrawSegment (ta, tb, Math.Max (1, r), fill);
		}

		public void Draw (cpVect point)
		{
			Draw (point, 0.5f);
		}

		public void Draw (cpVect point, cpColor color)
		{
			Draw (point, 0.5f, color);
		}

		public void Draw (cpVect point, float radius)
		{
			DrawDot (point, radius, cpColor.Red);
		}

		public void Draw (cpVect point, float radius, cpColor color)
		{
			DrawDot (point, radius, color);
		}

		#endregion

		#region DRAW CONSTRAINT

		public void Draw (cpDampedRotarySpring constraint)
		{
			//Not used
		}

		public void Draw (cpDampedSpring constraint)
		{
			var a = constraint.a.LocalToWorld (constraint.GetAnchorA ());
			var b = constraint.b.LocalToWorld (constraint.GetAnchorB ());

			DrawSpring (a, b, CONSTRAINT_COLOR);
		}

		public void Draw (cpSimpleMotor cpSimpleMotor)
		{
			//Not used
		}

		public void Draw (cpGrooveJoint constraint)
		{
			var a = constraint.a.LocalToWorld (constraint.grv_a);
			var b = constraint.a.LocalToWorld (constraint.grv_b);
			var c = constraint.b.LocalToWorld (constraint.anchorB);

			DrawSegment (a, b, 1, CONSTRAINT_COLOR);
			DrawCircle (c, 5f, CONSTRAINT_COLOR);
		}

		public void Draw (cpPivotJoint constraint)
		{
			cpVect a = cpTransform.Point (constraint.a.transform, constraint.GetAnchorA ());
			cpVect b = cpTransform.Point (constraint.b.transform, constraint.GetAnchorB ());

			//DrawSegment(a, b, 1, cpColor.Grey);
			DrawDot (a, 3, CONSTRAINT_COLOR);
			DrawDot (b, 3, CONSTRAINT_COLOR);
		}

		public void Draw (cpSlideJoint constraint)
		{
			cpVect a = cpTransform.Point (constraint.a.transform, constraint.GetAnchorA ());
			cpVect b = cpTransform.Point (constraint.b.transform, constraint.GetAnchorB ());

			DrawSegment (a, b, 1, cpColor.Grey);
			DrawDot (a, 5, CONSTRAINT_COLOR);
			DrawDot (b, 5, CONSTRAINT_COLOR);
		}

		public void Draw (cpPinJoint constraint)
		{
			cpVect a = cpTransform.Point (constraint.a.transform, constraint.GetAnchorA ());
			cpVect b = cpTransform.Point (constraint.b.transform, constraint.GetAnchorB ());

			DrawSegment (a, b, 1, cpColor.Grey);
			DrawDot (a, 5, CONSTRAINT_COLOR);
			DrawDot (b, 5, CONSTRAINT_COLOR);
		}

		#endregion
	}
}
