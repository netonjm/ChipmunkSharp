using System;
using ChipmunkSharp;
using CoreAnimation;
using CoreGraphics;

namespace AppKit
{
	public static class DrawExtensions
	{
		public static cpVect[] Add (this cpVect[] verts, float value)
		{
			for (int i = 0; i < verts.Length; i++) {
				verts[i].x += value;
				verts[i].y += value;
			}
			return verts;
		}
		public static void Substract (this cpVect[] verts, float value)
		{
			for (int i = 0; i < verts.Length; i++) {
				verts[i].x -= value;
				verts[i].y -= value;
			}
		}

		public static cpVect[] ToCpVect (this Rectangle rectangle)
		{
			var points = new cpVect[4];
			points[0] = new cpVect (rectangle.X, rectangle.Y);
			points[1] = new cpVect (rectangle.X + rectangle.Width, rectangle.Y);
			points[2] = new cpVect (rectangle.X + rectangle.Width, rectangle.Y + rectangle.Height);
			points[3] = new cpVect (rectangle.X, rectangle.Y + rectangle.Height);
			return points;
		}

		public static CGPoint[] ToCGPoints (this cpVect[] verts)
		{
			var points = new CGPoint[verts.Length];
			for (int i = 0; i < verts.Length; i++) {
				points[i] = verts[i].ToCGPoint ();
			}
			return points;
		}

		public static CGPoint ToCGPoint (this cpVect point)
		{
			return new CGPoint (point.x, point.y);
		}

		public static CGColor ToCGColor (this cpColor color)
		{
			return ToNSColor (color).CGColor;
		}

		public static nfloat CenterX (this CGRect sender)
		{
			return sender.Width / 2;
		}

		public static nfloat CenterY (this CGRect sender)
		{
			return sender.Height / 2;
		}

		public static void CenterIn (this CALayer sender, CALayer layer)
		{
			sender.Frame = new CGRect (layer.Frame.CenterX () - sender.Frame.CenterX (), layer.Frame.CenterX () - sender.Frame.CenterY (), sender.Frame.Width, sender.Frame.Height);
		}

		public static NSColor ToNSColor (this cpColor color)
		{
			return NSColor.FromRgba (color.r / 255, color.g / 255, color.b / 255, color.a / 255);
		}

		public static CGPath ToGCPath (this NSBezierPath bezierPath)
		{
			var path = new CGPath ();
			CGPoint[] points;
			for (int i = 0; i < bezierPath.ElementCount; i++) {
				var type = bezierPath.ElementAt (i, out points);
				switch (type) {
					case NSBezierPathElement.MoveTo:
						path.MoveToPoint (points[0]);
						break;
					case NSBezierPathElement.LineTo:
						path.AddLineToPoint (points[0]);
						break;
					case NSBezierPathElement.CurveTo:
						path.AddCurveToPoint (points[0], points[1], points[2]);
						break;
					case NSBezierPathElement.ClosePath:
						path.CloseSubpath ();
						break;
				}
			}
			return path;
		}
	}
}
