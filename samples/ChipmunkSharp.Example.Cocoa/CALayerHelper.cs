using System;
using AppKit;
using CoreAnimation;
using CoreGraphics;

namespace AppKit
{
	public static class CALayerHelper
	{
		public static CGRect GetRectFromPoints (CGPoint start, CGPoint end)
		{
			return new CGRect (Math.Min (start.X, end.X),
			   Math.Min (start.Y, end.Y),
			   Math.Abs (start.X - end.X),
			   Math.Abs (start.Y - end.Y));
		}

		public static CGPoint[] ToPoints (this CGRect rect)
		{
			var points = new CGPoint[4];
			points[0] = new CGPoint (rect.Left, rect.Top);
			points[1] = new CGPoint (rect.Right, rect.Top);
			points[2] = new CGPoint (rect.Right, rect.Bottom);
			points[3] = new CGPoint (rect.Left, rect.Bottom);
			return points;
		}
		
		public static CGPoint Add (this CGPoint verts, float value)
		{
			verts.X += value;
			verts.Y += value;
			return verts;
		}

		public static CGPoint[] Add (this CGPoint[] verts, float value)
		{
			for (int i = 0; i < verts.Length; i++) {
				verts[i].Add (value);
			}
			return verts;
		}

		public static CGPoint Substract (this CGPoint vert, float value)
		{
			vert.X -= value;
			vert.Y -= value;
			return vert;
		}

		public static CGPoint[] Substract (this CGPoint[] verts, float value)
		{
			for (int i = 0; i < verts.Length; i++) {
				verts[i].Substract (value);
			}
			return verts;
		}

		public static CGPoint[] GetRectanglePoints (CGPoint cGPoint1, CGPoint cGPoint2)
		{
			var points = new CGPoint[4];
			points[0] = new CGPoint (cGPoint1.Y, cGPoint1.X);
			points[1] = new CGPoint (cGPoint2.X, cGPoint1.Y);
			points[2] = new CGPoint (cGPoint2.X, cGPoint2.Y);
			points[3] = new CGPoint (cGPoint1.X, cGPoint2.Y);
			return points;
		}

		public static CAShapeLayer GetCircleLayer (CGPoint center, float radius, CGColor borderColor)
		{
			var layer = new CAShapeLayer ();
			var path = new NSBezierPath ();
			path.AppendPathWithOvalInRect (new CGRect (center.X - radius, center.Y - radius, radius * 2, radius * 2));
			layer.Path = path.ToGCPath ();
			layer.StrokeColor = borderColor;
			layer.BorderWidth = 2;
			layer.FillColor = NSColor.Clear.CGColor;
			return layer;
		}

		public static CAShapeLayer GetSolidCircleLayer (CGPoint center, float radius, CGColor color, CGColor borderColor)
		{
			var circle = GetCircleLayer (center, radius, borderColor);
			circle.FillColor = color;
			return circle;
		}

		public static CAShapeLayer GetPolygon (CGPoint[] verts, CGColor backgroundColor, CGColor borderColor, float borderWidth = 0)
		{
			var layer = new CAShapeLayer ();
			var path = new NSBezierPath ();
			path.Append (verts);
			path.ClosePath ();

			layer.Path = path.ToGCPath ();

			layer.StrokeColor = borderColor;
			layer.BorderWidth = borderWidth;
			layer.FillColor = backgroundColor;

			return layer;
		}

		const int arrowDistance = 8;

		public static CALayer DrawSegment (CGPoint startPoint, CGPoint endPoint, float radius, CGColor backgroundColor, CGColor borderColor, float borderWidth = 0)
		{
			var layer = new CAShapeLayer ();
			var linePath = new NSBezierPath ();
			
			var y2 = endPoint.Y;
			var y1 = startPoint.Y;
			var x2 = endPoint.X;
			var x1 = startPoint.X;

			var arrowStartPoint = new CGPoint (x1, y1);
			var arrowEndPoint = new CGPoint (x2, y2);

			linePath.LineWidth = radius * 2;
			linePath.MoveTo (new CGPoint (arrowStartPoint.X, arrowStartPoint.Y));
			linePath.LineTo (new CGPoint (arrowEndPoint.X, arrowEndPoint.Y));

	
			linePath.ClosePath ();

			layer.Path = linePath.ToGCPath ();
			layer.StrokeColor = borderColor;
			layer.BorderWidth = borderWidth;
			layer.FillColor = backgroundColor;
			//var point1 = cGPoint1.Substract (radius);
			//var point2 = cGPoint2.Add (radius);

			//var points = GetRectFromPoints (point1, point2).ToPoints ();
			//var polygon = GetPolygon (points, backgroundColor, borderColor, borderWidth);

			return layer;
		}
	}
}
