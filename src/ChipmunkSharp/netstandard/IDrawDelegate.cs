
using System;

namespace ChipmunkSharp
{
	[Flags]
	public enum PhysicsDrawFlags
	{

		None = 1 << 0,
		/// <summary>
		/// Draw shapes.
		/// </summary>
		Shapes = 1 << 1,

		/// <summary>
		/// Draw joint connections.
		/// </summary>
		Joints = 1 << 2,

		/// <summary>
		/// Draw contact points.
		/// </summary>
		ContactPoints = 1 << 3,

		/// <summary>
		/// Draw polygon BB.
		/// </summary>
		BB = 1 << 4,

		/// <summary>
		/// Draw All connections.
		/// </summary>
		All = 1 << 10,
	}

	public interface IDrawDelegate
	{
		void DrawCircle (cpVect center, float radius, cpColor color);
		void DrawSolidCircle (cpVect center, float radius, cpColor color);
		void DrawCircle (cpVect center, float radius, float angle, int segments, cpColor color);
		void DrawDot (cpVect pos, float radius, cpColor color);
		void DrawPolygon (cpVect[] verts, int count, cpColor fillColor, float borderWidth, cpColor borderColor);
		void DrawRect (Rectangle rect, cpColor color);
		void DrawSegment (cpVect from, cpVect to, float radius, cpColor color);
		void Draw (cpPolyShape poly, cpColor color);
		void Draw (cpBB bb);
		void Draw (cpBB bb, cpColor color);
		void Draw (cpContact contact);
		void Draw (cpCircleShape circle, cpColor color);
		void Draw (cpSegmentShape seg, cpColor color);
		void DrawFatSegment (cpVect ta, cpVect tb, float r, cpColor color);
		void Draw (cpVect point);
		void Draw (cpVect point, cpColor color);
		void Draw (cpVect point, float radius);
		void Draw (cpVect point, float radius, cpColor color);

		//DRAW CONSTRAINT
		void Draw (cpDampedRotarySpring constraint);
		void Draw (cpDampedSpring constraint);
		void Draw (cpSimpleMotor cpSimpleMotor);
		void Draw (cpGrooveJoint constraint);
		void Draw (cpPivotJoint constraint);
		void Draw (cpSlideJoint constraint);
		void Draw (cpPinJoint constraint);
	}
}
