using System.Collections.Generic;
using System.Text;
using System;
using Microsoft.Xna.Framework.Graphics;

using Microsoft.Xna.Framework;
using CocosSharp;


namespace ChipmunkSharp
{

	public class CCChipmunkDebugDraw : cpDebugDraw
	{


#if WINDOWS_PHONE || OUYA
        public const int CircleSegments = 16;
#else
		public const int CircleSegments = 32;
#endif
		internal Color TextColor = Color.White;

		CCPrimitiveBatch primitiveBatch;
		SpriteFont spriteFont;
		List<StringData> stringData;
		StringBuilder stringBuilder;

		cpVect[] springPoints = new cpVect[]{
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

		#region Structs

		struct StringData
		{
			public object[] Args;
			public Color Color;
			public string S;
			public int X, Y;

			public StringData(int x, int y, string s, object[] args, Color color)
			{
				X = x;
				Y = y;
				S = s;
				Args = args;
				Color = color;
			}
		}

		#endregion Structs

		#region Constructors

		public CCChipmunkDebugDraw(string spriteFontName)
		{
			primitiveBatch = new CCPrimitiveBatch(CCDrawManager.SharedDrawManager);
			spriteFont = CCContentManager.SharedContentManager.Load<SpriteFont>(spriteFontName);
			stringData = new List<StringData>();
			stringBuilder = new StringBuilder();
		}

		#endregion Constructors


		public override void DrawBB(cpBB bb, cpColor color)
		{
			if (!primitiveBatch.IsReady())
			{
				throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
			}

			DrawPolygon(new cpVect[] { 
 
						new cpVect(bb.r, bb.b),
					new cpVect(bb.r, bb.t),
					new cpVect(bb.l, bb.t),
					new cpVect(bb.l, bb.b)
				
				}, 4, color);
			return;
		}

		public override void DrawPolygon(cpVect[] vertices, int vertexCount, cpColor color)
		{
			CCColor4B fillColor = color.ToCCColor4B();

			if (!primitiveBatch.IsReady())
			{
				throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
			}

			for (int i = 0; i < vertexCount - 1; i++)
			{

				primitiveBatch.AddVertex(vertices[i].ToCCVector2(), fillColor, PrimitiveType.LineList);
				primitiveBatch.AddVertex(vertices[i + 1].ToCCVector2(), fillColor, PrimitiveType.LineList);
			}

			primitiveBatch.AddVertex(vertices[vertexCount - 1].ToCCVector2(), fillColor, PrimitiveType.LineList);
			primitiveBatch.AddVertex(vertices[0].ToCCVector2(), fillColor, PrimitiveType.LineList);
		}

		public override void DrawSolidPolygon(cpVect[] vertices, int vertexCount, cpColor color)
		{


			if (!primitiveBatch.IsReady())
			{
				throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
			}

			if (vertexCount == 2)
			{
				DrawPolygon(vertices, vertexCount, color);
				return;
			}

			CCColor4B colorFill = color.ToCCColor4B() * 0.5f;

			for (int i = 1; i < vertexCount - 1; i++)
			{
				CCVector2 vertice0 = vertices[0].ToCCVector2();
				primitiveBatch.AddVertex(ref vertice0, colorFill, PrimitiveType.TriangleList);
				primitiveBatch.AddVertex(vertices[i].ToCCVector2(), colorFill, PrimitiveType.TriangleList);
				primitiveBatch.AddVertex(vertices[i + 1].ToCCVector2(), colorFill, PrimitiveType.TriangleList);
			}

			DrawPolygon(vertices, vertexCount, color);
		}

		public override void DrawCircle(cpVect center, float radius, cpColor color)
		{
			if (!primitiveBatch.IsReady())
			{
				throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
			}
			const float increment = cp.M_PI * 2.0f / CircleSegments;
			float theta = 0.0f;

			var col = color;
			var colorOutline = col.ToCCColor4B();

			CCVector2 centr = center.ToCCVector2();
			CCVector2 thetaV = CCVector2.Zero;


			for (int i = 0, count = CircleSegments; i < count; i++)
			{

				thetaV.X = cp.cpfcos(theta);
				thetaV.Y = cp.cpfsin(theta);
				CCVector2 v1 = centr + Convert.ToSingle(radius) * thetaV;

				thetaV.X = cp.cpfcos(theta + increment);
				thetaV.Y = cp.cpfsin(theta + increment);
				CCVector2 v2 = centr +
							 Convert.ToSingle(radius) *
							 thetaV;

				primitiveBatch.AddVertex(ref v1, colorOutline, PrimitiveType.LineList);
				primitiveBatch.AddVertex(ref v2, colorOutline, PrimitiveType.LineList);

				theta += increment;
			}
		}

		public override void DrawSolidCircle(cpVect center, float radius, cpVect axis, cpColor color)
		{
			float segments = (10 * cp.cpfsqrt(radius));  //<- Let's try to guess at # segments for a reasonable smoothness

			var colorOutline = color.ToCCColor4B();
			var colorFill = colorOutline * 0.5f;
			var centr = center.ToCCVector2();

			if (!primitiveBatch.IsReady())
			{
				throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
			}

			float increment = cp.M_PI * 2.0f / segments;
			float theta = 0.0f;

			CCVector2 thetaV = new CCVector2((float)Math.Cos(theta), (float)Math.Sin(theta));
			CCVector2 v0 = center.ToCCVector2() + (float)radius * thetaV;
			theta += increment;

			var v1 = CCVector2.Zero;
			var v2 = CCVector2.Zero;

			for (int i = 0, count = (int)segments; i < count; i++)
			{
				thetaV.X = (float)Math.Cos(theta);
				thetaV.Y = (float)Math.Sin(theta);
				v1 = centr + Convert.ToSingle(radius) * thetaV;

				thetaV.X = (float)Math.Cos(theta + increment);
				thetaV.Y = (float)Math.Sin(theta + increment);
				v2 = centr +
						 Convert.ToSingle(radius) * thetaV;

				primitiveBatch.AddVertex(ref v0, colorFill, PrimitiveType.TriangleList);
				primitiveBatch.AddVertex(ref v1, colorFill, PrimitiveType.TriangleList);
				primitiveBatch.AddVertex(ref v2, colorFill, PrimitiveType.TriangleList);

				primitiveBatch.AddVertex(ref v1, colorOutline, PrimitiveType.LineList);
				primitiveBatch.AddVertex(ref v2, colorOutline, PrimitiveType.LineList);

				theta += increment;
			}

			DrawSegment(center, center + axis * radius, color);
		}

		public void DrawSegment(CCPoint from, CCPoint to, float radius, cpColor color)
		{
			var colorFill = color.ToCCColor4B();

			if (!primitiveBatch.IsReady())
			{
				throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
			}
			var a = from;
			var b = to;

			var n = CCPoint.Normalize(CCPoint.PerpendicularCCW(a - b));

			var nw = n * (float)radius;
			var v0 = b - nw;
			var v1 = b + nw;
			var v2 = a - nw;
			var v3 = a + nw;

			// Triangles from beginning to end
			primitiveBatch.AddVertex(v1, colorFill, PrimitiveType.TriangleList);
			primitiveBatch.AddVertex(v2, colorFill, PrimitiveType.TriangleList);
			primitiveBatch.AddVertex(v0, colorFill, PrimitiveType.TriangleList);

			primitiveBatch.AddVertex(v1, colorFill, PrimitiveType.TriangleList);
			primitiveBatch.AddVertex(v2, colorFill, PrimitiveType.TriangleList);
			primitiveBatch.AddVertex(v3, colorFill, PrimitiveType.TriangleList);

		}

		public override void DrawSegment(cpVect p1, cpVect p2, cpColor color)
		{

			if (!primitiveBatch.IsReady())
			{
				throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
			}

			DrawSegment(
				new CCPoint((float)p1.x, (float)p1.y),
				new CCPoint((float)p2.x, (float)p2.y),
				1, color);
		}

		public override void DrawSegment(cpVect p1, cpVect p2, float lineWidth, cpColor color)
		{

			if (!primitiveBatch.IsReady())
			{
				throw new InvalidOperationException("BeginCustomDraw must be called before drawing anything.");
			}

			DrawSegment(new CCPoint((float)p1.x, (float)p1.y), new CCPoint((float)p2.x, (float)p2.y), lineWidth, color);
		}

		public override void DrawString(int x, int y, string format, params object[] objects)
		{
			stringData.Add(new StringData(x, y, format, objects, Color.White));
		}



		public override void DrawPoint(cpVect p, float size, cpColor color)
		{

			float hs = (float)size / 2.0f;

			cpVect[] verts = new cpVect[]{

			p + new cpVect(-hs, -hs),
			p + new cpVect(hs, -hs),
			p + new cpVect(hs, hs),
			p + new cpVect(-hs, hs)
		};
			DrawSolidPolygon(verts, 4, color);
		}

		public bool Begin()
		{
			primitiveBatch.Begin();
			return true;
		}

		public void End()
		{
			primitiveBatch.End();

			var _batch = CCDrawManager.SharedDrawManager.SpriteBatch;

			_batch.Begin(SpriteSortMode.Deferred, BlendState.AlphaBlend);

			for (int i = 0; i < stringData.Count; i++)
			{
				stringBuilder.Length = 0;
				stringBuilder.AppendFormat(stringData[i].S, stringData[i].Args);

				_batch.DrawString(spriteFont,
					stringBuilder,
					new Vector2(stringData[i].X, stringData[i].Y),
					stringData[i].Color);
			}

			_batch.End();

			stringData.Clear();
		}

		public override void DrawSpring(cpVect a, cpVect b, cpColor cpColor)
		{
			cpVect prevPoint = new cpVect(a.x, a.y);
			var delta = cpVect.cpvsub(b, a);
			var len = cpVect.cpvlength(delta);
			var rot = cpVect.cpvmult(delta, 1f / len);
			cpVect tmpPoint;
			for (var i = 1; i < springPoints.Length; i++)
			{
				tmpPoint = cpVect.cpvadd(a, cpVect.cpvrotate(new cpVect(springPoints[i].x * len, springPoints[i].y * cp.scale), rot));
				DrawSegment(prevPoint, tmpPoint, cpColor);
				prevPoint = tmpPoint;
			}
		}



	}

}