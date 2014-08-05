/* Copyright (c) 2014 by Jose Medrano (@netonjm)
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{
	[Flags]
	public enum cpDrawFlags
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

	public abstract class cpDebugDraw
	{


		private cpDrawFlags m_drawFlags = 0x0;
		private int _PTMRatio = 1;

		//protected cpSpace space;

		public cpDebugDraw()
		{
			//	this.space = space;
		}

		public cpDebugDraw(int ptm)
		{
			_PTMRatio = ptm;
		}

		public int PTMRatio
		{
			get { return (_PTMRatio); }
		}
		/// Set the drawing flags.
		public void SetFlags(cpDrawFlags flags)
		{
			m_drawFlags = flags;
		}

		/// Get the drawing flags.
		public cpDrawFlags Flags
		{
			get { return (m_drawFlags); }
			set { m_drawFlags = value; }
		}

		/// Append flags to the current flags.
		public void AppendFlags(cpDrawFlags flags)
		{
			m_drawFlags |= flags;
		}

		/// Clear flags from the current flags.
		public void RemoveFlags(cpDrawFlags flags)
		{
			m_drawFlags &= ~flags;
		}



		/// Draw a closed polygon provided in CCW order.
		public abstract void DrawPolygon(cpVect[] vertices, int vertexCount, cpColor color);

		/// Draw a solid closed polygon provided in CCW order.
		public abstract void DrawSolidPolygon(cpVect[] vertices, int vertexCount, cpColor color);

		/// Draw a circle.
		public abstract void DrawCircle(cpVect center, float radius, cpColor color);

		/// Draw a solid circle.
		public abstract void DrawSolidCircle(cpVect center, float radius, cpVect axis, cpColor color);

		/// Draw a line segment.
		public abstract void DrawSegment(cpVect p1, cpVect p2, cpColor color);

		/// Draw a line segment with a strokeWidth.
		public abstract void DrawSegment(cpVect p1, cpVect p2, float lineWidth, cpColor color);

		public abstract void DrawString(int x, int y, string format, params object[] objects);

		public abstract void DrawSpring(cpVect a, cpVect b, cpColor cpColor);

		public abstract void DrawPoint(cpVect p, float size, cpColor color);

		public abstract void DrawBB(cpBB bb, cpColor color);

	}


}
