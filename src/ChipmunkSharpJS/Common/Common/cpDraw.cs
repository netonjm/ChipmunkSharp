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
    public enum ChipmunkDrawFlags
    {
        e_shapeBit = 0x1,
        e_jointBit = 0x2,
        e_aabbBit = 0x4,
        e_pairBit = 0x8,
        e_centerOfMassBit = 0x10
    }

    public abstract class cpDraw
    {
        private ChipmunkDrawFlags m_drawFlags = 0x0;
        private int _PTMRatio = 1;

        public cpDraw()
        {
        }

        public cpDraw(int ptm)
        {
            _PTMRatio = ptm;
        }

        public int PTMRatio
        {
            get { return (_PTMRatio); }
        }
        /// Set the drawing flags.
        public void SetFlags(ChipmunkDrawFlags flags)
        {
            m_drawFlags = flags;
        }

        /// Get the drawing flags.
        public ChipmunkDrawFlags Flags
        {
            get { return (m_drawFlags); }
            set { m_drawFlags = value; }
        }

        /// Append flags to the current flags.
        public void AppendFlags(ChipmunkDrawFlags flags)
        {
            m_drawFlags |= flags;
        }

        /// Clear flags from the current flags.
        public void ClearFlags(ChipmunkDrawFlags flags)
        {
            m_drawFlags &= ~flags;
        }



        /// Draw a closed polygon provided in CCW order.
        public abstract void DrawPolygon(List<cpVect> vertices, int vertexCount, cpColor color);

        /// Draw a solid closed polygon provided in CCW order.
        public abstract void DrawSolidPolygon(List<cpVect> vertices, int vertexCount, cpColor color);

        /// Draw a circle.
        public abstract void DrawCircle(cpVect center, float radius, cpColor color);

        /// Draw a solid circle.
        public abstract void DrawSolidCircle(cpVect center, float radius, cpVect axis, cpColor color);

        /// Draw a line segment.
        public abstract void DrawSegment(cpVect p1, cpVect p2, cpColor color);

		/// Draw a line segment with a strokeWidth.
		public abstract void DrawSegment(cpVect p1, cpVect p2, float lineWidth, cpColor color);

        public abstract void DrawString(int x, int y, string format, params object[] objects);

        /// Draw a transform. Choose your own length scale.
        /// @param xf a transform.
        //public abstract void DrawTransform(b2Transform xf);
    }
}
