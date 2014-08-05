/* Copyright (c) 2007 Scott Lembcke ported by Jose Medrano (@netonjm)
  
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
	public struct cpColor
	{

		public cpColor(float xr, float xg, float xb) { r = xr; g = xg; b = xb; a = 255; }
		public cpColor(float xr, float xg, float xb, float ba) { r = xr; g = xg; b = xb; a = ba; }

		public cpColor(cpColor color)
		{
			r = color.r; g = color.g; b = color.b; a = color.a;
		}

		public void Set(cpColor color)
		{
			Set(color.r, color.g, color.b, color.a);
		}


		public void Set(float ri, float gi, float bi, float ba) { r = ri; g = gi; b = bi; a = ba; }

		public float r, g, b, a;

		public static cpColor Red { get { return new cpColor(255, 0, 0); } }
		public static cpColor Green { get { return new cpColor(0, 255, 0); } }
		public static cpColor Blue { get { return new cpColor(0, 0, 255); } }
		public static cpColor Black { get { return new cpColor(0, 0, 0); } }

		public static cpColor White { get { return new cpColor(255, 255, 255); } }

		public static cpColor Grey { get { return new cpColor(84, 84, 84); } }
		public static cpColor DarkGrey { get { return new cpColor(50, 50, 50); } }

		public static cpColor CyanBlue { get { return new cpColor(170, 212, 255); } }

		public static cpColor WhiteGreen { get { return new cpColor(212, 255, 212); } }

		public static cpColor WhiteRed { get { return new cpColor(255, 127, 127); } }

	}
}
