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

        /*public b2Color() { _r = 0f; _g = 0f; _b = 0f; }*/
        public cpColor(float xr, float xg, float xb) { r = xr; g = xg; b = xb; }
        public void Set(float ri, float gi, float bi) { r = ri; g = gi; b = bi; }

        public float r, g, b;

        public static cpColor Red { get { return new cpColor(255, 0, 0); } }
        public static cpColor Green { get { return new cpColor(0, 255, 0); } }
        public static cpColor Blue { get { return new cpColor(0, 0, 255); } }

        //public static bool operator !=(cpColor p1, cpColor p2);
        //public static cpColor operator *(cpColor p1, cpColor p2);
        //public static cpColor operator *(cpColor p1, float scale);
        //public static cpColor operator *(float scale, cpColor p1);
        //public static cpColor operator /(cpColor p1, float div);
        //public static bool operator ==(cpColor p1, cpColor p2);
        // public static implicit operator Color(cpColor point);

        //public bool Equals(cpColor other);
        //public override bool Equals(object obj);
        //public override int GetHashCode();
        //public static cpColor Lerp(cpColor value1, cpColor value2, float amount);
        //public static cpColor Parse(string s);
        //public override string ToString();

    }
}
