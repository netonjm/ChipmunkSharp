//
// RandomHelper.cs
//
// Author:
//       Jose Medrano <josmed@microsoft.com>
//
// Copyright (c) 2015
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{
	public class RandomHelper
	{
		public static Random random = new Random(DateTime.Now.Millisecond);

		public static int RAND_MAX = 0x7fff;

		public static float next(float min, float max)
		{
			return (float)((max - min) * random.NextDouble() + min);
		}

		public static int rand()
		{
			return random.Next();
		}

		public static float randBell(float scale)
		{
			return (float)((scale) * (-(frand(.5f) + frand(.5f) + frand(.5f))));
		}


		public static float frand() //HACK
		{
			//float tmp = ((rand.Nextfloat() *  f) / ((float) (/*(uint)~0*/ 0xFFFFFFFF /*or is it -1 :P */)));
			//return tmp < 0 ? (-tmp) : tmp;
			return (float)(random.NextDouble());
		}

		/// <summary>
		/// This is bit spooky conversion of C -> C#...
		/// </summary>
		public static float frand(float f) //HACK
		{
			//float tmp = ((rand.Nextfloat() *  f) / ((float) (/*(uint)~0*/ 0xFFFFFFFF /*or is it -1 :P */)));
			//return tmp < 0 ? (-tmp) : tmp;
			return frand() * f;
		}

		public static float FastDistance2D(float x, float y)
		{
			// this function computes the distance from 0,0 to x,y with ~3.5% error
			float mn;
			// first compute the absolute value of x,y
			x = (x < 0.0f) ? -x : x;
			y = (y < 0.0f) ? -y : y;

			// compute the minimum of x,y
			mn = x < y ? x : y;

			// return the distance
			//return(x+y-(mn*0.5f)-(mn*0.25f)+(mn*0.0625f));
			return x + y - mn * .6875f;

		}
	}
}
