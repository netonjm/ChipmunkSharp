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
