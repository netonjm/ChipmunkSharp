using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	public class PlayerIndex
	{

		public int Count = 0;

		public int Index = 0;

		public PlayerIndex()
		{

		}

		public PlayerIndex(int count)
		{
			Count = count;
		}

		public bool Next()
		{
			if (Index >= Count - 1)
			{
				Index = 0;
				return true;
			}
			else
			{
				Index++;
				return false;
			}
		}

		public bool Prev()
		{

			if (Index <= 0)
			{
				Index = Count - 1;
				return true;
			}
			else
			{
				Index--;
				return false;

			}
		}



		internal void Reset()
		{
			Index = 0;
			//Count = 0;
		}
	}

}
