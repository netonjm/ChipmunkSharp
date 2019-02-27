using CoreGraphics;

namespace ChipmunkSharp
{
	public static class CocoaHelpers
	{
		public static CGPoint cpVert2Point (cpVect vert)
		{
			return new CGPoint (vert.x, vert.y);
		}

		public static CGPoint[] cpVertArray2ccpArrayN (cpVect[] cpVertArray, int count)
		{
			if (count == 0)
				return null;
			CGPoint[] pPoints = new CGPoint[count];

			for (int i = 0; i < count; ++i) {
				pPoints[i].X = cpVertArray[i].x;
				pPoints[i].Y = cpVertArray[i].y;
			}
			return pPoints;
		}
	}
}
