using AppKit;
using CoreGraphics;

namespace ChipmunkSharp.Example
{
	class GameObject : NSImageView
	{
		protected float angle = 0;

		public override bool IsFlipped => true;

		protected void SetPosition (cpVect vector)
		{
			Frame = new CGRect (vector.x, vector.y, Frame.Width, Frame.Height);
		}

		public GameObject (NSImage firstText)
		{
			Image = firstText;
		}

		public virtual void Update (long gametime)
		{

		}
	}
}
