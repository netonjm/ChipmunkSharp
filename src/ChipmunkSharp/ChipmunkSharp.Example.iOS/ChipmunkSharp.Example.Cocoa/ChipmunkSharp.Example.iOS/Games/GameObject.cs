using CoreGraphics;
using UIKit;
namespace ChipmunkSharp.Example
{
	class GameObject : UIImageView
	{
		protected float angle = 0;

		protected void SetPosition (cpVect vector)
		{
			Frame = new CGRect (vector.x, vector.y, Frame.Width, Frame.Height);
		}

		public GameObject (UIImage firstText)
		{
			Image = firstText;
		}

		public virtual void Update (long gametime)
		{

		}
	}
}
