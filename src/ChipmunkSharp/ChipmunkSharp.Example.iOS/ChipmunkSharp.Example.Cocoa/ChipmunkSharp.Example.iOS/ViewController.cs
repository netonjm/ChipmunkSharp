using System;

using UIKit;

namespace ChipmunkSharp.Example.iOS
{
	public partial class ViewController : UIViewController
	{
		protected ViewController (IntPtr handle) : base (handle)
		{
			// Note: this .ctor should not contain any initialization logic.
		}

		public override void ViewDidLoad ()
		{
			base.ViewDidLoad ();
			// Perform any additional setup after loading the view, typically from a nib.

			var space = new cpSpace {
				CollisionEnabled = true
			};

			#region	Scene

			var example = new Game1 (space);
			example.Initialize (View);
			example.StartAsync (loop: true);

			#endregion

		}

		public override void DidReceiveMemoryWarning ()
		{
			base.DidReceiveMemoryWarning ();
			// Release any cached data, images, etc that aren't in use.
		}
	}
}
