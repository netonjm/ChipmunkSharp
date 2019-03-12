using AppKit;
using CoreGraphics;

namespace ChipmunkSharp.Example.Cocoa
{
	static class MainClass
	{
		class SpaceView : NSView
		{
			public override bool IsFlipped => true; 
		}

		static void Main (string[] args)
		{
			NSApplication.Init ();
			NSApplication.SharedApplication.ActivationPolicy = NSApplicationActivationPolicy.Regular;

			var xPos = NSScreen.MainScreen.Frame.Width / 2;// NSWidth([[window screen] frame])/ 2 - NSWidth([window frame])/ 2;
			var yPos = NSScreen.MainScreen.Frame.Height / 2; // NSHeight([[window screen] frame])/ 2 - NSHeight([window frame])/ 2;
			var mainWindow = new NSWindow (new CGRect (xPos, yPos, 600, 368), NSWindowStyle.Titled | NSWindowStyle.Resizable | NSWindowStyle.Closable, NSBackingStore.Buffered, false) {
				ContentView = new SpaceView ()
			};

			var space = new cpSpace {
				CollisionEnabled = true
			};

			var drawDelegate = new CocoaDrawDelegate ();
			var drawView = new PhysicsDrawView (drawDelegate, space);
			drawView.AppendFlags (PhysicsDrawFlags.All);

			var demoLayer = new BallLayer (drawView, space);

			mainWindow.ContentView = demoLayer;

			#region	Scene

			var example = new Game2 (demoLayer);
			//example.Initialize (mainWindow.ContentView);
			example.StartAsync (loop: true);

			#endregion

			mainWindow.Title = "Example Debug Xamarin.Mac";

			//mainWindow.MakeKeyWindow();
			mainWindow.MakeKeyAndOrderFront (null);
			NSApplication.SharedApplication.ActivateIgnoringOtherApps (true);
			NSApplication.SharedApplication.Run ();
			//mainWindow.Dispose();
		}
	}
}
