using System;
using MonoMac;
using MonoMac.AppKit;
using MonoMac.Foundation;
using CocosSharp;

namespace ChipmunkExample
{
	class Program : NSApplicationDelegate 
	{
		//Game1 game;

		static void Main (string[] args)
		{
			NSApplication.Init ();

			using (var p = new NSAutoreleasePool()) 
			{
				NSApplication.SharedApplication.Delegate = new Program();
				NSApplication.Main(args);
			}

		}

		public override void FinishedLaunching (NSObject notification)
		{
			var application = new CCApplication();
			application.ApplicationDelegate = new AppDelegate();
			application.StartGame();
		}

		public override bool ApplicationShouldTerminateAfterLastWindowClosed (NSApplication sender)
		{
			return true;
		}
	}
}

