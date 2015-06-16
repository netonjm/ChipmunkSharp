using System.Reflection;
using Microsoft.Xna.Framework;
using CocosDenshion;
using CocosSharp;

namespace ChipmunkExample
{
	public class AppDelegate : CCApplicationDelegate
	{
		float preferredWidth;
		float preferredHeight;

		static CCWindow sharedWindow;
		public static CCSize DefaultResolution;

		public static CCWindow SharedWindow
		{
			get { return sharedWindow; }
		}

		public override void ApplicationDidFinishLaunching(CCApplication application, CCWindow mainWindow)
		{
			sharedWindow = mainWindow;
			preferredWidth = application.MainWindow.WindowSizeInPixels.Width;
			preferredHeight = application.MainWindow.WindowSizeInPixels.Height;

			DefaultResolution = new CCSize(preferredWidth, preferredHeight);
			application.ContentRootDirectory = "Content";
			application.ContentSearchPaths.Add("SD");

			CCScene scene = ChipmunkDemoLayer.ActualScene;
			mainWindow.RunWithScene(scene);
		}
	}
}