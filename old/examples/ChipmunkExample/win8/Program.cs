using CocosSharp;
using System;

namespace ChipmunkJSExample
{
	/// <summary>
	/// The main class.
	/// </summary>
	public static class Program
	{
		/// <summary>
		/// The main entry point for the application.
		/// </summary>
		static void Main()
		{
			CCApplication application = new CCApplication(true);
			application.ApplicationDelegate = new AppDelegate();

			application.StartGame();
		}
	}
}
