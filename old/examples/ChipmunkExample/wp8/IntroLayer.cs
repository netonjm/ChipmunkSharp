using System;
using Microsoft.Xna.Framework;
using CocosSharp;

namespace Chipmunkv7Example
{
	public class IntroLayer : CCLayerColor
	{
		public IntroLayer()
		{

			// create and initialize a Label
			//var label = new CCLabelTtf("Hello Cocos2D-XNA", "MarkerFelt", 22);

			// position the label on the center of the screen
			//label.Position = Window.WindowSizeInPixels.Center;

			// add the label as a child to this Layer
			//AddChild(label);

			// setup our color for the background
			Color = CCColor3B.Blue;
			Opacity = 255;

		}

		

	}
}

