using System;
using UIKit;
using CoreGraphics;

namespace ChipmunkSharp.Example
{
	class Game1 : GameBase
	{
		PhysicObject square;
		PhysicObject ground;
		cpSpace spc;

		public Game1 (cpSpace space)
		{
			spc = space;
			spc.SetGravity (new cpVect (0, 1f));
			spc.AddDefaultCollisionHandler ().preSolveFunc = new Func<cpArbiter, cpSpace, object, bool> ((arg1, arg2, arg3) => { Console.WriteLine ("ij"); return true; });
			spc.AddDefaultCollisionHandler ().beginFunc = new Func<cpArbiter, cpSpace, object, bool> ((arg1, arg2, arg3) => { Console.WriteLine ("ij"); return true; });
		}

		protected override void OnUpdate (long tick)
		{
			spc.Step (1 / 60);
			square.Update (tick);
			ground.Update (tick);
		}

		public void Initialize (UIView contentView)
		{
			var roseImage = new UIImage ("basketball.png");
			var squareImage = new UIImage ("football.png");
			square = new PhysicObject (squareImage, 20, spc, false);
			ground = new PhysicObject (roseImage, 20, spc, false);
			square.SetPosition (150, 0);
			ground.SetPosition (150, 0);

			contentView.AddSubview (square);
			contentView.AddSubview (ground);
		}
	}
}
