using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	class smoothLayer : ChipmunkDemoLayer
	{




		protected override void AddedToScene()
		{
			base.AddedToScene();



			Schedule();
		}

		public override void Update(float dt)
		{
			base.Update(dt);

		}



	}
}
