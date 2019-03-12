namespace ChipmunkSharp.Example
{
	class Game2 : GameBase
	{
		readonly ChipmunkDemoLayer demoLayer;

		public Game2 (ChipmunkDemoLayer demoLayer)
		{
			this.demoLayer = demoLayer;
		}

		protected override void OnUpdate (long tick)
		{
			demoLayer.Update (tick);
		}
	}
}
