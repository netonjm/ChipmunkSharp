using System.Threading.Tasks;

namespace ChipmunkSharp.Example
{
	abstract class GameBase
	{
		const int DefaultLoopTime = 10;
		bool stopping;

		public bool Loop { get; private set; }

		public int DelayTime { get; private set; } = DefaultLoopTime;

		public void Start (int delayTime = DefaultLoopTime, bool loop = false)
		{
			stopping = false;
			Loop = loop;
			DelayTime = delayTime;

			//Loop
			while (!stopping) {
				OnUpdate (0);
				Task.Delay (delayTime).Wait ();
			}
		}

		protected abstract void OnUpdate (long tick);

		public async Task StartAsync (int delayTime = DefaultLoopTime, bool loop = false)
		{
			await Task.Run (() => Start (delayTime, loop));
		}

		public void Stop ()
		{
			stopping = true;
		}
	}
}
