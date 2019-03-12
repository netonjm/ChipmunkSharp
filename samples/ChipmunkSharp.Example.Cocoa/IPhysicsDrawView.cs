using CoreGraphics;

namespace ChipmunkSharp
{
	public interface IPhysicsDrawView
	{
		cpBody Body { get; set; }
		CGPoint Position { get; set; }
		PhysicsDrawFlags Flags { get; set; }
		CocoaDrawDelegate drawDelegate { get; set; }
		void DebugDraw ();
		void AppendFlags (params PhysicsDrawFlags[] flags);
		void ClearFlags (params PhysicsDrawFlags[] flags);

		void DrawShape (cpShape shape);
	}
}
