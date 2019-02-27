using System;
using AppKit;

namespace ChipmunkSharp.Example
{
	class PhysicObject : GameObject
	{
		static Random rnd = new Random (DateTime.Now.Millisecond);

		static int collCount = 0;
		readonly cpBody physic;
		readonly cpSpace space;
		cpCircleShape shp;
		cpTransform trsf;

		public cpBody Physic {
			get { return physic; }
		}

		public cpShape Shape {
			get { return shp; }
		}

		public PhysicObject (NSImage firstText, int radius, cpSpace space, bool isKinematic = false) : base (firstText)
		{
			trsf = new cpTransform ();
			collCount++;
			physic = new cpBody (rnd.Next (500,1000), cp.PHYSICS_INFINITY);

			if (isKinematic)
				physic.SetBodyType (cpBodyType.KINEMATIC);
			shp = new cpCircleShape (physic, radius, cpVect.Zero);
			shp.Active ();
			shp.SetSensor (true);
			shp.SetCollisionType (1);
			physic.SetPosition (new cpVect ((float)Frame.Location.X, (float)Frame.Location.Y));
			if (space != null) {
				space.AddBody (physic);
				space.AddShape (shp);
				this.space = space;
			}
		}

		public void SetPosition (float x, float y)
		{
			Frame = new CoreGraphics.CGRect (x, y, shp.r, shp.r);
			physic.SetPosition (new cpVect (x, y));
		}

		public override void Update (long gametime)
		{
			physic.UpdateVelocity (space.GetGravity (), 1, 0.01f);
			physic.UpdatePosition (1);
			shp.Update (trsf);
			cpVect position = physic.GetPosition ();

			InvokeOnMainThread (() => {
				SetPosition (position);
			});

			angle = (physic.GetAngle () % 360) / 57.2958f;
			if (position.y > 200) {
				physic.ApplyImpulse (new cpVect (1, -150), new cpVect (0.1f, 0.1f));
			}
		}
	}
}
