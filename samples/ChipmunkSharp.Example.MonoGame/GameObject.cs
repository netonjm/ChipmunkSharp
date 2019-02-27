using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;

namespace ChipmunkSharp.Example.Desktop
{
	class PhysicObject : GameObject
	{
		static int collCount = 0;
		ChipmunkSharp.cpBody physic;
		ChipmunkSharp.cpSpace space;
		ChipmunkSharp.cpShape shp;
		ChipmunkSharp.cpTransform trsf;

		public ChipmunkSharp.cpBody Physic {
			get { return physic; }
		}

		public ChipmunkSharp.cpShape Shape {
			get { return shp; }
		}

		public PhysicObject (Texture2D firstText, Vector2 startingPos, ChipmunkSharp.cpSpace space, bool isKinematic = false) : base (firstText, startingPos)
		{
			trsf = new cpTransform ();
			collCount++;
			physic = new cpBody (1000, cp.PHYSICS_INFINITY);

			if (isKinematic)
				physic.SetBodyType (cpBodyType.KINEMATIC);
			shp = new ChipmunkSharp.cpCircleShape (physic, 100, ChipmunkSharp.cpVect.Zero);
			shp.Active ();
			shp.SetSensor (true);
			shp.SetCollisionType (1);
			physic.SetPosition (new ChipmunkSharp.cpVect (Position.X, Position.Y));
			if (space != null) {
				space.AddBody (physic);
				space.AddShape (shp);
				this.space = space;
			}
		}

		public override void Update (GameTime gametime)
		{
			base.Update (gametime);
			physic.UpdateVelocity (space.GetGravity (), 1, 0.01f);
			physic.UpdatePosition (1);
			shp.Update (trsf);
			ChipmunkSharp.cpVect vec = physic.GetPosition ();
			Position = new Vector2 (vec.x, vec.y);
			angle = (physic.GetAngle () % 360) / 57.2958f;
			if (Position.Y > 200) {
				physic.ApplyImpulse (new ChipmunkSharp.cpVect (1, -100), new ChipmunkSharp.cpVect (0.1f, 0.1f));
			}
		}
	}

	class GameObject
	{
		public Vector2 Position {
			get { return pos; }
			set { pos = value; }
		}
		protected Vector2 pos;
		protected Texture2D text;
		protected float angle = 0;

		public GameObject (Texture2D firstText, Vector2 startingPos)
		{
			text = firstText;
			pos = startingPos;
		}

		public virtual void Draw (SpriteBatch spriteBatch)
		{
			spriteBatch.Draw (text, pos, null, Color.White, angle, Vector2.Zero, 0.25f, SpriteEffects.None, 0f);
		}

		public virtual void Draw (SpriteBatch spriteBatch, float rotation)
		{
			spriteBatch.Draw (text, pos, null, Color.White, rotation, Vector2.Zero, 0.25f, SpriteEffects.None, 0f);
		}

		public virtual void Update (GameTime gametime)
		{

		}
	}
}
