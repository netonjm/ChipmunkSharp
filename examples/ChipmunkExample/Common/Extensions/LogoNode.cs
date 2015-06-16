using CocosSharp;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkExample
{
	public class LogoNode : CCNode
	{
		public CCParticleMeteor ParticleStay;
		//public CCParticleMeteor ParticleSelected;

		CCSprite xamarinLogo;

		CCParticleMeteor CreateMeteor(CCPoint position)
		{
			CCParticleMeteor dev = new CCParticleMeteor(position)
			{
				Angle = 90,
				AngleVar = 360,
				BlendFunc = new CCBlendFunc(1, 772),
				EmissionRate = 50,
				Gravity = new CCPoint(0, 200),
				Life = 1f,
				Speed = 15,
				SpeedVar = 5,
				StartSize = 100
			};
			return dev;
		}

		CCParticleMeteor CreateGalaxy(CCPoint position)
		{
			CCParticleMeteor dev = new CCParticleMeteor(position)
			{
				Angle = 90,
				AngleVar = 360,
				BlendFunc = new CCBlendFunc(772, 772),
				EmissionRate = 50,
				Life = 0.5f,
				Speed = 60,
				SpeedVar = 10,
				StartSize = 100,
				TangentialAccel = 80
			};
			return dev;
		}

		public LogoNode()
		{

			xamarinLogo = new CCSprite("xamarin-logo-no-shadow.png");
			xamarinLogo.Scale = .4f;
			AddChild(xamarinLogo, 2);

			ContentSize = xamarinLogo.ContentSize;

			xamarinLogo.Position = new CCPoint(
				ContentSize.Width * xamarinLogo.ScaleX * ScaleX * .5f,
				ContentSize.Height * xamarinLogo.ScaleY * ScaleY * .5f);

			ParticleStay = CreateMeteor(new CCPoint(100, 100));
			ParticleStay.Scale = .55f;
			AddChild(ParticleStay, 1);

			ParticleStay.Position = xamarinLogo.Position;

		}

		private bool _isHover;

		public bool IsHover
		{
			get { return _isHover; }
			set
			{

				if (!_isHover && value)
					ExecAnimationMouseIn();

				if (_isHover && !value)
					ExecAnimationMouseOut();

				_isHover = value;

			}
		}

		public void ExecAnimationMouseIn()
		{
			ParticleStay.Gravity = new CCPoint(0, 0);
			ParticleStay.Life = 3f;
		}

		public void ExecAnimationMouseOut()
		{
			ParticleStay.Gravity = new CCPoint(0, 200);
			ParticleStay.Life = 1f;
		}


		public CCPoint InitialPosition { get; set; }

		public CCPoint MoveOffset = CCPoint.Zero;


		public bool IsMoving { get; set; }
	}
}
