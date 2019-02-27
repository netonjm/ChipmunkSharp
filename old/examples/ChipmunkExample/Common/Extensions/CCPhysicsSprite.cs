
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using ChipmunkSharp;

namespace CocosSharp
{
	public class CCPhysicsSprite : CCSprite
	{

		cpBody _body;

		private bool _ignoreBodyRotation;

		private float m_fPTMRatio;

		public float PTMRatio
		{
			get { return m_fPTMRatio; }
			set { m_fPTMRatio = value; }
		}

		public bool IgnoreBodyRotation
		{
			get { return _ignoreBodyRotation; }
			set { _ignoreBodyRotation = value; }
		}

		public cpBody Body
		{
			get
			{
				return _body;
			}
			set
			{
				_body = value;
			}
		}

		public override CCPoint Position
		{
			get
			{
				if (_body == null)
					return base.Position;
				else
				{
					var position = _body.GetPosition();
					return new CCPoint(position.x, position.y);
				}

			}
			set
			{
				if ( _body == null)
					base.Position = value;
				else
					_body.SetPosition(new cpVect(value.X, value.Y));

			}
		}

		public override float Rotation
		{
			set
			{
				if (IgnoreBodyRotation || _body == null)
				{
					base.Rotation = value;
				}
				else
				{
					_body.SetAngle(-CCMathHelper.ToRadians(value));
				}
			}

		}

		public override float RotationY
		{
			get
			{
				return (IgnoreBodyRotation || _body == null ? base.RotationY : -CCMathHelper.ToDegrees((float)_body.GetAngle()));
			}
			set
			{

				if (IgnoreBodyRotation)
				{
					base.RotationY = value;
				}
				else
				{
					_body.SetAngle(-CCMathHelper.ToRadians(value));
				}


			}
		}

		public override float RotationX
		{
			get
			{
				return (IgnoreBodyRotation || _body == null ? base.RotationX : -CCMathHelper.ToDegrees(_body.GetAngle()));


			}
			set
			{
				if (IgnoreBodyRotation)
				{
					base.RotationX = value;
				}
				else
				{
					_body.SetAngle(-CCMathHelper.ToRadians(value));
				}

			}
		}

		public CCPhysicsSprite()
			: base()
		{
            IgnoreAnchorPointForPosition = true;
		}

        //public override void Visit()
        //{
        //    base.AdditionalTransform = NodeToBodyTransform();
        //    base.Visit();
        //}

        // returns the transform matrix according the Chipmunk Body values
        public CCAffineTransform NodeToBodyTransform()
        {

            var angle = CCPoint.ForAngle(-CCMathHelper.ToRadians(RotationX));

            cpVect rot = (IgnoreBodyRotation ? new cpVect(angle.X, angle.Y) : _body.GetRotation()); //TODO: CHECK ROT
            double x = _body.GetPosition().x + (double)rot.x * -AnchorPointInPoints.X - (double)rot.y * (-AnchorPointInPoints.Y);
            double y = _body.GetPosition().y + (double)rot.y * -AnchorPointInPoints.X + (double)rot.x * (-AnchorPointInPoints.Y);

            return new CCAffineTransform((float)rot.x, (float)rot.y, (float)-rot.y, (float)rot.x, (float)x, (float)y);
        }

		public CCPhysicsSprite(CCTexture2D pTexture)
			: base(pTexture)
		{

		}

		public CCPhysicsSprite(CCTexture2D pTexture, CCRect rect)
			: base(pTexture, rect)
		{
            IgnoreAnchorPointForPosition = true;
		}

		public CCPhysicsSprite(CCSpriteFrame pSpriteFrame)
			: base(pSpriteFrame)
		{
            IgnoreAnchorPointForPosition = true;
		}

		public CCPhysicsSprite(string pszSpriteFrameName)
			: base(pszSpriteFrameName)
		{
            IgnoreAnchorPointForPosition = true;
		}

		public CCPhysicsSprite(string pszFileName, CCRect rect)
			: base(pszFileName, rect)
		{
            IgnoreAnchorPointForPosition = true;
		}

		public bool IsDirty()
		{
			return true;
		}

	}
}
