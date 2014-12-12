using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace CocosSharp
{


    public static class Extensions
    {

        public static Vector2 ToVector(this CCPoint sender)
        {
            return new Vector2(sender.X, sender.Y);
        }


        public static CCPoint ToCCPoint(this Vector2 sender)
        {
            return new CCPoint(sender.X, sender.Y);
        }
    }
}
