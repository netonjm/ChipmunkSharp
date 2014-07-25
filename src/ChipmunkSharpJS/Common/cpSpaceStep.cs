/* Copyright (c) 2007 Scott Lembcke ported by Jose Medrano (@netonjm)
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 */

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{

	public class cpContactBufferHeader
	{
		public int stamp;
		public cpContactBufferHeader next;
		public int numContacts;
	};


	public partial class cpSpace
	{


		// **** Post Step Callback Functions
		public void addPostStepCallback(Action func)
		{
			cp.assertSoft(this.IsLocked,
		 "Adding a post-step callback when the space is not locked is unnecessary. " +
		 "Post-step callbacks will not called until the end of the next call to cpSpaceStep() or the next query.");

			this.postStepCallbacks.Add(func);
		}

		public void runPostStepCallbacks()
		{
			// Don't cache length because post step callbacks may add more post step callbacks
			// directly or indirectly.
			for (var i = 0; i < this.postStepCallbacks.Count; i++)
				this.postStepCallbacks[i]();
			this.postStepCallbacks.Clear();

		}

		//MARK: Locking Functions

		public void Lock()
		{
			locked++;
		}

		public void Unlock(bool runPostStep)
		{

			this.locked--;
			cp.assertSoft(this.locked >= 0, "Internal Error: Space lock underflow.");

			if (this.locked == 0 && runPostStep)
			{
				var waking = this.rousedBodies;
				for (var i = 0; i < waking.Count; i++)
					this.activateBody(waking[i]);

				waking.Clear();

				this.runPostStepCallbacks();
			}

		}























		//static bool QueryReject(cpShape a, cpShape b)
		//{
		//    return (
		//        // BBoxes must overlap
		//        !a.bb.Intersects(b.bb)
		//        // Don't collide shapes attached to the same body.
		//        || a.body == b.body
		//        // Don't collide objects in the same non-zero group
		//        || (a.group != 0 && a.group == b.group)
		//        // Don't collide objects that don't share at least on layer.
		//        || !(a.layers != 0 & b.layers != 0)
		//        // Don't collide infinite mass objects
		//        || (a.body.Mass == cpEnvironment.Infinity && b.body.Mass == cpEnvironment.Infinity)
		//    );
		//}

		//// Callback from the spatial hash.
		//public List<ContactPoint> collideShapes(cpShape a, cpShape b)
		//{
		//    cpEnvironment.assertWarn((a as ICollisionShape).collisionCode <= (b as ICollisionShape).collisionCode, "Collided shapes must be sorted by type");
		//    return (a as ICollisionShape).collisionTable[(b as ICollisionShape).collisionCode](a, b);
		//    //return null;

		//}

		//private void UpdateFunc(cpShape shape)
		//{
		//    var body = shape.body;
		//    shape.update(body.Position, body.Rotation);
		//}


	}
}

