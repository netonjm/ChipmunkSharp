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
using ChipmunkSharp.Shapes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace ChipmunkSharp
{

	public partial class cpSpace
	{


		#region OBSOLETE QUERY METHODS

		/// Query the space at a point and call @c func for each shape found.
		[Obsolete("This method was obsolete from Chipmunk JS")]
		public void pointQuery(cpVect point, int layers, int group, Action<cpShape> func)
		{

			if (point == null)
				return;

			var helper = new Action<cpShape>(shape =>
			{
				if (
					!(shape.filter.group != 0
					&& group == shape.filter.group)
					&& (layers != 0)
					&& shape.NearestPointQuery(point) != null
				)
				{
					func(shape);
				}
			});

			var bb = new cpBB(point.x, point.y, point.x, point.y);
			Lock();
			{
				this.activeShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
				this.staticShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
			} Unlock(true);
		}

		/// Query the space at a point and return the first shape found. Returns null if no shapes were found.
		[Obsolete("This method was obsolete from Chipmunk JS")]
		public cpShape pointQueryFirst(cpVect point, int layers, int group, Action<cpShape> func)
		{
			cpShape outShape = null;

			this.pointQuery(point, layers, group, new Action<cpShape>(shape =>
			{
				if (!shape.sensor)
					outShape = shape;
			}));
			return outShape;
		}

		/// Query a space for any shapes overlapping the given shape and call @c func for each shape found.
		[Obsolete("This method was obsolete from Chipmunk JS")]
		public bool shapeQuery(cpShape shape, Action<cpShape, List<ContactPoint>> func)
		{

			cpBody body = shape.body;

			//var bb = (body ? shape.update(body.p, body.rot) : shape.bb);
			if (body != null)
			{
				shape.Update(body.Position, body.Rotation);
			}

			var bb = new cpBB(shape.bb_l, shape.bb_b, shape.bb_r, shape.bb_t);

			//shapeQueryContext context = {func, data, false};
			bool anyCollision = false;

			Action<cpShape> helper = (b) =>
			{

				var a = shape;
				// Reject any of the simple cases
				if (
					(a.filter.group != 0 && a.filter.group == b.filter.group) ||

					a == b
				) return;


				List<ContactPoint> contacts = new List<ContactPoint>();

				// Shape 'a' should have the lower shape type. (required by collideShapes() )
				if ((a as ICollisionShape).CollisionCode <= (b as ICollisionShape).CollisionCode)
				{
					contacts = collideShapes(a, b);
				}
				else
				{
					contacts = collideShapes(b, a);
					List<ContactPoint> contactsModified = new List<ContactPoint>();
					for (var i = 0; i < contacts.Count; i++)
					{
						ContactPoint contacto = contacts[i];
						contacto.n = cpVect.cpvneg(contacto.n);
						contactsModified.Add(contacto);
					}
					contacts = contactsModified;
				}

				if (contacts.Count > 0)
				{
					anyCollision = !(a.sensor || b.sensor);

					if (func != null)
					{
						//List<ContactPoint> set = new List<ContactPoint>();
						//ContactPoint tmp;
						//for (var i = 0; i < contacts.Count; i++)
						//{
						//    tmp = new ContactPoint(contacts[i].p, contacts[i].n, contacts[i].dist, 0); // contacts[i].p, contacts[i].n, contacts[i].dist);
						//    set.Add(tmp);
						//}

						func(b, contacts);
					}
				}


			};

			Lock();
			{
				this.activeShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
				this.staticShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
			}
			Unlock(true);

			return anyCollision;
		}

		[Obsolete("This method was obsolete from Chipmunk JS")]
		public void segmentQuery(cpVect start, cpVect end, int layers, int group, Action<cpShape, float, cpVect> func)
		{
			var helper = new Func<object, float>(obj1 =>
		{

			cpShape shape = obj1 as cpShape;

			var info = shape.SegmentQuery(start, end);

			if (
				!(shape.filter.group != 0 && group == shape.filter.group)
				&& (layers != 0)
				&& info != null
			)
			{
				func(shape, info.alpha, info.normal);
			}
			return 1;

		});

			this.Lock();
			{
				this.staticShapes.SegmentQuery(start, end, 1, helper);
				this.activeShapes.SegmentQuery(start, end, 1, helper);
			} this.Unlock(true);
		}

		/// Perform a fast rectangle query on the space calling @c func for each shape found.
		/// Only the shape's bounding boxes are checked for overlap, not their full shape.
		[Obsolete("This method was obsolete from Chipmunk JS")]
		public void bbQuery(cpBB bb, int layers, int group, Action<cpShape> func)
		{
			var helper = new Action<cpShape>(shape =>
			{
				if (
					!(shape.filter.group > 0 && group == shape.filter.group) && (layers != 0) &&
					 cp.bbIntersects2(bb, shape.bb_l, shape.bb_b, shape.bb_r, shape.bb_t)
				)
				{
					func(shape);
				}
			});


			Lock();
			{
				this.activeShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
				this.staticShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
			} Unlock(true);
		}

		[Obsolete("This method was obsolete from Chipmunk JS")]
		public Action<object, object> makeCollideShapes()
		{
			// It would be nicer to use .bind() or something, but this is faster.
			return new Action<object, object>((obj1, obj2) =>
			{

				var a = obj1 as cpShape;
				var b = obj2 as cpShape;

				// Reject any of the simple cases
				if (
					// BBoxes must overlap
					//!bbIntersects(a.bb, b.bb)
					!(a.bb_l <= b.bb_r && b.bb_l <= a.bb_r && a.bb_b <= b.bb_t && b.bb_b <= a.bb_t)
					// Don't collide shapes attached to the same body.
					|| a.body == b.body
					// Don't collide objects in the same non-zero group
					|| (a.filter.group != 0 && a.filter.group == b.filter.group)
					// Don't collide objects that don't share at least on layer.
					//|| !(a.filter.categories != 0 & b.filter.categories != 0
					//)
				) return;

				var handler = lookupHandler(a.type, b.type, defaultHandler);

				var sensor = a.sensor || b.sensor;
				if (sensor && handler == cp.defaultCollisionHandler) return;

				// Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
				if ((a as ICollisionShape).CollisionCode > (b as ICollisionShape).CollisionCode)
				{
					var temp = a;
					a = b;
					b = temp;
				}

				// Narrow-phase collision detection.
				//cpContact *contacts = cpContactBufferGetArray(space);
				//int numContacts = cpCollideShapes(a, b, contacts);
				var contacts = collideShapes(a, b);
				if (contacts == null || contacts.Count == 0)
					return; // Shapes are not colliding.
				//cpSpacePushContacts(space, numContacts);

				// Get an arbiter from space.arbiterSet for the two shapes.
				// This is where the persistant contact magic comes from.
				var arbHash = cp.hashPair(a.hashid, b.hashid);

				cpArbiter arb;
				if (!cachedArbiters.TryGetValue(arbHash, out arb))
				{
					arb = new cpArbiter(a, b);
					cachedArbiters.Add(arbHash, arb);
				}

				arb.Update(contacts, handler, a, b);

				// Call the begin function first if it's the first step
				if (arb.state == cpArbiterState.FirstColl && !handler.begin(arb, this, null))
				{
					arb.Ignore(); // permanently ignore the collision until separation
				}

				if (
					// Ignore the arbiter if it has been flagged
					(arb.state != cpArbiterState.Ignore) &&
					// Call preSolve
					handler.preSolve(arb, this, null) &&
					// Process, but don't add collisions for sensors.
					!sensor
				)
				{
					this.arbiters.Add(arb);
				}
				else
				{
					//cpSpacePopContacts(space, numContacts);

					arb.contacts = null;

					// Normally arbiters are set as used after calling the post-solve callback.
					// However, post-solve callbacks are not called for sensors or arbiters rejected from pre-solve.
					if (arb.state != cpArbiterState.Ignore) arb.state = cpArbiterState.Normal;
				}

				// Time stamp the arbiter so we know it was used recently.
				arb.stamp = this.stamp;
			});
		}

		[Obsolete("This method was obsolete from Chipmunk JS")]
		public cpSegmentQueryInfo segmentQueryFirst(cpVect start, cpVect end, int layers, int group)
		{
			cpSegmentQueryInfo output = null;

			var helper = new Func<object, float>(o1 =>
			{
				cpShape shape = o1 as cpShape;

				cpSegmentQueryInfo info = shape.SegmentQuery(start, end);


				if (
					!(shape.filter.group != 0 && group == shape.filter.group) &&
					!shape.sensor && info != null &&
					(output == null)
				)
				{
					output = info;
				}

				return output != null ? output.alpha : 1;
			}
				);

			this.staticShapes.SegmentQuery(start, end, 1f, helper);
			this.activeShapes.SegmentQuery(start, end, output != null ? output.alpha : 1, helper);

			return output;
		}

		[Obsolete("This method was obsolete from Chipmunk JS")]
		public cpShape NearestPointQuery(cpVect point, int maxDistance, int layers, int group)
		{

			cpPointQueryInfo output = null;

			var helper = new Action<object, object>((o1, o2) =>
			{
				cpShape shape = o1 as cpShape;

				if (!(shape.filter.group > 0 && group == shape.filter.group) && !shape.sensor)
				{
					cpPointQueryInfo info = shape.NearestPointQuery(point);

					if (info.distance < maxDistance && (output == null || info.distance < output.distance))
						output = info;
				}
			});

			cpBB bb = cp.bbNewForCircle(point, maxDistance);

			this.activeShapes.Query(bb, helper);
			this.staticShapes.Query(bb, helper);

			return output.shape;
		}

		#endregion

	}
}