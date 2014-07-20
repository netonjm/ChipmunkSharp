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

		/// Query the space at a point and call @c func for each shape found.
		public void pointQuery(cpVect point, int layers, int group, Action<cpShape> func)
		{

			if (point == null)
				return;

			var helper = new Action<cpShape>(shape =>
			{
				if (
					!(shape.group != 0 && group == shape.group) && (layers != 0 & shape.layers != 0) && shape.PointQuery(point) != null
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
					(a.group != 0 && a.group == b.group) ||
					(a.layers & b.layers) == 0 ||
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

		public void segmentQuery(cpVect start, cpVect end, int layers, int group, Action<cpShape, float, cpVect> func)
		{
			var helper = new Func<object, float>(obj1 =>
		{

			cpShape shape = obj1 as cpShape;

			var info = shape.SegmentQuery(start, end);

			if (
				!(shape.group != 0 && group == shape.group) && (layers != 0 & shape.layers != 0) &&
				info != null
			)
			{
				func(shape, info.t, info.n);
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
		public void bbQuery(cpBB bb, int layers, int group, Action<cpShape> func)
		{
			var helper = new Action<cpShape>(shape =>
			{
				if (
					!(shape.group > 0 && group == shape.group) && (layers > 0 & shape.layers > 0) &&
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
		//point, layers, group
		public void pointQueryFirst(cpVect point, float maxDistance, int layers, int group, Action<cpShape, float, cpVect> func)
		{
			var helper = new Action<object, object>((o1, o2) =>
			{
				var shape = o1 as cpShape;
				if (!(shape.group != 0 && group == shape.group) && (layers != 0 & shape.layers != 0))
				{
					var info = shape.NearestPointQuery(point);

					if (info.d < maxDistance) func(shape, info.d, info.p);
				}
			});

			var bb = cp.bbNewForCircle(point, maxDistance);

			this.Lock();
			{
				this.activeShapes.Query(bb, (o1, o2) => helper(o1, o2));
				this.staticShapes.Query(bb, (o1, o2) => helper(o1, o2));
			} this.Unlock(true);

		}
		//nearestPointQuery



	}
}