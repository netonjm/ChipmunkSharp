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

	public partial class cpSpace
	{


		#region OBSOLETE

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
				this.dynamicShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
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
		public bool shapeQuery(cpShape shape, Action<cpShape, List<cpContact>> func)
		{

			cpBody body = shape.body;

			//var bb = (body ? shape.update(body.p, body.rot) : shape.bb);
			if (body != null)
			{
				shape.Update(body.GetPosition(), body.GetRotation());
			}

			var bb = new cpBB(shape.bb.l, shape.bb.b, shape.bb.r, shape.bb.t);

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


				List<cpContact> contacts = new List<cpContact>();

				// Shape 'a' should have the lower shape type. (required by collideShapes() )
				if ((a as ICollisionShape).CollisionCode <= (b as ICollisionShape).CollisionCode)
				{
					contacts = collideShapes(a, b);
				}
				else
				{
					contacts = collideShapes(b, a);
					List<cpContact> contactsModified = new List<cpContact>();
					for (var i = 0; i < contacts.Count; i++)
					{
						cpContact contacto = contacts[i];
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
				this.dynamicShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
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
				this.dynamicShapes.SegmentQuery(start, end, 1, helper);
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
					 cp.bbIntersects2(bb, shape.bb.l, shape.bb.b, shape.bb.r, shape.bb.t)
				)
				{
					func(shape);
				}
			});


			Lock();
			{
				this.dynamicShapes.Query(bb, (o1, o2) => { helper(o1 as cpShape); });
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
					!(a.bb.l <= b.bb.r && b.bb.l <= a.bb.r && a.bb.b <= b.bb.t && b.bb.b <= a.bb.t)
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
				if (arb.state == cpArbiterState.FirstCollision && !handler.beginFunc(arb, this, null))
				{
					arb.Ignore(); // permanently ignore the collision until separation
				}

				if (
					// Ignore the arbiter if it has been flagged
					(arb.state != cpArbiterState.Ignore) &&
					// Call preSolve
					handler.preSolveFunc(arb, this, null) &&
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
			this.dynamicShapes.SegmentQuery(start, end, output != null ? output.alpha : 1, helper);

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

			this.dynamicShapes.Query(bb, helper);
			this.staticShapes.Query(bb, helper);

			return output.shape;
		}

		#endregion

		/*

		//MARK: Nearest Point Query Functions

		public string NearestPointQuery(PointQueryContext context, cpShape shape, string id, object data)
		{
			if (
				!cpShapeFilter.Reject(shape.filter, context.filter)
			)
			{
				cpPointQueryInfo info = null;
				shape.PointQuery(context.point, ref info);

				if (info.shape != null && info.distance < context.maxDistance) context.func(shape, info.point, info.distance, info.gradient, data);
			}
			return id;
		}

		public string PointQuery(cpVect point, float maxDistance, cpShapeFilter filter, Action<cpShape, cpVect, float, cpVect, object> func, object data)
		{
			PointQueryContext context = new PointQueryContext(point, maxDistance, filter, func);
			cpBB bb = cpBB.cpBBNewForCircle(point, cp.cpfmax(maxDistance, 0.0f));

			Lock();
			{
				this.activeShapes.Query(context, bb, NearestPointQuery, data);
				//cpSpatialIndexQuery(this.dynamicShapes, &context, bb, NearestPointQuery, data);
				this.staticShapes.Query(context, bb, NearestPointQuery, data);
			} Unlock(true);
		}

		public string NearestPointQueryNearest(PointQueryContext context, cpShape shape, string id, ref cpPointQueryInfo output)
		{
			if (
				!cpShapeFilter.Reject(shape.filter, context.filter) && !shape.sensor
			)
			{
				cpPointQueryInfo info = null;
				shape.PointQuery(context.point, ref info);

				if (info.distance < output.distance)
					output = info;
			}

			return id;
		}

		//MARK: Segment Query Functions

		public float SegmentQuery(SegmentQueryContext context, cpShape shape, object data)
		{
			cpSegmentQueryInfo info = null;

			if (
				!cpShapeFilter.Reject(shape.filter, context.filter) &&
				shape.SegmentQuery(context.start, context.end, context.radius, ref info)
			)
			{
				context.func(shape, info.point, info.normal, info.alpha, data);
			}

			return 1.0f;
		}

		public void SegmentQuery(cpVect start, cpVect end, float radius, cpShapeFilter filter, Action<cpShape, cpVect, cpVect, float, object> func, object data)
		{
			SegmentQueryContext context = new SegmentQueryContext(
		start, end,
		radius,
		filter,
		func
	);

			Lock();
			{
				this.staticShapes.SegmentQuery(context, start, end, 1.0f, SegmentQuery, data);
				this.activeShapes.SegmentQuery(context, start, end, 1.0f, SegmentQuery, data);
				//this.dynamicShapes.SegmentQuery(context, start, end, 1.0f, SegmentQuery, data);

				//cpSpatialIndexSegmentQuery(space->staticShapes, &context, start, end, 1.0f, (cpSpatialIndexSegmentQueryFunc)SegmentQuery, data);
				//cpSpatialIndexSegmentQuery(space->dynamicShapes, &context, start, end, 1.0f, (cpSpatialIndexSegmentQueryFunc)SegmentQuery, data);

			} Unlock(true);
		}

		public float SegmentQueryFirst(SegmentQueryContext context, cpShape shape, ref cpSegmentQueryInfo output)
		{
			cpSegmentQueryInfo info = null;

			if (
				!cpShapeFilter.Reject(shape.filter, context.filter) && !shape.sensor &&
				shape.SegmentQuery(context.start, context.end, context.radius, ref info) &&
				info.alpha < output.alpha
			)
			{
				output = info;
			}

			return output.alpha;
		}

		public cpShape SegmentQueryFirst(cpSpace space, cpVect start, cpVect end, float radius, cpShapeFilter filter, ref cpSegmentQueryInfo output)
		{

			cpSegmentQueryInfo info = new cpSegmentQueryInfo(null, end, cpVect.Zero, 1.0f);
			if (output == null)
				output = info;

			SegmentQueryContext context = new SegmentQueryContext(
			   start, end,
			   radius,
			   filter,
			   null);

			this.staticShapes.SegmentQuery(context, start, end, 1.0f, SegmentQueryFirst, ref output);
			//		this.dynamicShapes.SegmentQuery(context, start, end, output.alpha , SegmentQueryFirst, ref output);
			this.activeShapes.SegmentQuery(context, start, end, output.alpha, SegmentQueryFirst, ref output);

			return output.shape;
		}

		//MARK: BB Query Functions

		public string BBQuery(BBQueryContext context, cpShape shape, string id, object data)
		{
			if (
				!cpShapeFilter.Reject(shape.filter, context.filter) &&
				context.bb.Intersects(shape.GetBB())
			)
			{
				context.func(shape, data);
			}

			return id;
		}

		public void BBQuery(cpBB bb, cpShapeFilter filter, Action<cpShape, object> func, object data)
		{
			BBQueryContext context = new BBQueryContext(bb, filter, func);

			Lock();
			{

				this.staticShapes.Query(ref context, bb, BBQuery, data);
				this.activeShapes.Query(ref context, bb, BBQuery, data);
				//this.dynamicShapes.SegmentQuery(context, start, end, 1.0f, SegmentQuery, data);

				//cpSpatialIndexQuery(space->dynamicShapes, &context, bb, (cpSpatialIndexQueryFunc)BBQuery, data);
				//cpSpatialIndexQuery(space->staticShapes, &context, bb, (cpSpatialIndexQueryFunc)BBQuery, data);
			} Unlock(true);
		}

		public string ShapeQuery(cpShape a, cpShape b, string id, ShapeQueryContext context)
		{
			if (cpShapeFilter.Reject(a.filter, b.filter) || a == b) return id;

			cpContactPointSet set = cpShape.Collide(a, b);
			if (set.Count > 0)
			{
				if (context.func != null) context.func(b, set, context.data);
				context.anyCollision = !(a.sensor || b.sensor);
			}
			return id;
		}

		public bool ShapeQuery(cpShape shape, Action<cpShape, cpContactPointSet, object> func, object data)
		{
			cpBody body = shape.body;
			cpBB bb = (body != null ? shape.Update(body.transform) : shape.GetBB());
			ShapeQueryContext context = new ShapeQueryContext(func, data, false);

			Lock();
			{
				this.staticShapes.Query(shape, bb, ShapeQuery, ref context);
				this.activeShapes.Query(shape, bb, ShapeQuery, ref context);

				//cpSpatialIndexQuery(space->dynamicShapes, shape, bb, (cpSpatialIndexQueryFunc)ShapeQuery, &context);
				//cpSpatialIndexQuery(space->staticShapes, shape, bb, (cpSpatialIndexQueryFunc)ShapeQuery, &context);
			} Unlock(true);

			return context.anyCollision;
		}

		 */

	}

	public struct BBQueryContext
	{
		public cpBB bb;
		public cpShapeFilter filter;
		public Action<cpShape, object> func;


		public BBQueryContext(cpBB bb1, cpShapeFilter filter1, Action<cpShape, object> func1)
		{
			// TODO: Complete member initialization
			this.bb = bb1;
			this.filter = filter1;
			this.func = func1;
		}
	};

	public struct ShapeQueryContext
	{
		public Action<cpShape, cpContactPointSet, object> func;
		public object data;
		public bool anyCollision;

		public ShapeQueryContext(Action<cpShape, cpContactPointSet, object> func, object data, bool anyCollision)
		{
			// TODO: Complete member initialization
			this.func = func;
			this.data = data;
			this.anyCollision = anyCollision;
		}
	};

	public struct SegmentQueryContext
	{
		public cpVect start, end;
		public float radius;
		public cpShapeFilter filter;
		public Action<cpShape, cpVect, cpVect, float, object> func;


		public SegmentQueryContext(cpVect start1, cpVect end1, float radius1, cpShapeFilter filter1, Action<cpShape, cpVect, cpVect, float, object> func1)
		{
			// TODO: Complete member initialization
			this.start = start1;
			this.end = end1;
			this.radius = radius1;
			this.filter = filter1;
			this.func = func1;
		}
	};


	public struct PointQueryContext
	{

		public cpVect point;
		public float maxDistance;
		public cpShapeFilter filter;
		public Action<cpShape, cpVect, float, cpVect, object> func;

		public PointQueryContext(cpVect point1, float maxDistance1, cpShapeFilter filter1, Action<cpShape, cpVect, float, cpVect, object> func1)
		{
			// TODO: Complete member initialization
			this.point = point1;
			this.maxDistance = maxDistance1;
			this.filter = filter1;
			this.func = func1;
		}
	};

}