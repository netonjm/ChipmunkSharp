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


		public ulong NearestPointQuery(PointQueryContext context, cpShape shape, ulong id, object data)
		{
			if (
				!cpShapeFilter.Reject(shape.filter, context.filter)
			)
			{
				cpPointQueryInfo info = null;
				shape.PointQuery(context.point, ref info);

				if (info.shape != null && info.distance < context.maxDistance) 
					context.func(shape, info.point, info.distance, info.gradient, data);
			}
			return id;
		}

		public void PointQuery(cpVect point, float maxDistance, cpShapeFilter filter, Action<cpShape, cpVect, float, cpVect, object> func, object data)
		{
			PointQueryContext context = new PointQueryContext(point, maxDistance, filter, func);
			cpBB bb = cpBB.cpBBNewForCircle(point, cp.cpfmax(maxDistance, 0.0f));

			Lock();
			{
				this.staticShapes.Query(context, bb, (ctx, shape, colid, o) => NearestPointQuery((PointQueryContext)ctx, shape as cpShape, colid, o), data);
				this.dynamicShapes.Query(context, bb, (ctx, shape, colid, o) => NearestPointQuery((PointQueryContext)ctx, shape as cpShape, colid, o), data);
			} Unlock(true);
		}

		public ulong NearestPointQueryNearest(object ctx, cpShape shape, ulong id, ref object outp)
		{
			PointQueryContext context = (PointQueryContext)ctx;
			cpPointQueryInfo output = (cpPointQueryInfo)outp;

			if (
				!cpShapeFilter.Reject(shape.filter, context.filter) && !shape.sensor
			)
			{
				cpPointQueryInfo info = null;
				shape.PointQuery(context.point, ref info);

				if (info.distance < output.distance)
					outp = (object)info;
			}

			return id;
		}

		public cpShape PointQueryNearest(cpVect point, float maxDistance, cpShapeFilter filter, ref cpPointQueryInfo output)
		{
			cpPointQueryInfo info = new cpPointQueryInfo(null, cpVect.Zero, maxDistance, cpVect.Zero);
			if (output == null)
				output = info;

			PointQueryContext context = new PointQueryContext(
			point, maxDistance,
			filter,
			null
			);

			cpBB bb = cpBB.cpBBNewForCircle(point, cp.cpfmax(maxDistance, 0.0f));

			object outp = (object)output;

			this.dynamicShapes.Query(context, bb,
				(o1, o2, s, o3) => NearestPointQueryNearest(o1, (cpShape)o2, s, ref outp)
			, null);

			this.staticShapes.Query(context, bb,
				(o1, o2, s, o3) => NearestPointQueryNearest(o1, (cpShape)o2, s, ref outp)
			, null);

			output = (cpPointQueryInfo)outp;

			return output.shape;
		}


		//MARK: Segment Query Functions

		public float SegmentQueryFunc(SegmentQueryContext context, cpShape shape, object data)
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
				this.staticShapes.SegmentQuery(context, start, end, 1.0f,
					(o1, o2, o3) => SegmentQueryFunc((SegmentQueryContext)o1, o2 as cpShape, o3)
				, data);
				this.dynamicShapes.SegmentQuery(context, start, end, 1.0f,
					(o1, o2, o3) => SegmentQueryFunc((SegmentQueryContext)o1, o2 as cpShape, o3)
					, data);

			} Unlock(true);
		}

		public float SegmentQueryFirstFunc(SegmentQueryContext context, cpShape shape, cpSegmentQueryInfo output)
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

		public cpShape SegmentQueryFirst(cpVect start, cpVect end, float radius, cpShapeFilter filter, ref cpSegmentQueryInfo output)
		{

			cpSegmentQueryInfo info = new cpSegmentQueryInfo(null, end, cpVect.Zero, 1.0f);
			if (output == null)
				output = info;

			SegmentQueryContext context = new SegmentQueryContext(
			   start, end,
			   radius,
			   filter,
			   null);

			this.staticShapes.SegmentQuery(context, start, end, 1.0f,
			 (o1, o2, o3) => SegmentQueryFirstFunc((SegmentQueryContext)o1, o2 as cpShape, (cpSegmentQueryInfo)o3)
				 , output);
			//		this.dynamicShapes.SegmentQuery(context, start, end, output.alpha , SegmentQueryFirst, ref output);
			this.dynamicShapes.SegmentQuery(context, start, end, output.alpha,
				 (o1, o2, o3) => SegmentQueryFirstFunc((SegmentQueryContext)o1, o2 as cpShape, (cpSegmentQueryInfo)o3)
				, output);

			return output.shape;
		}

		//MARK: BB Query Functions

		public ulong BBQueryFunc(BBQueryContext context, cpShape shape, ulong id, object data)
		{
			if (
				!cpShapeFilter.Reject(shape.filter, context.filter) &&
				context.bb.Intersects(shape.bb)
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

				this.staticShapes.Query(context, bb, (o1, o2, s, o3) =>

					BBQueryFunc((BBQueryContext)o1, o2 as cpShape, s, o3)
				, data);

				this.dynamicShapes.Query(context, bb, (o1, o2, s, o3) =>

					BBQueryFunc((BBQueryContext)o1, o2 as cpShape, s, o3)
				, data);

			} Unlock(true);
		}

		public ulong ShapeQueryFunc(cpShape a, cpShape b, ulong id, ShapeQueryContext context)
		{
			if (cpShapeFilter.Reject(a.filter, b.filter) || a == b) return id;

			List<cpContact> contacts = new List<cpContact>();
			cpContactPointSet set = cpShape.Collide(a, b, ref contacts);

			if (set.count > 0)
			{
				if (context.func != null) context.func(b, set, context.data);
				context.anyCollision = !(a.sensor || b.sensor);
			}
			return id;
		}

		public bool ShapeQuery(cpShape shape, Action<cpShape, cpContactPointSet, object> func, object data)
		{

			cpBody body = shape.body;
			cpBB bb = (body != null ? shape.Update(body.transform) : shape.bb);
			ShapeQueryContext context = new ShapeQueryContext(func, data, false);

			object ctx = (object)context;

			Lock();
			{

				this.staticShapes.Query(shape, bb,
					(o1, o2, s, o3) => ShapeQueryFunc(o1 as cpShape, o2 as cpShape, s, (ShapeQueryContext)o3)
				, ctx);

				this.dynamicShapes.Query(shape, bb,
					(o1, o2, s, o3) => ShapeQueryFunc(o1 as cpShape, o2 as cpShape, s, (ShapeQueryContext)o3)
				, ctx);

			} Unlock(true);

			return ((ShapeQueryContext)ctx).anyCollision;
		}

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