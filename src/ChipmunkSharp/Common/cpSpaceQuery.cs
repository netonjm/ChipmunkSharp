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


    public struct ShapeQueryContext
    {
        public cpSpaceShapeQueryFunc func;
        public object data;
        public bool anyCollision;
        public ShapeQueryContext(cpSpaceShapeQueryFunc func, object data, bool any)
        {

            this.func = func;
            this.data = data;
            this.anyCollision = any;
        }


    };

    public struct PointQueryContext
    {
        public cpVect point;
        public int layers;
        public int group;
        public cpSpacePointQueryFunc func;
        public object data;

        public PointQueryContext(cpVect point, int layers, int group, cpSpacePointQueryFunc func, object data)
        {
            this.point = point;
            this.layers = layers;
            this.group = group;
            this.func = func;
            this.data = data;
        }

    };


    /// Segment query callback function type.
    public delegate void cpSpaceSegmentQueryFunc(cpShape shape, float t, cpVect n, object data);
    /// Point query callback function type.
    public delegate void cpSpacePointQueryFunc(cpShape shape, object data);
    /// Nearest point query callback function type.
    public delegate void cpSpaceNearestPointQueryFunc(cpShape shape, float distance, cpVect point, object data);
    /// Rectangle Query callback function type.
    public delegate void cpSpaceBBQueryFunc(cpShape shape, object data);
    /// Space/body iterator callback function type.
    public delegate void cpSpaceBodyIteratorFunc(cpBody body, object data);
    /// Shape query callback function type.
    public delegate void cpSpaceShapeQueryFunc(cpShape shape, List<cpContact> points, object data);
    //struct cpContactBufferHeader cpContactBufferHeader;

    public partial class cpSpace
    {


        public static int PointQuery(PointQueryContext context, cpShape shape, int id, object data)
        {
            if (
                !(shape.group != 0 && context.group == shape.group) && (context.layers != 0 & shape.layers != 0) &&
                shape.PointQuery(context.point)  // cpShapePointQuery(shape, context.point)
            )
            {
                context.func(shape, context.data);
            }

            return id;
        }

        /// Query the space at a point and call @c func for each shape found.
        //public void PointQuery(cpVect point, int layers, int group, cpSpacePointQueryFunc func, object data)
        //{

        //    PointQueryContext context = new PointQueryContext(point, layers, group, func, data);
        //    cpBB bb = cpBB.cpBBNewForCircle(point, 0.0f);

        //    lock (this)
        //    {
        //        locked = true;

        //        //space.activeShapes

        //        activeShapes.IndexQuery(context, bb, ShapeQuery, data);
        //        staticShapes.IndexQuery(context, bb, ShapeQuery, data);



        //        //cpSpatialIndexQuery(space.activeShapes, context, bb, PointQuery, data);
        //        //cpSpatialIndexQuery(space.staticShapes, context, bb, PointQuery, data);
        //        locked = false;
        //    }

        //}

        //MARK: Query Functions



        // Callback from the spatial hash.
        public static int ShapeQuery(cpShape a, cpShape b, int id, ShapeQueryContext context)
        {
            // Reject any of the simple cases
            if (
                (a.group > 0 && a.group == b.group) ||
                !(a.layers > 0 & b.layers > 0) ||
                a == b
            ) return id;

            List<cpContact> contacts = new List<cpContact>();
            //int numContacts = 0;

            // Shape 'a' should have the lower shape type. (required by cpCollideShapes() )
            if (a.klass.type <= b.klass.type)
            {
                a.Collides(b, id, contacts);
            }
            else
            {
                b.Collides(a, id, contacts);
                for (int i = 0; i < contacts.Count; i++)
                    contacts[i].n.Neg(); // = cpVect.cpvneg(.n);
            }

            if (contacts.Count > 0)
            {
                context.anyCollision = !(a.sensor || b.sensor);

                if (context.func != null)
                {
                    context.func(b, contacts, context.data);
                }
            }

            return id;
        }

        public bool ShapeQuery(cpShape shape, cpSpaceShapeQueryFunc func, object data)
        {

            cpBody body = shape.body;
            cpBB bb = body != null ? shape.Update(body.Position, body.Rotation) : shape.bb;
            ShapeQueryContext context = new ShapeQueryContext(func, data, false);


            Lock();
            {

                activeShapes.Query((object)shape, bb, (a, b, i, c) =>
                {
                    return ShapeQuery((cpShape)a, (cpShape)b, i, (ShapeQueryContext)c);
                }, (object)context);

                staticShapes.Query((object)shape, bb, (a, b, i, c) =>
                {
                    return ShapeQuery((cpShape)a, (cpShape)b, i, (ShapeQueryContext)c);
                }, (object)context);

                //staticShapes.Query((object)shape, bb, ShapeQuery, context);
                // cpSpatialIndexQuery(space.staticShapes, shape, bb, (cpSpatialIndexQueryFunc)ShapeQuery, context);
            }

            Unlock(true);

            return context.anyCollision;
        }

        ///// Query the space at a point and call @c func for each shape found.
        //public void cpSpaceNearestPointQuery(cpSpace space, cpVect point, float maxDistance, int layers, int group, cpSpaceNearestPointQueryFunc func, object data);
        ///// Query the space at a point and return the nearest shape found. Returns null if no shapes were found.
        //public cpShape cpSpaceNearestPointQueryNearest(cpSpace space, cpVect point, float maxDistance, int layers, int group, cpNearestPointQueryInfo output);

        ///// Perform a directed line segment query (like a raycast) against the space calling @c func for each shape intersected.
        //public void cpSpaceSegmentQuery(cpSpace space, cpVect start, cpVect end, int layers, int group, cpSpaceSegmentQueryFunc func, object data);
        ///// Perform a directed line segment query (like a raycast) against the space and return the first shape hit. Returns null if no shapes were hit.
        //public cpShape cpSpaceSegmentQueryFirst(cpSpace space, cpVect start, cpVect end, int layers, int group, cpSegmentQueryInfo output);

        ///// Perform a fast rectangle query on the space calling @c func for each shape found.
        ///// Only the shape's bounding boxes are checked for overlap, not their full shape.
        //public void cpSpaceBBQuery(cpSpace space, cpBB bb, int layers, int group, cpSpaceBBQueryFunc func, object data);

        ///// Query a space for any shapes overlapping the given shape and call @c func for each shape found.
        //public bool cpSpaceShapeQuery(cpSpace space, cpShape shape, cpSpaceShapeQueryFunc func, object data);

        /// Call cpBodyActivate() for any shape that is overlaps the given shape.
        //public void cpSpaceActivateShapesTouchingShape(cpSpace space, cpShape shape);


        //public static void spaceEachShapeIterator(cpShape shape, spaceShapeContext context)
        //{
        //    //context.func(shape, context.data);
        //}


    }
}
//public static Pair cpSpaceHashQuery(cpSpaceHash hash, object obj, cpBB bb, cpSpatialIndexQueryFunc func, object data)
//{
//    // Get the dimensions in cell coordinates.
//    cpFloat dim = hash.celldim;
//    int l = floor_int(bb.l / dim);  // Fix by ShiftZ
//    int r = floor_int(bb.r / dim);
//    int b = floor_int(bb.b / dim);
//    int t = floor_int(bb.t / dim);

//    int n = hash.numcells;
//    cpSpaceHashBin** table = hash.table;

//    // Iterate over the cells and query them.
//    for (int i = l; i <= r; i++)
//    {
//        for (int j = b; j <= t; j++)
//        {
//            query_helper(hash, &table[hash_func(i, j, n)], obj, func, data);
//        }
//    }

//    hash.stamp++;
//}


/// Query the space at a point and return the first shape found. Returns null if no shapes were found.
//public cpShape PointQueryFirst(cpSpace space, cpVect point, int layers, int group)
//{
//    cpShape shape = null;
//    space.PointQuery(point, layers, group, PointQueryFirst, shape);
//    //cpSpacePointQuery(space,);

//    return shape;
//}

//public void PointQuery(cpVect point, int layers, int group, Func<cpSpace, cpVect, int, int, cpShape> PointQueryFirst, cpShape shape)
//{

//}
