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


    ///// Segment query callback function type.
    //public delegate void cpSpaceSegmentQueryFunc(cpShape shape, float t, cpVect n, object data);
    ///// Point query callback function type.
    //public delegate void cpSpacePointQueryFunc(cpShape shape, object data);
    ///// Nearest point query callback function type.
    //public delegate void cpSpaceNearestPointQueryFunc(cpShape shape, float distance, cpVect point, object data);
    ///// Rectangle Query callback function type.
    //public delegate void cpSpaceBBQueryFunc(cpShape shape, object data);
    ///// Space/body iterator callback function type.
    //public delegate void cpSpaceBodyIteratorFunc(cpBody body, object data);
    ///// Shape query callback function type.
    //public delegate void cpSpaceShapeQueryFunc(cpShape shape, List<ContactPoint> points, object data);
    ////struct cpContactBufferHeader cpContactBufferHeader;


    public partial class cpSpace
    {


        /// Query the space at a point and call @c func for each shape found.
        public void pointQuery(cpVect point, int layers, int group, Action<cpShape> func)
        {
            var helper = new Action<cpShape>(shape =>
            {
                if (
                    !(shape.group > 0 && group == shape.group) && (layers > 0 & shape.layers > 0) &&
                    shape.PointQuery(point)
                )
                {
                    func(shape);
                }
            });

            var bb = new cpBB(point.x, point.y, point.x, point.y);
            Lock();
            {
                this.activeShapes.query(bb, (o1, o2) => { helper(o1 as cpShape); return null; });
                this.staticShapes.query(bb, (o1, o2) => { helper(o1 as cpShape); return null; });
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
                shape.update(body.Position, body.Rotation);
            }

            var bb = shape.bb;

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
                if ((a as ICollisionShape).collisionCode <= (b as ICollisionShape).collisionCode)
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
                this.activeShapes.query(bb, (o1, o2) => { helper(o1 as cpShape); return null; });
                this.staticShapes.query(bb, (o1, o2) => { helper(o1 as cpShape); return null; });
            }
            Unlock(true);

            return anyCollision;
        }


        /// Perform a fast rectangle query on the space calling @c func for each shape found.
        /// Only the shape's bounding boxes are checked for overlap, not their full shape.
        public void bbQuery(cpBB bb, int layers, int group, Action<cpShape> func)
        {
            var helper = new Action<cpShape>(shape =>
            {
                if (
                    !(shape.group > 0 && group == shape.group) && (layers > 0 & shape.layers > 0) &&
                     cpEnvironment.bbIntersects2(bb, shape.bb.l, shape.bb.b, shape.bb.r, shape.bb.t)
                )
                {
                    func(shape);
                }
            });


            Lock();
            {
                this.activeShapes.query(bb, (o1, o2) => { helper(o1 as cpShape); return null; });
                this.staticShapes.query(bb, (o1, o2) => { helper(o1 as cpShape); return null; });
            } Unlock(true);
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