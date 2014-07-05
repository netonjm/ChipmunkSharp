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

namespace ChipmunkSharp.Shapes
{


    public class cpNearestPointQueryInfo
    {

        /// The nearest shape, NULL if no shape was within range.
        public cpShape shape;
        /// The closest point on the shape's surface. (in world space coordinates)
        public cpVect p;
        /// The distance to the point. The distance is negative if the point is inside the shape.
        public float d;
        /// The gradient of the signed distance function.
        /// The same as info.p/info.d, but accurate even for very small values of info.d.
        //   public cpVect g;

        public cpNearestPointQueryInfo(cpShape shape, cpVect p, float d)
        {
            /// The nearest shape, NULL if no shape was within range.
            this.shape = shape;
            /// The closest point on the shape's surface. (in world space coordinates)
            this.p = p;
            /// The distance to the point. The distance is negative if the point is inside the shape.
            this.d = d;
            //    this.g = g;
        }


        public void Set(cpNearestPointQueryInfo newPointInfo)
        {
            /// The nearest shape, NULL if no shape was within range.
            shape = newPointInfo.shape;
            p = newPointInfo.p;
            d = newPointInfo.d;
            // g = newPointInfo.g;
        }

        //public static cpNearestPointQueryInfo CreateEmpty()
        //{
        //    cpNearestPointQueryInfo tmp = new cpNearestPointQueryInfo(null, cpVect.ZERO, cpEnvironment.Infinity, cpVect.ZERO);
        //    return tmp;
        //}
    }

    /// Extended point query info struct. Returned from calling pointQuery on a shape.
    public class cpSegmentQueryInfo
    {

        /// The shape that was hit, NULL if no collision occured.
        public cpShape shape;
        /// The normalized distance along the query segment in the range [0, 1].
        public float t;
        /// The normal of the surface hit.
        public cpVect n;

        public cpSegmentQueryInfo(cpShape shape, float t, cpVect n)
        {
            /// The shape that was hit, NULL if no collision occured.
            this.shape = shape;
            /// The normalized distance along the query segment in the range [0, 1].
            this.t = t;
            /// The normal of the surface hit.
            this.n = n;


        }

        public void Set(cpSegmentQueryInfo info1)
        {
            /// The shape that was hit, NULL if no collision occured.
            this.shape = info1.shape;
            /// The normalized distance along the query segment in the range [0, 1].
            this.t = info1.t;
            /// The normal of the surface hit.
            this.n = info1.n;
        }

        public static cpSegmentQueryInfo CreateBlanck()
        {
            return new cpSegmentQueryInfo(null, 1.0f, cpVect.ZERO);
        }

        public cpVect hitPoint(cpVect start, cpVect end)
        {
            return cpVect.Lerp(start, end, this.t);
        }

        public float hitDist(cpVect start, cpVect end)
        {
            return cpVect.Distance(start, end) * this.t;
        }





    }


    /// Extended point query info struct. Returned from calling pointQuery on a shape.
    public struct cpPointQueryExtendedInfo
    {
        public cpShape shape;
        public cpVect n;
        public float d;

        public cpPointQueryExtendedInfo(cpShape tShape)
        {
            /// The nearest shape, NULL if no shape was within range.
            this.shape = tShape;
            /// The closest point on the shape's surface. (in world space coordinates)
            this.d = cp.Infinity;
            /// The distance to the point. The distance is negative if the point is inside the shape.
            this.n = cpVect.ZERO;
        }

    }

}
