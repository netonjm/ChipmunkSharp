/* Copyright (c) 2014 ported by Jose Medrano (@netonjm)
  
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

    public struct cpMat2x2
    {
        // Row major [[a, b][c d]]
        public float a, b, c, d;

        public cpMat2x2(float a, float b, float c, float d)
        {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
        }

    } ;

    public class cpEnvironment
    {

        // Chipmunk 6.2.1
        public static string CP_VERSION_MAJOR = "6";
        public static string CP_VERSION_MINOR = "2";
        public static string CP_VERSION_RELEASE = "1";

        public static string cpVersionString = string.Format("{0}.{1}.{2}", CP_VERSION_MAJOR, CP_VERSION_MINOR, CP_VERSION_RELEASE);


        public static int shapeIDCounter = 0;
        public static int CP_USE_CGPOINTS = 1;
        public static int CP_NO_GROUP = 0;
        public static int CP_ALL_LAYERS = -1;

        public static int numApplyImpulse = 0;
        public static int numApplyContact = 0;

        public static float scale;

        public static byte[] INFINITY = { 0x00, 0x00, 0x80, 0x7F };

        public static float MAGIC_EPSILON = 1e-5F;

        public static float INFINITY_FLOAT
        {
            get
            {
                return BitConverter.ToSingle(INFINITY, 0);
            }
        }
        public static void resetShapeIdCounter()
        {
            shapeIDCounter = 0;
        }

        //public static int CP_HASH_COEF = 3344921057ul;

        public static int CP_HASH_PAIR(int A, int B)
        {
            return A ^ B;
        }

        public static int CP_HASH_PAIR(cpPolyShape poly, int B)
        {
            return poly.hashid ^ B;
        }
        public static int CP_HASH_PAIR(cpSegmentShape seg, int B)
        {
            return seg.hashid ^ B;
        }

        //===========================================================

        /** Clamp a value between from and to.
           @since v0.99.1
       */
        #region Mathemathical Operations and variables

        //public static double INFINITY2 = 1e1000;

        //public static float M_PI = 3.14159265358979323846264338327950288f;
        //public static float M_E = 2.71828182845904523536028747135266250f;

        public static float cpfceil(float a)
        {
            return (float)Math.Ceiling((double)a);
        }

        public static float cpffloor(float a)
        {
            return (float)Math.Floor((double)a);
        }

        public static float cpfpow(float a, float b)
        {
            return (float)Math.Pow((double)a, (double)b);
        }

        public static float cpfexp(float a)
        {
            return (float)Math.Exp((double)a);
        }

        public static float cpfmod(float a, float b)
        {
            return a % b;
        }

        public static float cpfatan2(float a, float b)
        {
            return (float)Math.Atan2((double)a, (double)b);
        }

        public static float cpfacos(float a)
        {
            return (float)Math.Acos((double)a);
        }

        public static float cpfsqrt(float a)
        {
            return (float)Math.Sqrt((double)a);
        }

        public static float cpfsin(float a)
        {
            return (float)Math.Sin((double)a);
        }

        public static float cpfcos(float a)
        {
            return (float)Math.Cos((double)a);
        }

        //public static IntPtr cpHashValue
        //{
        //    get
        //    {
        //        return new IntPtr();
        //    }
        //}

        //public static bool cpBool
        //{
        //    get
        //    {
        //        return new IntPtr();
        //    }
        //}




        /// Return the max of two cpFloats.
        public static float cpfmax(float a, float b)
        {
            return (a > b) ? a : b;
        }

        /// Return the min of two floats.
        public static float cpfmin(float a, float b)
        {
            return (a < b) ? a : b;
        }

        /// Return the absolute value of a float.
        public static float cpfabs(float f)
        {
            return (f < 0) ? -f : f;
        }

        /// Clamp @c f to be between @c min and @c max.
        public static float cpfclamp(float f, float min, float max)
        {
            return cpfmin(cpfmax(f, min), max);
        }

        public static float cpclamp(float value, float min_inclusive, float max_inclusive)
        {
            if (min_inclusive > max_inclusive)
            {
                float ftmp = min_inclusive;
                min_inclusive = max_inclusive;
                max_inclusive = ftmp;
            }

            return value < min_inclusive ? min_inclusive : value < max_inclusive ? value : max_inclusive;
        }

        /// Clamp @c f to be between 0 and 1.
        public static float cpfclamp01(float f)
        {
            return cpfmax(0.0f, cpfmin(f, 1.0f));
        }

        /// Linearly interpolate (or extrapolate) between @c f1 and @c f2 by @c t percent.
        public static float cpflerp(float f1, float f2, float t)
        {
            return f1 * (1.0f - t) + f2 * t;
        }

        /// Linearly interpolate from @c f1 to @c f2 by no more than @c d.
        public static float cpflerpconst(float f1, float f2, float d)
        {
            return f1 + cpfclamp(f2 - f1, -d, d);
        }

        /// Return the min of two cpFloats.
        //static float cpfmin(float a, float b)
        //{
        //    return (a < b) ? a : b;
        //}

        //static float cpfmax(float a, float b)
        //{
        //    return (a > b) ? a : b;
        //}

        #endregion

        public static float k_scalar_body(cpBody body, cpVect r, cpVect n)
        {
            var rcn = r.CrossProduct(n);
            return body.m_inv + body.i_inv * rcn * rcn;
        }

        public static float k_scalar(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect n)
        {
            var value = k_scalar_body(a, r1, n) + k_scalar_body(b, r2, n);

            //assertSoft(value != 0, "Unsolvable collision or constraint.");

            return value;
        }


        public static void cpAssertHard(object p1, string p2)
        {

        }





        #region Physics Operations

        public static cpVect relative_velocity(cpBody a, cpBody b, cpVect r1, cpVect r2)
        {

            cpVect v1_sum = cpVect.cpvadd(a.v, cpVect.cpvmult(cpVect.cpvperp(r1), a.w));
            cpVect v2_sum = cpVect.cpvadd(b.v, cpVect.cpvmult(cpVect.cpvperp(r2), b.w));

            return cpVect.cpvsub(v2_sum, v1_sum);
        }

        public static float normal_relative_velocity(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect n)
        {
            return cpVect.cpvdot(relative_velocity(a, b, r1, r2), n);
        }



        public static void apply_impulse(cpBody body, cpVect j, cpVect r)
        {
            body.v = cpVect.cpvadd(body.v, cpVect.cpvmult(j, body.m_inv));
            body.w += body.i_inv * cpVect.cpvcross(r, j);
        }

        public static void apply_impulses(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect vec)
        {
            apply_impulse(a, vec.Neg(), r1);
            apply_impulse(b, vec, r2);
        }


        public static void apply_impulses(cpBody a, cpBody b, cpVect r1, cpVect r2, float jx, float jy)
        {
            apply_impulse(a, -jx, -jy, r1);
            apply_impulse(b, jx, jy, r2);
        }

        public static void apply_impulse(cpBody body, float jx, float jy, cpVect r)
        {
            //	body.v = body.v.add(vmult(j, body.m_inv));
            body.v.x += jx * body.m_inv;
            body.v.y += jy * body.m_inv;
            //	body.w += body.i_inv*vcross(r, j);
            body.w += body.i_inv * (r.x * jy - r.y * jx);
        }

        public static void apply_bias_impulses(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect j)
        {
            apply_bias_impulse(a, cpVect.cpvneg(j), r1);
            apply_bias_impulse(b, j, r2);
        }

        public static void apply_bias_impulse(cpBody body, cpVect j, cpVect r)
        {
            body.v_bias = cpVect.cpvadd(body.v_bias, cpVect.cpvmult(j, body.m_inv));
            body.w_bias += body.i_inv * cpVect.cpvcross(r, j);
        }

        /*
        var apply_bias_impulses = function(a, b, r1, r2, j)
        {
            apply_bias_impulse(a, vneg(j), r1);
            apply_bias_impulse(b, j, r2);
        };*/


        // k1 and k2 are modified by the function to contain the outputs.
        public static void k_tensor(cpBody a, cpBody b, cpVect r1, cpVect r2, cpVect k1, cpVect k2)
        {
            // calculate mass matrix
            // If I wasn't lazy and wrote a proper matrix class, this wouldn't be so gross...
            float k11, k12, k21, k22;
            var m_sum = a.m_inv + b.m_inv;

            // start with I*m_sum
            k11 = m_sum; k12 = 0;
            k21 = 0; k22 = m_sum;

            // add the influence from r1
            var a_i_inv = a.i_inv;
            var r1xsq = r1.x * r1.x * a_i_inv;
            var r1ysq = r1.y * r1.y * a_i_inv;
            var r1nxy = -r1.x * r1.y * a_i_inv;
            k11 += r1ysq; k12 += r1nxy;
            k21 += r1nxy; k22 += r1xsq;

            // add the influnce from r2
            var b_i_inv = b.i_inv;
            var r2xsq = r2.x * r2.x * b_i_inv;
            var r2ysq = r2.y * r2.y * b_i_inv;
            var r2nxy = -r2.x * r2.y * b_i_inv;
            k11 += r2ysq; k12 += r2nxy;
            k21 += r2nxy; k22 += r2xsq;

            // invert
            var determinant = k11 * k22 - k12 * k21;

            //assertSoft(determinant !== 0, "Unsolvable constraint.");
            if (determinant == 0)
            {
                throw new NotImplementedException("Unsolvable constraint.");
            }

            var det_inv = 1 / determinant;

            k1.x = k22 * det_inv; k1.y = -k12 * det_inv;
            k2.x = -k21 * det_inv; k2.y = k11 * det_inv;
        }

        public static float bias_coef(float errorBias, float dt)
        {
            return 1.0f - (float)Math.Pow(errorBias, dt);
        }


        #endregion



        public static void cpAssertSoft(bool p1, string p2)
        {
			if (!p1)
            	LogWrite(string.Format("cpAssertSoft:{0} Value:{1}", p2, p1));
        }

        public static void LogWrite(string message)
        {
            Console.WriteLine(message);
        }

        public static void cpAssertWarn(bool p1, string p2)
        {
            LogWrite(string.Format("AssertWarn:{0} Value:{1}", p2, p1));
        }

        public static void cpAssertWarn(bool p1, string p2, params object[] parameters)
        {
            LogWrite(string.Format("AssertWarn:{0} Value:{1}", p2, p1));
        }

        //internal static object CP_HASH_PAIR(int a, int b)
        //{
        //    throw new NotImplementedException();
        //}

        public static void cpAssertHard(string p)
        {
            LogWrite(string.Format("cpAssertHard:{0} Value:{1}", p, ""));
        }

        public static float PHYSICS_INFINITY { get { return INFINITY_FLOAT; } }

        public static void CCLOG(string p)
        {
            Console.WriteLine(p);
        }

        public static void CCASSERT(bool p1, string p2)
        {
            LogWrite(string.Format("CCASSERT:{0} VALUE:{1}", p2, p1));
        }

        public static void CC_ASSERT(bool p)
        {
            LogWrite(string.Format("CCASSERT:{0}", p));
        }

        #region MOMENTS

        public static float momentForCircle(float m, float r1, float r2, cpVect offset)
        {

            return m * (0.5f * (r1 * r1 + r2 * r2) + offset.LengthSQ);
        }

        public static float areaForCircle(float r1, float r2)
        {
            return (float)Math.PI * (float)Math.Abs(r1 * r1 - r2 * r2);
        }

        public static float momentForSegment(float m, cpVect a, cpVect b)
        {
            var offset = a.Add(b).Multiply(0.5f);
            return m * (b.DistanceSQ(a) / 12 + offset.LengthSQ);
        }

        public static float areaForSegment(cpVect a, cpVect b, float r)
        {
            return r * ((float)Math.PI * r + 2 * a.Distance(b));
        }

        public static float areaForSegment(float m, List<float> verts, cpVect offset)
        {
            float sum1 = 0;
            float sum2 = 0;
            //var len = ;
            for (var i = 0; i < verts.Count; i += 2)
            {
                var v1x = verts[i] + offset.x;
                var v1y = verts[i + 1] + offset.y;
                var v2x = verts[(i + 2) % verts.Count] + offset.x;
                var v2y = verts[(i + 3) % verts.Count] + offset.y;

                var a = cpVect.cpvcross2(v2x, v2y, v1x, v1y);
                var b = cpVect.cpvdot2(v1x, v1y, v1x, v1y) + cpVect.cpvdot2(v1x, v1y, v2x, v2y) + cpVect.cpvdot2(v2x, v2y, v2x, v2y);

                sum1 += a * b;
                sum2 += a;
            }

            return (m * sum1) / (6 * sum2);
        }

        public static float areaForPoly(List<float> verts)
        {
            float area = 0.0f;
            for (int i = 0, len = verts.Count; i < len; i += 2)
            {
                area += cpVect.cpvcross(new cpVect(verts[i], verts[i + 1]), new cpVect(verts[(i + 2) % len], verts[(i + 3) % len]));
            }

            return -area / 2;
        }

        public static cpVect centroidForPoly(List<float> verts)
        {
            var sum = 0.0f;
            var vsum = cpVect.ZERO;

            for (int i = 0, len = verts.Count; i < len; i += 2)
            {
                var v1 = new cpVect(verts[i], verts[i + 1]);
                var v2 = new cpVect(verts[(i + 2) % len], verts[(i + 3) % len]);
                var cross = cpVect.cpvcross(v1, v2);

                sum += cross;
                vsum = vsum.Add(v1.Add(v2).Multiply(cross)); //  vadd(vsum, vmult(vadd(v1, v2), cross));
            }

            return vsum.Multiply(1 / (3 * sum));
        }

        public static void recenterPoly(ref List<float> verts)
        {
            var centroid = centroidForPoly(verts);

            for (var i = 0; i < verts.Count; i += 2)
            {
                verts[i] -= centroid.x;
                verts[i + 1] -= centroid.y;
            }
        }

        public static float momentForBox2(float m, cpBB box)
        {
            var width = box.r - box.l;
            var height = box.t - box.b;
            var offset = new cpVect(box.l + box.r, box.b + box.t).Multiply(0.5f);

            // TODO NaN when offset is 0 and m is INFINITY	
            return momentForBox(m, width, height) + m * offset.LengthSQ;
        }
        public static float momentForBox(float m, float width, float height)
        {
            return m * (width * width + height * height) / 12;
        }

        //public static float momentForBox2(float m, cpBB box)
        //{
        //    var width = box.r - box.l;
        //    var height = box.t - box.b;
        //    var offset = new cpVect(box.l + box.r, box.b + box.t).Multiply(0.5f);

        //    // TODO NaN when offset is 0 and m is INFINITY	
        //    return momentForBox(m, width, height) + m * offset.LengthSQ;
        //}

        //public static float momentForBox(float m, float width, float height)
        //{
        //    return m * (width * width + height * height) / 12;
        //}

        #endregion

        public static int numLeaves { get; set; }

        internal static void AssertSoft(bool p1, string p2)
        {
            throw new NotImplementedException();
        }

        public static int numNodes { get; set; }

        public static int numPairs { get; set; }
    }
}
