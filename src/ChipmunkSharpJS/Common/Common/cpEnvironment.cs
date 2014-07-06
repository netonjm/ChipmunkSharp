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

using ChipmunkSharp.Constraints;
using ChipmunkSharp.Shapes;
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

    public class cp
    {

        // Chipmunk 6.2.1
        public static string CP_VERSION_MAJOR = "6";
        public static string CP_VERSION_MINOR = "2";
        public static string CP_VERSION_RELEASE = "1";

        public static string cpVersionString = string.Format("{0}.{1}.{2}", CP_VERSION_MAJOR, CP_VERSION_MINOR, CP_VERSION_RELEASE);


        public static int numLeaves { get; set; }
        public static int numNodes { get; set; }
        public static int numPairs { get; set; }

        public static int shapeIDCounter = 0;
        public static int CP_USE_CGPOINTS = 1;

        public static int NO_GROUP = 0;
        public static int ALL_LAYERS = ~0;

        public static int numApplyImpulse = 0;
        public static int numApplyContact = 0;

        public static float scale;

        public static byte[] INFINITY = { 0x00, 0x00, 0x80, 0x7F };

        public static float MAGIC_EPSILON = 1e-5F;

        public static float Infinity
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

        public static string hashPair(string a, string b)
        {
            return Convert.ToInt32(a) < Convert.ToInt32(b) ? a + " " + b : b + " " + a;
        }

        //public static int CP_HASH_PAIR(cpPolyShape poly, int B)
        //{
        //    return poly.hashid ^ B;
        //}
        //public static int CP_HASH_PAIR(cpSegmentShape seg, int B)
        //{
        //    return seg.hashid ^ B;
        //}

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

        #region ASSERTS



        public static void assert(string p2)
        {
            LogWrite(string.Format("Assert:{0}", p2));
        }

        public static void assert(bool p1, string p2)
        {
            if (!p1)
                LogWrite(string.Format("Assert:{0} Value:{1}", p2, p1));
        }

        public static void assertHard(bool p1, string p2)
        {
            if (!p1)
                LogWrite(string.Format("AssertHard:{0} Value:{1}", p2, p1));
        }

        public static void assertHard(string p)
        {
            LogWrite(string.Format("AssertHard:{0} Value:{1}", p, ""));
        }

        public static void assertSoft(bool p1, string p2)
        {
            if (!p1)
                LogWrite(string.Format("cpAssertSoft:{0} Value:{1}", p2, p1));
        }

        public static void assertSpaceUnlocked(cpSpace space)
        {
            assertSoft(!space.isLocked, "This addition/removal cannot be done safely during a call to cpSpaceStep() or during a query. Put these calls into a post-step callback.");
        }

        public static void assertWarn(bool p1, string p2)
        {
            if (!p1)
                LogWrite(string.Format("AssertWarn:{0} Value:{1}", p2, p1));
        }

        public static void assertWarn(string p)
        {
            LogWrite(string.Format("AssertWarn:{0}", p));
        }

        public static void assertWarn(bool p)
        {
            if (!p)
                LogWrite(string.Format("AssertWarn: ERROR DETECTED"));
        }

        public static void LogWrite(string message)
        {
            // Console.WriteLine(message);
        }

        #endregion

        public static float PHYSICS_INFINITY { get { return Infinity; } }

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

        public static float areaForPoly(float[] verts)
        {
            float area = 0.0f;
            for (int i = 0, len = verts.Length; i < len; i += 2)
            {
                area += cpVect.cpvcross(new cpVect(verts[i], verts[i + 1]), new cpVect(verts[(i + 2) % len], verts[(i + 3) % len]));
            }

            return -area / 2;
        }

        public static cpVect centroidForPoly(float[] verts)
        {
            float sum = 0;
            var vsum = new cpVect(0, 0);

            for (int i = 0, len = verts.Length; i < len; i += 2)
            {
                var v1 = new cpVect(verts[i], verts[i + 1]);
                var v2 = new cpVect(verts[(i + 2) % len], verts[(i + 3) % len]);
                var cross = cpVect.cpvcross(v1, v2);

                sum += cross;
                vsum = cpVect.cpvadd(vsum, cpVect.cpvmult(cpVect.cpvadd(v1, v2), cross));
            }

            return cpVect.cpvmult(vsum, 1 / (3 * sum));
        }

        public static void recenterPoly(float[] verts)
        {
            var centroid = centroidForPoly(verts);

            for (var i = 0; i < verts.Length; i += 2)
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

        public static void unlinkThread(Pair prev, Node leaf, Pair next)
        {
            if (next != null)
            {
                if (next.leafA == leaf)
                    next.prevA = prev;
                else next.prevB = prev;
            }

            if (prev != null)
            {
                if (prev.leafA == leaf) prev.nextA = next;
                else prev.nextB = next;
            }
            else
            {
                leaf.pairs = next;
            }
        }

        public static void pairInsert(Node a, Node b, cpBBTree tree)
        {
            Pair nextA = a.pairs, nextB = b.pairs;
            var pair = tree.MakePair(a, nextA, b, nextB);
            a.pairs = b.pairs = pair;

            if (nextA != null)
            {
                if (nextA.leafA == a) nextA.prevA = pair; else nextA.prevB = pair;
            }

            if (nextB != null)
            {
                if (nextB.leafA == b) nextB.prevA = pair; else nextB.prevB = pair;
            }
        }

        public static Node partitionNodes(cpBBTree tree, Dictionary<int, Leaf> nodes, int offset, int count)
        {
            //int count = nodes.Count;
            //int offset = 0;

            if (count == 1)
            {
                return nodes[0];
            }
            else if (count == 2)
            {
                return tree.makeNode(nodes[offset], nodes[offset + 1]);
            }

            // Find the AABB for these nodes
            //var bb = nodes[offset].bb;
            Leaf node = nodes[offset];
            float bb_l = node.bb_l,
        bb_b = node.bb_b,
        bb_r = node.bb_r,
        bb_t = node.bb_t;


            var end = offset + count;
            for (var i = offset + 1; i < end; i++)
            {
                //bb = bbMerge(bb, nodes[i].bb);
                node = nodes[i];
                bb_l = Math.Min(bb_l, node.bb_l);
                bb_b = Math.Min(bb_b, node.bb_b);
                bb_r = Math.Max(bb_r, node.bb_r);
                bb_t = Math.Max(bb_t, node.bb_t);
            }

            // Split it on it's longest axis
            var splitWidth = (bb_r - bb_l > bb_t - bb_b);

            // Sort the bounds and use the median as the splitting point
            float[] bounds = new float[count * 2];
            if (splitWidth)
            {
                for (var i = offset; i < end; i++)
                {
                    bounds[2 * i + 0] = nodes[i].bb_l;
                    bounds[2 * i + 1] = nodes[i].bb_r;
                }
            }
            else
            {
                for (var i = offset; i < end; i++)
                {
                    bounds[2 * i + 0] = nodes[i].bb_b;
                    bounds[2 * i + 1] = nodes[i].bb_t;
                }
            }

            //TODO: ¿?


            float split = (bounds[count - 1] + bounds[count]) * 0.5f; // use the median as the split

            // Generate the child BBs
            //var a = bb, b = bb;
            float a_l = bb_l, a_b = bb_b, a_r = bb_r, a_t = bb_t;
            float b_l = bb_l, b_b = bb_b, b_r = bb_r, b_t = bb_t;

            if (splitWidth) a_r = b_l = split; else a_t = b_b = split;

            // Partition the nodes
            var right = end;

            for (var left = offset; left < right; )
            {
                node = nodes[left];
                //	if(bbMergedArea(node.bb, b) < bbMergedArea(node.bb, a)){
                if (bbTreeMergedArea2(node, b_l, b_b, b_r, b_t) < bbTreeMergedArea2(node, a_l, a_b, a_r, a_t))
                {
                    right--;
                    nodes[left] = nodes[right];
                    nodes[right] = node;
                }
                else
                {
                    left++;
                }
            }

            if (right == count)
            {
                Node tmp = null;
                for (var i = offset; i < end; i++)
                    tmp = cp.subtreeInsert(tmp, nodes[i], tree);
                return node;
            }

            // Recurse and build the node!
            return new Node(
                partitionNodes(tree, nodes, offset, right - offset),
                partitionNodes(tree, nodes, right, end - right),
                tree
            );


        }

        public static float bbTreeMergedArea2(Node node, float l, float b, float r, float t)
        {
            return (Math.Max(node.bb_r, r) - Math.Min(node.bb_l, l)) * (Math.Max(node.bb_t, t) - Math.Min(node.bb_b, b));
            //return (Math.Max(node.bb.r, r) - Math.Min(node.bb.l, l)) * (Math.Max(node.bb.t, t) - Math.Min(node.bb.b, b));
        }

        public static float bbProximity(Node a, Leaf b)
        {
            return Math.Abs(a.bb_l + a.bb_r - b.bb_l - b.bb_r) + Math.Abs(a.bb_b + a.bb_t - b.bb_b - b.bb_t);
            //return Math.Abs(a.bb.l + a.bb.r - b.bb.l - b.bb.r) + Math.Abs(a.bb.b + a.bb.t - b.bb.b - b.bb.t);
        }

        public static void nodeRender(Node node, int depth)
        {
            if (!node.isLeaf && depth <= 10)
            {
                nodeRender(node.A, depth + 1);
                nodeRender(node.B, depth + 1);
            }

            var str = "";
            for (var i = 0; i < depth; i++)
            {
                str += " ";
            }

            LogWrite(str + node.bb_b + " " + node.bb_t);
        }

        public static Node SubtreeRemove(Node subtree, Leaf leaf, cpBBTree tree)
        {
            if (leaf == subtree)
            {
                return null;
            }
            else
            {
                var parent = leaf.parent;
                if (parent == subtree)
                {
                    var other = subtree.otherChild(leaf);
                    other.parent = subtree.parent;
                    subtree.recycle(tree);
                    return other;
                }
                else
                {
                    if (parent == null)
                        return null;

                    parent.parent.ReplaceChild(parent, parent.otherChild(leaf), tree);
                    return subtree;
                }
            }
        }

        public static int numContacts { get; set; }

        public static int step { get; set; }

        public static void apply_bias_impulse(cpBody body, float jx, float jy, cpVect r)
        {
            //body.v_bias = vadd(body.v_bias, vmult(j, body.m_inv));
            body.v_bias.x += jx * body.m_inv;
            body.v_bias.y += jy * body.m_inv;
            body.w_bias += body.i_inv * cpVect.cpvcross2(r.x, r.y, jx, jy);
        }

        public static void unthreadHelper(cpArbiter arb, cpBody body, cpArbiter prev, cpArbiter next)
        {
            // thread_x_y is quite ugly, but it avoids making unnecessary js objects per arbiter.
            if (prev != null)
            {
                // cpArbiterThreadForBody(prev, body)->next = next;
                if (prev.body_a == body)
                {
                    prev.thread_a_next = next;
                }
                else
                {
                    prev.thread_b_next = next;
                }
            }
            else
            {
                body.arbiterList = next;
            }

            if (next != null)
            {
                // cpArbiterThreadForBody(next, body)->prev = prev;
                if (next.body_a == body)
                {
                    next.thread_a_prev = prev;
                }
                else
                {
                    next.thread_b_prev = prev;
                }
            }
        }

        public static cpConstraint filterConstraints(cpConstraint node, cpBody body, cpConstraint filter)
        {
            if (node == filter)
            {
                return node.next(body);
            }
            else if (node.a == body)
            {
                node.next_a = filterConstraints(node.next_a, body, filter);
            }
            else
            {
                node.next_b = filterConstraints(node.next_b, body, filter);
            }

            return node;
        }

        public static cpBody componentRoot(cpBody body)
        {
            return (body != null ? body.nodeRoot : null);
        }

        public static void componentActivate(cpBody root)
        {
            if (root == null || !root.isSleeping()) return;
            cp.assertHard(!root.isRogue(), "Internal Error: componentActivate() called on a rogue body.");

            var space = root.space;
            cpBody body = root;
            while (body != null)
            {
                var next = body.nodeNext;

                body.nodeIdleTime = 0;
                body.nodeRoot = null;
                body.nodeNext = null;

                space.activateBody(body);

                body = next;
            }


            space.sleepingComponents.Remove(root);
        }


        public static void floodFillComponent(cpBody root, cpBody body)
        {
            // Rogue bodies cannot be put to sleep and prevent bodies they are touching from sleeping anyway.
            // Static bodies (which are a type of rogue body) are effectively sleeping all the time.
            if (!body.isRogue())
            {
                var other_root = componentRoot(body);
                if (other_root == null)
                {
                    componentAdd(root, body);
                    for (var arb = body.arbiterList; arb != null; arb = arb.next(body))
                    {
                        floodFillComponent(root, (body == arb.body_a ? arb.body_b : arb.body_a));
                    }
                    for (var constraint = body.constraintList; constraint != null; constraint = constraint.next(body))
                    {
                        floodFillComponent(root, (body == constraint.a ? constraint.b : constraint.a));
                    }
                }
                else
                {
                    assertSoft(other_root == root, "Internal Error: Inconsistency detected in the contact graph.");
                }
            }
        }

        public static void componentAdd(cpBody root, cpBody body)
        {
            body.nodeRoot = root;

            if (body != root)
            {
                body.nodeNext = root.nodeNext;
                root.nodeNext = body;
            }
        }

        public static bool componentActive(cpBody root, float threshold)
        {
            for (var body = root; body != null; body = body.nodeNext)
            {
                if (body.nodeIdleTime < threshold)
                    return true;
            }

            return false;
        }

        public static CollisionHandler defaultCollisionHandler = new CollisionHandler();

        public static void updateFunc(cpShape shape)
        {
            var body = shape.body;
            shape.update(body.Position, body.Rotation);
        }



        //// **** All Important cpSpaceStep() Function


        /// Returns true if @c a and @c b intersect.

        public static bool bbIntersects(cpBB a, cpBB b)
        {
            return (a.l <= b.r && b.l <= a.r && a.b <= b.t && b.b <= a.t); ;
        }

        public static bool bbIntersects2(cpBB bb, float l, float b, float r, float t)
        {
            return (bb.l <= r && l <= bb.r && bb.b <= t && b <= bb.t);
        }

        public static float bbProximity(Node a, Node b)
        {
            return Math.Abs(a.bb_l + a.bb_r - b.bb_l - b.bb_r) + Math.Abs(a.bb_b + a.bb_t - b.bb_b - b.bb_t);
            // return Math.Abs(a.bb.l + a.bb.r - b.bb.l - b.bb.r) + Math.Abs(a.bb.b + a.bb.t - b.bb.b - b.bb.t);
        }

        public static float bbTreeMergedArea(Node a, Node b)
        {
            return (Math.Max(a.bb_r, b.bb_r) - Math.Min(a.bb_l, b.bb_l)) * (Math.Max(a.bb_t, b.bb_t) - Math.Min(a.bb_b, b.bb_b));
            //return (Math.Max(a.bb.r, b.bb.r) - Math.Min(a.bb.l, b.bb.l)) * (Math.Max(a.bb.t, b.bb.t) - Math.Min(a.bb.b, b.bb.b));
        }


        public static bool bbTreeIntersectsNode(Node a, Node b)
        {
            return (a.bb_l <= b.bb_r && b.bb_l <= a.bb_r && a.bb_b <= b.bb_t && b.bb_b <= a.bb_t);
            //return (a.bb.l <= b.bb.r && b.bb.l <= a.bb.r && a.bb.b <= b.bb.t && b.bb.b <= a.bb.t);
        }


        public static Node subtreeInsert(Node subtree, Leaf leaf, cpBBTree tree)
        {
            //	var s = new Error().stack;
            //	traces[s] = traces[s] ? traces[s]+1 : 1;

            if (subtree == null)
            {
                return leaf;
            }
            else if (subtree.isLeaf)
            {
                return tree.makeNode(leaf, subtree);
            }
            else
            {
                var cost_a = subtree.B.bbArea() + bbTreeMergedArea(subtree.A, leaf);
                var cost_b = subtree.A.bbArea() + bbTreeMergedArea(subtree.B, leaf);

                if (cost_a == cost_b)
                {
                    cost_a = bbProximity(subtree.A, leaf);
                    cost_b = bbProximity(subtree.B, leaf);
                }

                if (cost_b < cost_a)
                {
                    subtree.setB(subtreeInsert(subtree.B, leaf, tree));
                }
                else
                {
                    subtree.setA(subtreeInsert(subtree.A, leaf, tree));
                }

                //		subtree.bb = bbMerge(subtree.bb, leaf.bb);
                subtree.bb_l = Math.Min(subtree.bb_l, leaf.bb_l);
                subtree.bb_b = Math.Min(subtree.bb_b, leaf.bb_b);
                subtree.bb_r = Math.Max(subtree.bb_r, leaf.bb_r);
                subtree.bb_t = Math.Max(subtree.bb_t, leaf.bb_t);


                return subtree;
            }
        }

        /// Check that a set of vertexes is convex and has a clockwise winding.
        public static bool polyValidate(float[] verts)
        {
            var len = verts.Length;
            for (var i = 0; i < len; i += 2)
            {
                var ax = verts[i];
                var ay = verts[i + 1];
                var bx = verts[(i + 2) % len];
                var by = verts[(i + 3) % len];
                var cx = verts[(i + 4) % len];
                var cy = verts[(i + 5) % len];

                //if(vcross(vsub(b, a), vsub(c, b)) > 0){
                if (cpVect.cpvcross2(bx - ax, by - ay, cx - bx, cy - by) > 0)
                {
                    return false;
                }
            }

            return true;
        }


        internal static cpVect closestPointOnSegment2(float px, float py, float ax, float ay, float bx, float by)
        {
            var deltax = ax - bx;
            var deltay = ay - by;
            var t = cpfclamp01(cpVect.cpvdot2(deltax, deltay, px - bx, py - by) / cpVect.vlengthsq2(deltax, deltay));
            return new cpVect(bx + deltax * t, by + deltay * t);
        }


        public static cpPolyShape BoxShape(cpBody body, float width, float height)
        {
            var hw = width / 2;
            var hh = height / 2;

            return BoxShape2(body, new cpBB(-hw, -hh, hw, hh));
        }
        public static cpPolyShape BoxShape2(cpBody body, cpBB box)
        {
            float[] verts = new float[] {
		box.l, box.b,
		box.l, box.t,
		box.r, box.t,
		box.r, box.b};

            return new cpPolyShape(body, verts, cpVect.ZERO);
        }





        internal static cpSegmentQueryInfo circleSegmentQuery(cpShape shape, cpVect center, float r, cpVect a, cpVect b)
        {
            // offset the line to be relative to the circle
            a = cpVect.cpvsub(a, center);
            b = cpVect.cpvsub(b, center);

            var qa = cpVect.cpvdot(a, a) - 2 * cpVect.cpvdot(a, b) + cpVect.cpvdot(b, b);
            var qb = -2 * cpVect.cpvdot(a, a) + 2 * cpVect.cpvdot(a, b);
            var qc = cpVect.cpvdot(a, a) - r * r;

            var det = qb * qb - 4 * qa * qc;

            if (det >= 0)
            {
                var t = (-qb - cp.cpfsqrt(det)) / (2 * qa);
                if (0 <= t && t <= 1)
                {
                    return new cpSegmentQueryInfo(shape, t, cpVect.cpvnormalize(cpVect.cpvlerp(a, b, t)));
                }

            }
            return cpSegmentQueryInfo.CreateBlanck();
        }

        public static cpVect closestPointOnSegment(cpVect p, cpVect a, cpVect b)
        {
            var delta = cpVect.cpvsub(a, b);
            var t = cp.cpfclamp01(cpVect.cpvdot(delta, cpVect.cpvsub(p, b)) / cpVect.cpvlengthsq(delta));
            return cpVect.cpvadd(b, cpVect.cpvmult(delta, t));
        }

        public static float segValueOnAxis(cpSegmentShape seg, cpVect n, float d)
        {
            // Like cpPolyValueOnAxis(), but for segments.
            var a = cpVect.cpvdot(n, seg.ta) - seg.r;
            var b = cpVect.cpvdot(n, seg.tb) - seg.r;
            return Math.Min(a, b) - d;
        }

        public static void findPointsBehindSeg(List<ContactPoint> arr, cpSegmentShape seg, cpPolyShape poly, float pDist, int coef)
        {
            var dta = cpVect.cpvcross(seg.tn, seg.ta);
            var dtb = cpVect.cpvcross(seg.tn, seg.tb);
            var n = cpVect.cpvmult(seg.tn, coef);

            var verts = poly.tVerts;
            for (var i = 0; i < verts.Length; i += 2)
            {
                var vx = verts[i];
                var vy = verts[i + 1];
                if (cpVect.cpvdot2(vx, vy, n.x, n.y) < cpVect.cpvdot(seg.tn, seg.ta) * coef + seg.r)
                {
                    var dt = cpVect.cpvcross2(seg.tn.x, seg.tn.y, vx, vy);
                    if (dta >= dt && dt >= dtb)
                    {
                        arr.Add(new ContactPoint(new cpVect(vx, vy), n, pDist, hashPair(poly.hashid, i.ToString())));
                    }
                }
            }
        }

        public static int GRABABLE_MASK_BIT { get { return (1 << 31); } }
        public static int NOT_GRABABLE_MASK { get { return ~GRABABLE_MASK_BIT; } }

        //public static Node SubtreeInsert(Node subtree, Leaf leaf, cpBBTree tree)
        //{
        //    //	var s = new Error().stack;
        //    //	traces[s] = traces[s] ? traces[s]+1 : 1;

        //    if (subtree == null)
        //    {
        //        return leaf;
        //    }
        //    else if (subtree.isLeaf)
        //    {
        //        return tree.makeNode(leaf, subtree);
        //    }
        //    else
        //    {
        //        var cost_a = subtree.B.bbArea() + bbTreeMergedArea(subtree.A, leaf);
        //        var cost_b = subtree.A.bbArea() + bbTreeMergedArea(subtree.B, leaf);

        //        if (cost_a == cost_b)
        //        {
        //            cost_a = bbProximity(subtree.A, leaf);
        //            cost_b = bbProximity(subtree.B, leaf);
        //        }

        //        if (cost_b < cost_a)
        //        {
        //            subtree.setB(subtreeInsert(subtree.B, leaf, tree));
        //        }
        //        else
        //        {
        //            subtree.setA(subtreeInsert(subtree.A, leaf, tree));
        //        }

        //        //		subtree.bb = bbMerge(subtree.bb, leaf.bb);
        //        subtree.bb_l = Math.Min(subtree.bb_l, leaf.bb_l);
        //        subtree.bb_b = Math.Min(subtree.bb_b, leaf.bb_b);
        //        subtree.bb_r = Math.Max(subtree.bb_r, leaf.bb_r);
        //        subtree.bb_t = Math.Max(subtree.bb_t, leaf.bb_t);

        //        return subtree;
        //    }
        //}


        public static float[] convexHull(float[] verts, float[] result, float tolerance)
        {
            if (result != null)
            {
                // Copy the line vertexes into the empty part of the result polyline to use as a scratch buffer.
                for (var i = 0; i < verts.Length; i++)
                {
                    result[i] = verts[i];
                }
            }
            else
            {
                // If a result array was not specified, reduce the input instead.
                result = verts;
            }

            // Degenerate case, all points are the same.
            int[] indexes = loopIndexes(verts);
            int start = indexes[0], end = indexes[1];

            int position;
            float[] dev;

            if (start == end)
            {
                //if(first) (*first) = 0;
                position = 2;
                dev = new float[position];
                for (int i = 0; i < position; i++)
                    dev[i] = result[i];
                return dev;
            }

            SWAP(result, 0, start);
            SWAP(result, 1, end == 0 ? start : end);

            var a = new cpVect(result[0], result[1]);
            var b = new cpVect(result[2], result[3]);

            var count = verts.Length >> 1;
            //if(first) (*first) = start;
            var resultCount = QHullReduce(tolerance, result, 2, count - 2, a, b, a, 1) + 1;

            position = resultCount * 2;

            dev = new float[position];
            for (int i = 0; i < position; i++)
                dev[i] = result[i];

            assertSoft(polyValidate(result),
                "Internal error: cpConvexHull() and cpPolyValidate() did not agree." +
                "Please report this error with as much info as you can.");
            return dev;
        }

        public static int QHullReduce(float tol, float[] verts, int offs, int count, cpVect a, cpVect pivot, cpVect b, int resultPos)
        {
            if (count < 0)
            {
                return 0;
            }
            else if (count == 0)
            {
                verts[resultPos * 2] = pivot.x;
                verts[resultPos * 2 + 1] = pivot.y;
                return 1;
            }
            else
            {
                var left_count = QHullPartition(verts, offs, count, a, pivot, tol);
                var left = new cpVect(verts[offs * 2], verts[offs * 2 + 1]);
                var index = QHullReduce(tol, verts, offs + 1, left_count - 1, a, left, pivot, resultPos);

                var pivotPos = resultPos + index++;
                verts[pivotPos * 2] = pivot.x;
                verts[pivotPos * 2 + 1] = pivot.y;

                var right_count = QHullPartition(verts, offs + left_count, count - left_count, pivot, b, tol);
                var right = new cpVect(verts[(offs + left_count) * 2], verts[(offs + left_count) * 2 + 1]);
                return index + QHullReduce(tol, verts, offs + left_count + 1, right_count - 1, pivot, right, b, resultPos + index);
            }
        }

        private static int QHullPartition(float[] verts, int offs, int count, cpVect a, cpVect b, float tol)
        {
            if (count == 0) return 0;

            float max = 0;
            var pivot = offs;

            var delta = cpVect.cpvsub(b, a);
            var valueTol = tol * cpVect.cpvlength(delta);

            var head = offs;
            for (var tail = offs + count - 1; head <= tail; )
            {
                var v = new cpVect(verts[head * 2], verts[head * 2 + 1]);
                float value = cpVect.cpvcross(delta, cpVect.cpvsub(v, a));
                if (value > valueTol)
                {
                    if (value > max)
                    {
                        max = value;
                        pivot = head;
                    }

                    head++;
                }
                else
                {
                    SWAP(verts, head, tail);
                    tail--;
                }
            }

            // move the new pivot to the front if it's not already there.
            if (pivot != offs) SWAP(verts, offs, pivot);
            return head - offs;
        }

        public static int[] loopIndexes(float[] verts)
        {
            int start = 0, end = 0;
            float minx, miny, maxx, maxy;
            minx = maxx = verts[0];
            miny = maxy = verts[1];

            var count = verts.Length >> 1;
            for (var i = 1; i < count; i++)
            {
                var x = verts[i * 2];
                var y = verts[i * 2 + 1];

                if (x < minx || (x == minx && y < miny))
                {
                    minx = x;
                    miny = y;
                    start = i;
                }
                else if (x > maxx || (x == maxx && y > maxy))
                {
                    maxx = x;
                    maxy = y;
                    end = i;
                }
            }
            return new int[] { start, end };
        }

        public static void SWAP(float[] arr, int idx1, int idx2)
        {
            var tmp = arr[idx1 * 2];
            arr[idx1 * 2] = arr[idx2 * 2];
            arr[idx2 * 2] = tmp;

            tmp = arr[idx1 * 2 + 1];
            arr[idx1 * 2 + 1] = arr[idx2 * 2 + 1];
            arr[idx2 * 2 + 1] = tmp;
        }

        public static float momentForPoly(float m, float[] verts, cpVect offset)
        {
            float sum1 = 0f;
            float sum2 = 0f;
            int len = verts.Length;
            for (var i = 0; i < len; i += 2)
            {
                var v1x = verts[i] + offset.x;
                var v1y = verts[i + 1] + offset.y;
                var v2x = verts[(i + 2) % len] + offset.x;
                var v2y = verts[(i + 3) % len] + offset.y;

                var a = cpVect.cpvcross2(v2x, v2y, v1x, v1y);
                var b = cpVect.cpvdot2(v1x, v1y, v1x, v1y) + cpVect.cpvdot2(v1x, v1y, v2x, v2y) + cpVect.cpvdot2(v2x, v2y, v2x, v2y);

                sum1 += a * b;
                sum2 += a;
            }

            return (m * sum1) / (6 * sum2);
        }


        public static List<cpColor> _styles;


        public static int randColor()
        {
            return new Random(DateTime.Now.Millisecond).Next(0, 255);
        }

        public static List<cpColor> styles
        {
            get
            {

                if (_styles == null)
                {
                    _styles = new List<cpColor>();
                    for (var i = 0; i < 100; i++)
                    {
                        styles.Add(new cpColor(randColor(), randColor(), randColor()));
                    }
                }

                return _styles;

            }
            set
            {
                _styles = value;
            }
        }

        public static cpVect canvas2point(float x, float y, float scale)
        {
            return new cpVect(x / scale, 480 - y / scale);
        }

        public static cpColor GetShapeColor(cpShape shape)
        {

            if (shape.sensor)
                return new cpColor(255, 255, 255);
            else
            {

                if (shape.body.isSleeping())
                {
                    return new cpColor(50, 50, 50);
                }
                else if (shape.body.nodeIdleTime > shape.space.sleepTimeThreshold)
                {
                    return new cpColor(170, 170, 170);
                }
                else
                {
                    return styles[int.Parse(shape.hashid) % styles.Count];
                }
            }
        }

        public static cpVect point2canvas(cpVect point, float scale)
        {
            return new cpVect(point.x * scale, (480 - point.y) * scale);
        }

        public static float last_MSA_min = 0;

        public static float findMSA(cpPolyShape poly, cpSplittingPlane[] planes)
        {
            float min_index = 0;
            var min = poly.valueOnAxis(planes[0].n, planes[0].d);
            if (min > 0) return -1;

            for (var i = 1; i < planes.Length; i++)
            {
                var dist = poly.valueOnAxis(planes[i].n, planes[i].d);
                if (dist > 0)
                {
                    return -1;
                }
                else if (dist > min)
                {
                    min = dist;
                    min_index = i;
                }
            }

            last_MSA_min = min;
            return min_index;
        }


        public static List<ContactPoint> findVerts(cpPolyShape poly1, cpPolyShape poly2, cpVect n, float dist)
        {
            List<ContactPoint> arr = new List<ContactPoint>();

            var verts1 = poly1.tVerts;
            for (var i = 0; i < verts1.Length; i += 2)
            {
                var vx = verts1[i];
                var vy = verts1[i + 1];
                if (poly2.containsVert(vx, vy))
                {
                    arr.Add(new ContactPoint(new cpVect(vx, vy), n, dist, cp.hashPair(poly1.hashid, (i >> 1).ToString())));
                }
            }

            var verts2 = poly2.tVerts;
            for (var i = 0; i < verts2.Length; i += 2)
            {
                var vx = verts2[i];
                var vy = verts2[i + 1];
                if (poly1.containsVert(vx, vy))
                {
                    arr.Add(new ContactPoint(new cpVect(vx, vy), n, dist, cp.hashPair(poly2.hashid, (i >> 1).ToString())));
                }
            }

            return (arr.Count > 0 ? arr : cp.findVertsFallback(poly1, poly2, n, dist));



        }

        public static List<ContactPoint> findVertsFallback(cpPolyShape poly1, cpPolyShape poly2, cpVect n, float dist)
        {
            List<ContactPoint> arr = new List<ContactPoint>();


            var verts1 = poly1.tVerts;
            for (var i = 0; i < verts1.Length; i += 2)
            {
                var vx = verts1[i];
                var vy = verts1[i + 1];
                if (poly2.containsVertPartial(vx, vy, cpVect.cpvneg(n)))
                {
                    arr.Add(new ContactPoint(new cpVect(vx, vy), n, dist, cp.hashPair(poly1.hashid, i.ToString())));
                }
            }

            var verts2 = poly2.tVerts;
            for (var i = 0; i < verts2.Length; i += 2)
            {
                var vx = verts2[i];
                var vy = verts2[i + 1];
                if (poly1.containsVertPartial(vx, vy, n))
                {
                    arr.Add(new ContactPoint(new cpVect(vx, vy), n, dist, cp.hashPair(poly2.hashid, i.ToString())));
                }
            }

            return arr;
        }

        internal static cpBB bbNewForCircle(cpVect point, int maxDistance)
        {
            throw new NotImplementedException();
        }
    }
}