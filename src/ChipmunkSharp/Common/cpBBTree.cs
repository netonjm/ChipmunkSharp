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
using System.Threading;
using System.Linq;

namespace ChipmunkSharp
{

    //MARK: Spatial Index

    /// Spatial index bounding box callback function type.
    /// The spatial index calls this function and passes you a pointer to an object you added
    /// when it needs to get the bounding box associated with that object.
    public delegate cpBB cpSpatialIndexBBFunc(object obj);
    /// Spatial index/object iterator callback function type.
    public delegate void cpSpatialIndexIteratorFunc(object obj, object data);
    /// Spatial query callback function type.
    public delegate int cpSpatialIndexQueryFunc(object obj1, object obj2, int id, object data);
    /// Spatial segment query callback function type.
    public delegate float cpSpatialIndexSegmentQueryFunc(object obj1, object obj2, object data);



    //MARK: Spatial Index Implementation

    public delegate void cpSpatialIndexDestroyImpl(cpBBTree index);
    public delegate int cpSpatialIndexCountImpl(cpBBTree index);
    public delegate void cpSpatialIndexEachImpl(cpBBTree index, cpSpatialIndexIteratorFunc func, object data);
    public delegate bool cpSpatialIndexContainsImpl(cpBBTree index, object obj, uint hashid);
    public delegate void cpSpatialIndexInsertImpl(cpBBTree index, object obj, uint hashid);
    public delegate void cpSpatialIndexRemoveImpl(cpBBTree index, object obj, uint hashid);
    public delegate void cpSpatialIndexReindexImpl(cpBBTree index);
    public delegate void cpSpatialIndexReindexObjectImpl(cpBBTree index, object obj, uint hashid);
    public delegate void cpSpatialIndexReindexQueryImpl(cpBBTree index, cpSpatialIndexQueryFunc func, object data);
    public delegate void cpSpatialIndexQueryImpl(cpBBTree index, object obj, cpBB bb, cpSpatialIndexQueryFunc func, object data);
    public delegate void cpSpatialIndexSegmentQueryImpl(cpBBTree index, object obj, cpVect a, cpVect b, float t_exit, cpSpatialIndexSegmentQueryFunc func, object data);

    public struct dynamicToStaticContext
    {

        public cpSpatialIndexBBFunc bbfunc;
        public cpBBTree staticIndex;
        public cpSpatialIndexQueryFunc queryFunc;
        public object data;


        public dynamicToStaticContext(cpSpatialIndexBBFunc cpSpatialIndexBBFunc, cpBBTree staticIndex1, cpSpatialIndexQueryFunc func, object data1)
        {
            // TODO: Complete member initialization
            this.bbfunc = cpSpatialIndexBBFunc;
            this.staticIndex = staticIndex1;
            this.queryFunc = func;
            this.data = data1;
        }
    }

    //public class cpHashSet : KeyValuePair<int, object>
    //{
    //    public int Id;
    //}

    public class MarkContext
    {
        public cpBBTree tree;
        public Node staticRoot;
        public cpSpatialIndexQueryFunc func;
        public object data;

        public MarkContext(cpBBTree tree, Node staticRoot, cpSpatialIndexQueryFunc func, object data)
        {
            this.tree = tree;
            this.staticRoot = staticRoot;
            this.func = func;
            this.data = data;

        }
    }


    /// Bounding box tree velocity callback function.
    /// This function should return an estimate for the object's velocity.
    public delegate cpVect cpBBTreeVelocityFunc(object obj);

    public class Node
    {
        public Node a, b;

        public uint stamp;
        public uint STAMP { get { return stamp; } set { stamp = value; } }

        public Node A { get { return a; } set { a = value; } }
        public Node B { get { return b; } set { b = value; } }

        public object obj;
        public cpBB bb;
        public Node parent;

        public Pair pairs;


        public Pair PAIRS { get { return pairs; } set { pairs = value; } }

        public bool IsLeaf()
        {
            return (obj != null);
        }

        public void MarkSubtree(MarkContext context)
        {
            if (IsLeaf())
            {
                MarkLeaf(context);
            }
            else
            {
                A.MarkSubtree(context);
                B.MarkSubtree(context);
            }
        }

        //MARK: Marking Functions

        public void MarkLeafQuery(Node leaf, bool left, MarkContext context)
        {
            if (cpBB.Intersects(leaf.bb, this.bb))
            {
                if (this.IsLeaf())
                {
                    if (left)
                    {
                        context.tree.PairInsert(leaf, this);
                    }
                    else
                    {
                        if (this.STAMP < leaf.STAMP)
                            context.tree.PairInsert(this, leaf);

                        context.func(leaf.obj, this.obj, 0, context.data);
                    }
                }
                else
                {
                    this.A.MarkLeafQuery(leaf, left, context);
                    this.B.MarkLeafQuery(leaf, left, context);
                    //MarkLeafQuery(, );
                    //MarkLeafQuery(this.B, leaf, left, context);
                }
            }
        }

        public void MarkLeaf(MarkContext context)
        {
            cpBBTree tree = context.tree;
            if (STAMP == tree.GetMasterTree().stamp)
            {
                Node staticRoot = context.staticRoot;
                if (staticRoot != null)
                    staticRoot.MarkLeafQuery(this, false, context);

                for (Node node = this; node.parent != null; node = node.parent)
                {
                    if (node == node.parent.A)
                    {
                        node.parent.B.MarkLeafQuery(this, true, context);
                    }
                    else
                    {
                        node.parent.A.MarkLeafQuery(this, false, context);
                    }
                }
            }
            else
            {
                Pair pair = PAIRS;
                while (pair != null)
                {
                    if (this == pair.b.leaf)
                    {
                        pair.id = context.func(pair.a.leaf.obj, this.obj, pair.id, context.data);
                        pair = pair.b.next;
                    }
                    else
                    {
                        pair = pair.a.next;
                    }
                }
            }
        }



    }

    public class Leaf : Node
    {
        internal void Update(cpBBTree cpBBTree)
        {

        }
    }

    // Can't use anonymous unions and still get good x-compiler compatability
    //#define A node.children.a
    //#define B node.children.b
    //#define STAMP node.leaf.stamp
    //#define PAIRS node.leaf.pairs

    public struct Thread
    {

        public Pair prev;
        public Node leaf;
        public Pair next;

        public Thread(Pair prev, Node leaf, Pair next)
        {
            this.prev = prev;
            this.leaf = leaf;
            this.next = next;
        }

    }

    public class Pair
    {
        public Thread a, b;
        public int id;

        public Pair()
        {

        }

        public Pair(Thread a, Thread b, int id)
        {
            this.a = a;
            this.b = b;
            this.id = id;
        }

    };


    /**
        @defgroup cpSpatialIndex cpSpatialIndex
	
        Spatial indexes are data structures that are used to accelerate collision detection
        and spatial queries. Chipmunk provides a number of spatial index algorithms to pick from
        and they are programmed in a generic way so that you can use them for holding more than
        just cpShape structs.
	
        It works by using @c void pointers to the objects you add and using a callback to ask your code
        for bounding boxes when it needs them. Several types of queries can be performed an index as well
        as reindexing and full collision information. All communication to the spatial indexes is performed
        through callback functions.
	
        Spatial indexes should be treated as opaque structs.
        This meanns you shouldn't be reading any of the struct fields.
        @{
    */

    public class cpBBTree : Dictionary<int, object>
    {

        public void SetVelocityFunc(cpBBTreeVelocityFunc func)
        {
            velocityFunc = func;
        }


        public cpBBTreeVelocityFunc velocityFunc;

        public Dictionary<string, Leaf> leaves { get; set; }
        public Node root { get; set; }

        public Node pooledNodes { get; set; }
        public Pair pooledPairs { get; set; }
        public List<object> allocatedBuffers { get; set; }

        public uint stamp { get; set; }

        public cpSpatialIndexBBFunc bbfunc { get; set; }

        public cpBB bb { get; set; }

        // public cpBBTree spatialIndex { get; set; }
        public cpBBTree staticIndex { get; set; }
        public cpBBTree dynamicIndex { get; set; }

        public object defaultValue { get; set; }


        //MARK: Misc Functions

        public cpBBTree(cpBBTree staticIndex)
        {

            this.staticIndex = staticIndex;
            if (staticIndex != null)
            {
                if (staticIndex.dynamicIndex != null)
                {
                    cpEnvironment.cpAssertHard("This static index is already associated with a dynamic index.");
                }
                staticIndex.dynamicIndex = this;
            }
        }

        /// Get the number of objects in the spatial index.
        public static int cpSpatialIndexCount(cpBBTree index)
        {
            return index.Count;
        }

        //public static void cpSpatialIndexCollideStatic(cpSpatialIndex dynamicIndex, cpSpatialIndex staticIndex, cpSpatialIndexQueryFunc func, object data)
        //{
        //    if (staticIndex != null && cpSpatialIndexCount(staticIndex) > 0)
        //    {
        //        dynamicToStaticContext context = new dynamicToStaticContext(dynamicIndex.bbfunc, staticIndex, func, data);
        //        cpSpatialIndexEach(dynamicIndex, (cpSpatialIndexIteratorFunc)dynamicToStaticIter, context);
        //    }
        //}

        public cpBBTree GetMasterTree()
        {
            return (dynamicIndex != null ? dynamicIndex : this);
        }


        public static cpBBTree cpSpatialIndexInit(cpBBTree index, cpSpatialIndexBBFunc bbfunc, cpBBTree staticIndex)
        {
            //index.klass = klass;
            index.bbfunc = bbfunc;
            index.staticIndex = staticIndex;

            if (staticIndex != null)
            {
                cpEnvironment.cpAssertHard(staticIndex.dynamicIndex != null, "This static index is already associated with a dynamic index.");
                staticIndex.dynamicIndex = index;
            }

            return index;
        }


        //public void IndexEach(cpSpatialIndexIteratorFunc updateBBCache, object p)
        //{
        //    //     foreach (var item in collection)
        //    //   {
        //    //cpSpatialIndexIteratorFunc
        //    //   }//
        //    // throw new NotImplementedException();

        //    //foreach (var item in this)
        //    //{
        //    //    updateBBCache.BeginInvoke()
        //    //}

        //}

        //public void IndexQuery(PointQueryContext context, cpBB bb, Func<PointQueryContext, cpShape, int, object, int> PointQuery, object data)
        //{
        //    // throw new NotImplementedException();

        //    sub


        //}


        //public void SegmentQuery(void *obj, cpVect a, cpVect b, cpFloat t_exit, cpSpatialIndexSegmentQueryFunc func, void *data);


        //public void IndexSegmentQuery(object obj, cpVect a, cpVect b, float t_exit, cpSpatialIndexSegmentQueryFunc func, object data)
        //{
        //    func(obj, a, b, t_exit, func, data);
        //}



        /// Get the number of objects in the spatial index.
        //public static int cpSpatialIndexCount(cpBBTree index)
        //{
        //    //return index->klass->count(index);
        //    //return 

        //    return index.Count;


        //}

        //public static void cpSpatialIndexCollideStatic(cpSpatialIndex dynamicIndex, cpSpatialIndex staticIndex, cpSpatialIndexQueryFunc func, object data)
        //{
        //    if (staticIndex != null && cpSpatialIndexCount(staticIndex) > 0)
        //    {
        //        dynamicToStaticContext context = new dynamicToStaticContext(dynamicIndex.bbfunc, staticIndex, func, data);
        //        cpSpatialIndexEach(dynamicIndex, (cpSpatialIndexIteratorFunc)dynamicToStaticIter, context);
        //    }
        //}






        //internal void ReindexObject(int p, cpShape shape)
        //{
        //    throw new NotImplementedException();
        //}

        //internal void Reindex()
        //{
        //    throw new NotImplementedException();
        //}



        //public void IndexEach(cpSpatialIndexIteratorFunc updateBBCache, object p)
        //{
        //    //     foreach (var item in collection)
        //    //   {
        //    //cpSpatialIndexIteratorFunc
        //    //   }//
        //    // throw new NotImplementedException();

        //    //foreach (var item in this)
        //    //{
        //    //    updateBBCache.BeginInvoke()
        //    //}

        //}

        //public void IndexQuery(PointQueryContext context, cpBB bb, Func<PointQueryContext, cpShape, int, object, int> PointQuery, object data)
        //{
        //    // throw new NotImplementedException();

        //    sub


        //}


        //public void SegmentQuery(void *obj, cpVect a, cpVect b, cpFloat t_exit, cpSpatialIndexSegmentQueryFunc func, void *data);


        //public void IndexSegmentQuery(object obj, cpVect a, cpVect b, float t_exit, cpSpatialIndexSegmentQueryFunc func, object data)
        //{
        //    func(obj, a, b, t_exit, func, data);
        //}




        public cpBB GetBB(object obj)
        {

            //TODO: GETBB

            //cpBB bb = tree.bb;

            // cpBBTreeVelocityFunc velocityFunc = velocityFunc;
            if (velocityFunc != null)
            {
                float coef = 0.1f;
                float x = (bb.r - bb.l) * coef;
                float y = (bb.t - bb.b) * coef;

                cpVect v = cpVect.cpvmult(velocityFunc(obj), 0.1f);
                return cpBB.cpBBNew(bb.l + cpEnvironment.cpfmin(-x, v.x), bb.b + cpEnvironment.cpfmin(-y, v.y), bb.r + cpEnvironment.cpfmax(x, v.x), bb.t + cpEnvironment.cpfmax(y, v.y));
            }
            else
            {
                return bb;
            }
        }

        //public cpBBTree GetTree()
        //{
        //    return (index as cpBBTree);
        //}

        //public Node GetRootIfTree()
        //{
        //    //var tree = GetTree(index);
        //    //if (tree != null)
        //    return root;
        //}

        //public static cpBBTree GetMasterTree(cpBBTree tree)
        //{
        //   // cpBBTree dynamicTree = GetTree(tree.dynamicIndex);
        //    return (dynamicTree != null ? dynamicTree : tree);
        //}

        public void IncrementStamp()
        {
            //  cpBBTree dynamicTree = tree.dynamicIndex;
            if (dynamicIndex != null)
                dynamicIndex.stamp++;
            else
                stamp++;
        }

        //MARK: Pair/Thread Functions

        public void PairRecycle(Pair pair)
        {
            // Share the pool of the master tree.
            // TODO would be lovely to move the pairs stuff into an external data structure.
            //tree = GetMasterTree(tree);

            pair.a.next = pooledPairs;
            pooledPairs = pair;
        }

        public Pair PairFromPool()
        {
            // Share the pool of the master tree.
            // TODO: would be lovely to move the pairs stuff into an external data structure.
            // tree = GetMasterTree(tree);

            Pair pair = pooledPairs;

            if (pair != null)
            {
                pooledPairs = pair.a.next;
                return pair;
            }
            else
            {

                Pair buffer = new Pair(); // (Pair*)cpcalloc(1, CP_BUFFER_BYTES);
                allocatedBuffers.Add(buffer);
                PairRecycle(buffer);
                return buffer;
            }
        }

        public static void ThreadUnlink(Thread thread)
        {
            Pair next = thread.next;
            Pair prev = thread.prev;

            if (next != null)
            {
                if (next.a.leaf == thread.leaf) next.a.prev = prev; else next.b.prev = prev;
            }

            if (prev != null)
            {
                if (prev.a.leaf == thread.leaf) prev.a.next = next; else prev.b.next = next;
            }
            else
            {
                thread.leaf.pairs = next;
            }
        }

        public void PairsClear(Node leaf)//, cpBBTree tree)
        {
            Pair pair = leaf.pairs;
            leaf.pairs = null;

            while (pair != null)
            {
                if (pair.a.leaf == leaf)
                {
                    Pair next = pair.a.next;
                    ThreadUnlink(pair.b);
                    PairRecycle(pair);
                    pair = next;
                }
                else
                {
                    Pair next = pair.b.next;
                    ThreadUnlink(pair.a);
                    PairRecycle(pair);
                    pair = next;
                }
            }
        }

        public void PairInsert(Node a, Node b)
        {
            Pair nextA = a.pairs, nextB = b.pairs;
            Pair pair = PairFromPool();
            Pair temp = new Pair(new Thread(null, a, nextA), new Thread(null, b, nextB), 0);

            a.pairs = b.pairs = pair;
            pair = temp;

            if (nextA != null)
            {
                if (nextA.a.leaf == a) nextA.a.prev = pair; else nextA.b.prev = pair;
            }

            if (nextB != null)
            {
                if (nextB.a.leaf == b) nextB.a.prev = pair; else nextB.b.prev = pair;
            }
        }

        //MARK: Node Functions

        public void NodeRecycle(Node node)
        {
            node.parent = pooledNodes;
            pooledNodes = node;
        }

        public Node NodeFromPool()
        {
            Node node = pooledNodes;

            if (node != null)
            {
                pooledNodes = node.parent;
                return node;
            }
            else
            {

                Node buffer = new Node(); // (Pair*)cpcalloc(1, CP_BUFFER_BYTES);
                allocatedBuffers.Add(buffer);
                NodeRecycle(buffer);
                return buffer;

                // Pool is exhausted, make more
                //int count = CP_BUFFER_BYTES / sizeof(Node);
                //cpAssertHard(count, "Internal Error: Buffer size is too small.");

                //Node* buffer = (Node*)cpcalloc(1, CP_BUFFER_BYTES);
                //cpArrayPush(tree->allocatedBuffers, buffer);

                //// push all but the first one, return the first instead
                //for (int i = 1; i < count; i++) NodeRecycle(tree, buffer + i);
                //return buffer;
            }
        }


        public static void NodeSetA(Node node, Node value)
        {
            node.a = value;
            value.parent = node;
        }

        public static void NodeSetB(Node node, Node value)
        {
            node.b = value;
            value.parent = node;
        }

        public Node NodeNew(Node a, Node b)
        {
            Node node = NodeFromPool();

            node.obj = null;
            node.bb = cpBB.Merge(a.bb, b.bb);
            node.parent = null;

            NodeSetA(node, a);
            NodeSetB(node, b);

            return node;
        }

        //public static bool NodeIsLeaf(Node node)
        //{
        //    return (node.obj != null);
        //}

        public static Node NodeOther(Node node, Node child)
        {
            return (node.A == child ? node.B : node.A);
        }

        public void NodeReplaceChild(Node parent, Node child, Node value)
        {
            cpEnvironment.cpAssertSoft(!parent.IsLeaf(), "Internal Error: Cannot replace child of a leaf.");
            cpEnvironment.cpAssertSoft(child == parent.a || child == parent.b, "Internal Error: Node is not a child of parent.");

            if (parent.A == child)
            {
                NodeRecycle(parent.A);
                NodeSetA(parent, value);
            }
            else
            {
                NodeRecycle(parent.B);
                NodeSetB(parent, value);
            }

            for (Node node = parent; node != null; node = node.parent)
            {
                node.bb = cpBB.Merge(node.A.bb, node.B.bb);
            }
        }


        //MARK: Subtree Functions

        public static float cpBBProximity(cpBB a, cpBB b)
        {
            return cpEnvironment.cpfabs(a.l + a.r - b.l - b.r) + cpEnvironment.cpfabs(a.b + a.t - b.b - b.t);
        }

        public Node SubtreeInsert(Node subtree, Node leaf)
        {
            if (subtree == null)
            {
                return leaf;
            }
            else if (subtree.IsLeaf())
            {
                return NodeNew(leaf, subtree);
            }
            else
            {
                float cost_a = cpBB.Area(subtree.B.bb) + cpBB.MergedArea(subtree.A.bb, leaf.bb);
                float cost_b = cpBB.Area(subtree.A.bb) + cpBB.MergedArea(subtree.B.bb, leaf.bb);

                if (cost_a == cost_b)
                {
                    cost_a = cpBBProximity(subtree.A.bb, leaf.bb);
                    cost_b = cpBBProximity(subtree.B.bb, leaf.bb);
                }

                if (cost_b < cost_a)
                {
                    NodeSetB(subtree, SubtreeInsert(subtree.B, leaf));
                }
                else
                {
                    NodeSetA(subtree, SubtreeInsert(subtree.A, leaf));
                }

                subtree.bb = subtree.bb.Merge(leaf.bb);
                return subtree;
            }
        }

        public bool Contains(object obj)
        {
            foreach (var item in this)
            {
                if (item.Value == obj)
                    return true;
            }
            return false;
        }

        public bool ContainsHash(int hashid)
        {
            foreach (var item in this)
            {
                if (item.Key == hashid)
                    return true;
            }
            return false;
        }

        public void ReindexQuery(cpSpatialIndexQueryFunc func, object data)
        {

            if (root != null) return;

            // LeafUpdate() may modify tree->root. Don't cache it.
            foreach (var item in leaves)
                item.Value.Update(this);//  LeafUpdate()

            // cpSpatialIndex* staticIndex = staticIndex;
            Node staticRoot = (staticIndex != null ? staticIndex.root : null);

            MarkContext context = new MarkContext(this, staticRoot, func, data);
            root.MarkSubtree(context);
            if (staticIndex != null && staticRoot != null)
                CollideStatic(staticIndex, func, data);

            IncrementStamp();

        }

        public void CollideStatic(cpBBTree staticIndex, cpSpatialIndexQueryFunc func, object data)
        {
            if (staticIndex != null && cpSpatialIndexCount(staticIndex) > 0)
            {

                dynamicToStaticContext context = new dynamicToStaticContext(dynamicIndex.bbfunc, staticIndex, func, data);

                dynamicToStaticIter(dynamicIndex, context);

                //foreach (var item in this)
                //{
                //  staticIndex.Query(item.Value,  )
                //}
                //TODO: FINISH
                // cpSpatialIndexEach(dynamicIndex, (cpSpatialIndexIteratorFunc)dynamicToStaticIter, context);
            }
        }

        public void dynamicToStaticIter(object obj, dynamicToStaticContext context)
        {
            context.staticIndex.Query(obj, context.bbfunc(obj), context.queryFunc, context.data);
            //IndexQuery(, );
        }

        //MARK: Query
        public void SegmentQuery(object obj, cpVect a, cpVect b, float t_exit, cpSpatialIndexSegmentQueryFunc func, object data)
        {
            //Node* root = root;
            if (root != null)
                SubtreeSegmentQuery(root, obj, a, b, t_exit, func, data);
        }

        public void Query(object obj, cpBB bb, cpSpatialIndexQueryFunc func, object data)
        {
            if (root != null)
                SubtreeQuery(root, obj, bb, func, data);
        }

        public static void SubtreeQuery(Node subtree, object obj, cpBB bb, cpSpatialIndexQueryFunc func, object data)
        {
            if (subtree.bb.Intersects(bb))
            {
                if (subtree.IsLeaf())
                {
                    func(obj, subtree.obj, 0, data);
                }
                else
                {
                    SubtreeQuery(subtree.A, obj, bb, func, data);
                    SubtreeQuery(subtree.B, obj, bb, func, data);
                }
            }
        }

        public static float SubtreeSegmentQuery(Node subtree, object obj, cpVect a, cpVect b, float t_exit, cpSpatialIndexSegmentQueryFunc func, object data)
        {
            if (subtree.IsLeaf())
            {
                return func(obj, subtree.obj, data);
            }
            else
            {
                float t_a = subtree.A.bb.SegmentQuery(a, b);
                float t_b = subtree.B.bb.SegmentQuery(a, b);

                if (t_a < t_b)
                {
                    if (t_a < t_exit) t_exit = cpEnvironment.cpfmin(t_exit, SubtreeSegmentQuery(subtree.A, obj, a, b, t_exit, func, data));
                    if (t_b < t_exit) t_exit = cpEnvironment.cpfmin(t_exit, SubtreeSegmentQuery(subtree.B, obj, a, b, t_exit, func, data));
                }
                else
                {
                    if (t_b < t_exit) t_exit = cpEnvironment.cpfmin(t_exit, SubtreeSegmentQuery(subtree.B, obj, a, b, t_exit, func, data));
                    if (t_a < t_exit) t_exit = cpEnvironment.cpfmin(t_exit, SubtreeSegmentQuery(subtree.A, obj, a, b, t_exit, func, data));
                }

                return t_exit;
            }
        }

        public void SubtreeRecycle(Node node)
        {
            if (!node.IsLeaf())
            {
                SubtreeRecycle(node.A);
                SubtreeRecycle(node.B);
                NodeRecycle(node);
            }
        }

        public Node SubtreeRemove(Node subtree, Node leaf)
        {
            if (leaf == subtree)
            {
                return null;
            }
            else
            {
                Node parent = leaf.parent;
                if (parent == subtree)
                {
                    Node other = NodeOther(subtree, leaf);
                    other.parent = subtree.parent;
                    NodeRecycle(subtree);
                    return other;
                }
                else
                {
                    NodeReplaceChild(parent.parent, parent, NodeOther(parent, leaf));
                    return subtree;
                }
            }
        }

        //MARK: Leaf Functions

        public Node LeafNew(object obj, cpBB bb)
        {
            Node node = NodeFromPool();
            node.obj = obj;

            node.bb = GetBB(obj);

            node.parent = null;
            node.STAMP = 0;
            node.PAIRS = null;

            return node;
        }

        public bool LeafUpdate(Node leaf)
        {
            //Node root = tree.root;
            cpBB bb = bbfunc(leaf.obj);

            if (!leaf.bb.ContainsBB(bb))
            {
                leaf.bb = GetBB(leaf.obj);

                root = SubtreeRemove(root, leaf);
                root = SubtreeInsert(root, leaf);

                PairsClear(leaf);
                leaf.STAMP = stamp;

                return true;
            }
            else
            {
                return false;
            }
        }

        public static int VoidQueryFunc(object obj1, object obj2, int id, object data) { return id; }

        public static void LeafAddPairs(Node leaf, cpBBTree tree)
        {
            cpBBTree dynamicIndex = tree.dynamicIndex;
            if (dynamicIndex != null)
            {
                // Node dynamicRoot = ;
                if (dynamicIndex.root != null)
                {
                    //cpBBTree dynamicTree = GetTree(dynamicIndex);
                    MarkContext context = new MarkContext(dynamicIndex, null, null, null);
                    //MarkLeafQuery(dynamicIndex.root, );
                    dynamicIndex.root.MarkLeafQuery(leaf, true, context);
                }
            }
            else
            {
                //Node staticRoot = tree.staticIndex.root; // GetRootIfTree(tree.staticIndex);
                MarkContext context = new MarkContext(tree, tree.staticIndex.root, VoidQueryFunc, null);
                leaf.MarkLeaf(context);
            }
        }


        public void SetDefaultValue(cpCollisionHandler value)
        {
            defaultValue = value;
        }

        internal void Reindex()
        {
            throw new NotImplementedException();
        }

        internal void ReindexObject(int p, cpShape shape)
        {
            throw new NotImplementedException();
        }
    };






}