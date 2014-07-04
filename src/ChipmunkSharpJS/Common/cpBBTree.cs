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
/**
	@defgroup cpSpatialIndex cpSpatialIndex
	
	Spatial indexes are data structures that are used to accelerate collision detection
	and spatial queries. Chipmunk provides a number of spatial index algorithms to pick from
	and they are programmed in a generic way so that you can use them for holding more than
	just Shapes.
	
	It works by using pointers to the objects you add and using a callback to ask your code
	for bounding boxes when it needs them. Several types of queries can be performed an index as well
	as reindexing and full collision information. All communication to the spatial indexes is performed
	through callback functions.
	
	Spatial indexes should be treated as opaque structs.
	This means you shouldn't be reading any of the fields directly.

	All spatial indexes define the following methods:
		
	// The number of objects in the spatial index.
	count = 0;

	// Iterate the objects in the spatial index. @c func will be called once for each object.
	each(func);
	
	// Returns true if the spatial index contains the given object.
	// Most spatial indexes use hashed storage, so you must provide a hash value too.
	contains(obj, hashid);

	// Add an object to a spatial index.
	insert(obj, hashid);

	// Remove an object from a spatial index.
	remove(obj, hashid);
	
	// Perform a full reindex of a spatial index.
	reindex();

	// Reindex a single object in the spatial index.
	reindexObject(obj, hashid);

	// Perform a point query against the spatial index, calling @c func for each potential match.
	// A pointer to the point will be passed as @c obj1 of @c func.
	// func(shape);
	pointQuery(point, func);

	// Perform a segment query against the spatial index, calling @c func for each potential match.
	// func(shape);
	segmentQuery(vect a, vect b, t_exit, func);

	// Perform a rectangle query against the spatial index, calling @c func for each potential match.
	// func(shape);
	query(bb, func);

	// Simultaneously reindex and find all colliding objects.
	// @c func will be called once for each potentially overlapping pair of objects found.
	// If the spatial index was initialized with a static index, it will collide it's objects against that as well.
	reindexQuery(func);
*/
using System;
using System.Collections.Generic;
using System.Threading;
using System.Linq;

namespace ChipmunkSharp
{


    public interface IObjectBox
    {
        cpBB bb { get; set; }
    }

    //MARK: Spatial Index

    /// Bounding box tree velocity callback function.
    /// This function should return an estimate for the object's velocity.
    public delegate cpVect cpBBTreeVelocityFunc(object obj);

    public class Node
    {

        public bool IsLeaf;

        private Node a { get; set; }
        private Node b { get; set; }

        public Node A { get { return a; } }
        public Node B { get { return b; } }

        public uint stamp;
        public uint STAMP { get { return stamp; } set { stamp = value; } }

        public IObjectBox obj;

        public cpBB bb;

        public Node parent;

        public Pair pairs;
        public Pair PAIRS { get { return pairs; } set { pairs = value; } }

        public void SetA(Node value)
        {
            this.a = value;
            value.parent = this;
        }

        public void SetB(Node value)
        {
            this.b = value;
            value.parent = this;
        }

        public virtual void recycle(cpBBTree tree)
        {
            this.parent = tree.pooledNodes;
            tree.pooledNodes = this;
        }
        public Node OtherChild(Leaf child)
        {
            return (A == child ? B : A);
        }
        public void ReplaceChild(Node child, Node value, cpBBTree tree)
        {

            cpEnvironment.assertSoft(child == this.A || child == this.B, "Node is not a child of parent.");

            if (this.A == child)
            {
                this.A.recycle(tree);
                SetA(value);
            }
            else
            {
                this.B.recycle(tree);
                this.SetB(value);
            }

            for (var node = this; node != null; node = node.parent)
            {
                //node.bb = bbMerge(node.A.bb, node.B.bb);
                var a = node.A;
                var b = node.B;
                node.bb.l = Math.Min(a.bb.l, b.bb.l);
                node.bb.b = Math.Min(a.bb.b, b.bb.b);
                node.bb.r = Math.Max(a.bb.r, b.bb.r);
                node.bb.t = Math.Max(a.bb.t, b.bb.t);
            }
        }

        public virtual void MarkLeafQuery(Leaf leaf, bool left, cpBBTree tree, Func<object, object, object> func)
        {
            if (bbTreeIntersectsNode(leaf, this))
            {
                this.A.MarkLeafQuery(leaf, left, tree, func);
                this.B.MarkLeafQuery(leaf, left, tree, func);
            }
        }

        public virtual void MarkSubtree(cpBBTree tree, Node staticRoot, Func<object, object, object> func)
        {
            this.A.MarkSubtree(tree, staticRoot, func);
            this.B.MarkSubtree(tree, staticRoot, func);
        }

        public Node()
        {
        }

        public Node(Node a, Node b, cpBBTree tree)
        {

            this.obj = null;
            bb = new cpBB(
                Math.Min(a.bb.l, b.bb.l),
                Math.Min(a.bb.b, b.bb.b),
                Math.Max(a.bb.r, b.bb.r),
                Math.Max(a.bb.t, b.bb.t)
                );

            parent = null;

            this.a = a;
            this.b = b;
        }

        public static bool bbTreeIntersectsNode(Node a, Node b)
        {
            return (a.bb.l <= b.bb.r && b.bb.l <= a.bb.r && a.bb.b <= b.bb.t && b.bb.b <= a.bb.t);
        }

        //public Node SubtreeRemove(Node subtree, Leaf leaf, cpBBTree tree)
        //{
        //    if (leaf == subtree)
        //    {
        //        return null;
        //    }
        //    else
        //    {
        //        var parent = leaf.parent;
        //        if (parent == subtree)
        //        {
        //            var other = subtree.OtherChild(leaf);
        //            other.parent = subtree.parent;
        //            subtree.recycle(tree);
        //            return other;
        //        }
        //        else
        //        {

        //            if (parent != null)
        //            {
        //                parent.parent.ReplaceChild(parent, parent.OtherChild(leaf), tree);
        //                return subtree;
        //            }
        //        }
        //    }

        //    return null;
        //}

        //public bool IsLeaf()
        //{
        //    return (obj != null);
        //}


    }

    public class Leaf : Node
    {

        /// <summary>
        /// 
        /// </summary>
        /// <param name="cpBBTree"></param>
        /// <param name="value"></param>

        public Leaf(cpBBTree tree, IObjectBox obj)
        {

            IsLeaf = true;

            this.obj = obj; //THIS IS THE GENERIC REAL VALUE

            bb = tree.getBB(obj);

            this.parent = null;

            this.stamp = 1;
            this.pairs = null;

            cpEnvironment.numLeaves++;
        }

        public void clearPairs(cpBBTree tree)
        {
            // tree.PairsClear(this);

            Pair pair = this.pairs;
            Pair next;
            this.pairs = null;

            while (pair != null)
            {
                if (pair.leafA == this)
                {
                    next = pair.nextA;
                    cpEnvironment.unlinkThread(pair.prevB, pair.leafB, pair.nextB);
                }
                else
                {
                    next = pair.nextB;
                    cpEnvironment.unlinkThread(pair.prevA, pair.leafA, pair.nextA);
                }
                pair.recycle(tree);
                pair = next;
            }

        }

        public override void recycle(cpBBTree tree)
        {
        }

        public override void MarkLeafQuery(Leaf leaf, bool left, cpBBTree tree, Func<object, object, object> func)
        {
            if (bbTreeIntersectsNode(leaf, this))
            {
                if (left)
                {
                    tree.PairInsert(leaf, this);
                }
                else
                {
                    if (this.stamp < leaf.stamp)
                        tree.PairInsert(this, leaf);
                    if (func != null)
                        func(leaf.obj, this.obj);
                }
            }
        }

        public override void MarkSubtree(cpBBTree tree, Node staticRoot, Func<object, object, object> func)
        {
            if (this.stamp == tree.getStamp())
            {
                if (staticRoot != null) staticRoot.MarkLeafQuery(this, false, tree, func);

                for (Node node = this; node.parent != null; node = node.parent)
                {
                    if (node == node.parent.A)
                    {
                        node.parent.B.MarkLeafQuery(this, true, tree, func);
                    }
                    else
                    {
                        node.parent.A.MarkLeafQuery(this, false, tree, func);
                    }
                }
            }
            else
            {
                var pair = this.pairs;
                while (pair != null)
                {
                    if (this == pair.leafB)
                    {
                        if (func != null) func(pair.leafA.obj, this.obj);
                        pair = pair.nextB;
                    }
                    else
                    {
                        pair = pair.nextA;
                    }
                }
            }
        }

        public bool containsObj(object objData)
        {
            var obj = objData as IObjectBox;
            if (obj == null)
                return false;

            return (this.bb.l <= obj.bb.l && this.bb.r >= obj.bb.r && this.bb.b <= obj.bb.b && this.bb.t >= obj.bb.t);
        }

        //MARK: Marking Functions

        public bool Update(cpBBTree tree)
        {

            var root = tree.root;

            if (obj != null)
            {

                bb = tree.getBB(obj);
                root = cpEnvironment.SubtreeRemove(root, this, tree);
                tree.root = tree.SubtreeInsert(root, this);//tree.root = SubtreeInsert(root, this, tree);
                this.clearPairs(tree);
                this.stamp = tree.getStamp();

                return true;
            }

            return false;
        }

        public void AddPairs(cpBBTree tree)
        {
            var dynamicIndex = tree.dynamicIndex;
            if (dynamicIndex != null)
            {
                var dynamicRoot = dynamicIndex.root;
                if (dynamicRoot != null)
                {
                    dynamicRoot.MarkLeafQuery(this, true, dynamicIndex, null);
                }
            }
            else
            {
                Node staticRoot = tree.root;
                MarkSubtree(tree, staticRoot, null);
            }

        }

    }

    public class Pair
    {
        public Node leafA;
        public Pair nextA;
        public Pair prevA;

        public Node leafB;
        public Pair nextB;
        public Pair prevB;

        // Objects created with constructors are faster than object literals. :(
        public Pair(Node leafA, Pair nextA, Node leafB, Pair nextB)
        {
            this.leafA = leafA;
            this.nextA = nextA;
            this.leafB = leafB;
            this.nextB = nextB;
            this.prevB = this.prevA = null;
        }

        public void recycle(cpBBTree tree)
        {
            this.prevA = tree.pooledPairs;
            tree.pooledPairs = this;
        }
    }


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

    public class cpBBTree
    {

        // Collide the objects in an index against the objects in a staticIndex using the query callback function.
        public void CollideStatic(cpBBTree staticIndex, Func<object, object, object> func)
        {
            if (staticIndex.Count > 0)
            {
                //var query = staticIndex.Query;

                each((obj) =>
                {
                    staticIndex.query(new cpBB(obj.bb.l, obj.bb.b, obj.bb.r, obj.bb.t), func);
                });
            }
        }

        public Node root { get; set; }

        public cpBBTree staticIndex { get; set; }
        public cpBBTree dynamicIndex { get; set; }

        public Dictionary<int, Leaf> leaves { get; set; }

        public Node pooledNodes { get; set; }
        public Pair pooledPairs { get; set; }

        public uint stamp { get; set; }

        //MARK: Misc Functions

        public Func<object, cpVect> velocityFunc { get; set; }

        public cpBBTree(cpBBTree staticIndex)
        {

            this.staticIndex = staticIndex;

            if (staticIndex != null)
            {
                if (staticIndex.dynamicIndex != null)
                {
                    throw new NotImplementedException("This static index is already associated with a dynamic index.");
                }
                staticIndex.dynamicIndex = this;
            }


            this.velocityFunc = null;

            // This is a hash from object ID -> object for the objects stored in the BBTree.
            leaves = new Dictionary<int, Leaf>();

            // elements = new Dictionary<int, object>();
            root = null;

            // A linked list containing an object pool of tree nodes and pairs.
            this.pooledNodes = null;
            this.pooledPairs = null;

            stamp = 0;
        }

        /// Get the number of objects in the spatial index.
        public int Count
        {
            get
            {
                return leaves.Count;
            }
        }

        public Pair MakePair(Node leafA, Pair nextA, Node leafB, Pair nextB)
        {
            var pair = this.pooledPairs;
            if (pair != null)
            {
                this.pooledPairs = pair.prevA;

                pair.prevA = null;
                pair.leafA = leafA;
                pair.nextA = nextA;

                pair.prevB = null;
                pair.leafB = leafB;
                pair.nextB = nextB;

                return pair;
            }
            else
            {
                cpEnvironment.numPairs++;
                return new Pair(leafA, nextA, leafB, nextB);
            }
        }

        public void SubtreeRecycle(Node node)
        {
            if (node.IsLeaf)
            {
                SubtreeRecycle(node.A);
                SubtreeRecycle(node.B);
                node.recycle(this);
            }
        }

        public Leaf insert(int hashid, IObjectBox value)
        {
            var leaf = new Leaf(this, value);
            this.leaves.Add(hashid, leaf);

            this.root = SubtreeInsert(root, leaf);

            leaf.stamp = getStamp();
            leaf.AddPairs(this);
            incrementStamp();
            return leaf;
        }

        public void remove(int key)
        {
            Leaf leaf;
            if (TryGetValue(key, out leaf))
            {
                //remove elements adds more functionality than simple array
                leaves.Remove(key);
                if (root != null)
                    this.root = cpEnvironment.SubtreeRemove(this.root, leaf, this);
                leaf.clearPairs(this);
                leaf.recycle(this);
            }
        }

        public bool containsValue(object obj)
        {
            foreach (var item in leaves)
            {
                if (item.Value.obj == obj)
                    return true;
            }
            return false;
        }

        public bool Contains(Leaf obj)
        {
            foreach (var item in leaves)
            {
                if (item.Value == obj)
                    return true;
            }
            return false;
        }

        public bool TryGetValue(int key, out Leaf value)
        {
            return leaves.TryGetValue(key, out value);
        }

        public bool ContainsHash(int hashid)
        {
            foreach (var item in leaves)
                if (item.Key == hashid)
                    return true;
            return false;
        }

        public Node makeNode(Node a, Node b)
        {
            var node = this.pooledNodes;
            if (node != null)
            {
                this.pooledNodes = node.parent;
                node = new Node(a, b, this);
                return node;
            }
            else
            {
                cpEnvironment.numNodes++;
                return new Node(a, b, this);
            }
        }

        public cpBB getBB(object obj)
        {

            //TODO: GETBB

            cpBB bb = (obj as IObjectBox).bb;

            if (bb == null)
                throw new NotImplementedException();

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
                //TODO: Add interface
                return bb;
            }

        }

        public uint getStamp()
        {
            var dynamic = this.dynamicIndex;
            return (dynamic != null && dynamic.stamp != 0 ? dynamic.stamp : this.stamp);
        }

        public void incrementStamp()
        {
            //  cpBBTree dynamicTree = tree.dynamicIndex;
            if (dynamicIndex != null && this.dynamicIndex.stamp != 0)
                dynamicIndex.stamp++;
            else
                stamp++;
        }


        public void reindexQuery(Func<object, object, object> func)
        {

            if (root == null) return;

            // LeafUpdate() may modify tree->root. Don't cache it.
            foreach (var item in leaves)
                item.Value.Update(this);//  LeafUpdate()


            Node staticRoot = (staticIndex != null ? staticIndex.root : null);

            root.MarkSubtree(this, staticRoot, func);

            if (staticIndex != null && staticRoot == null)
                CollideStatic(staticIndex, func); //, data);

            incrementStamp();

        }

        public void reindex()
        {
            reindexQuery(voidQueryFunc); //cpBBTreeReindexQuery(tree, VoidQueryFunc, null);
        }

        public object voidQueryFunc(object obj1, object obj2) { return null; }

        public void reindexObject(int key, object obj)
        {
            Leaf leaf;
            if (leaves.TryGetValue(key, out leaf))
            {

                if (leaf.Update(this))
                    leaf.AddPairs(this);

                incrementStamp();
            }
        }

        public void PointQuery(cpVect point, Func<object, object, object> func)
        {
            query(new cpBB(point.x, point.y, point.x, point.y), func);
        }

        public void SegmentQuery(cpVect a, cpVect b, float t_exit, Func<object, float> func)
        {
            //Node* root = root;
            if (root != null)
                SubtreeSegmentQuery(this.root, a, b, t_exit, func);
        }

        public float SubtreeSegmentQuery(Node subtree, cpVect a, cpVect b, float t_exit, Func<object, float> func)
        {
            if (subtree.IsLeaf)
            {
                return func(subtree.obj);
            }
            else
            {
                float t_a = NodeSegmentQuery(subtree.A, a, b);
                float t_b = NodeSegmentQuery(subtree.B, a, b);

                if (t_a < t_b)
                {
                    if (t_a < t_exit) t_exit = Math.Min(t_exit, SubtreeSegmentQuery(subtree.A, a, b, t_exit, func));
                    if (t_b < t_exit) t_exit = Math.Min(t_exit, SubtreeSegmentQuery(subtree.B, a, b, t_exit, func));
                }
                else
                {
                    if (t_b < t_exit) t_exit = Math.Min(t_exit, SubtreeSegmentQuery(subtree.B, a, b, t_exit, func));
                    if (t_a < t_exit) t_exit = Math.Min(t_exit, SubtreeSegmentQuery(subtree.A, a, b, t_exit, func));
                }

                return t_exit;
            }
        }

        public float NodeSegmentQuery(Node node, cpVect a, cpVect b)
        {
            float idx = 1 / (b.x - a.x);
            float tx1 = (node.bb.l == a.x ? -cpEnvironment.Infinity : (node.bb.l - a.x) * idx);
            float tx2 = (node.bb.r == a.x ? cpEnvironment.Infinity : (node.bb.r - a.x) * idx);
            float txmin = Math.Min(tx1, tx2);
            float txmax = Math.Max(tx1, tx2);

            float idy = 1 / (b.y - a.y);
            float ty1 = (node.bb.b == a.y ? -cpEnvironment.Infinity : (node.bb.b - a.y) * idy);
            float ty2 = (node.bb.t == a.y ? cpEnvironment.Infinity : (node.bb.t - a.y) * idy);
            float tymin = Math.Min(ty1, ty2);
            float tymax = Math.Max(ty1, ty2);

            if (tymin <= txmax && txmin <= tymax)
            {
                var min_ = Math.Max(txmin, tymin);
                var max_ = Math.Min(txmax, tymax);

                if (0.0 <= max_ && min_ <= 1.0f) return Math.Max(min_, 0.0f);
            }

            return cpEnvironment.Infinity;
        }

        public void query(cpBB bb, Func<object, object, object> func)
        {
            if (root != null)
                subtreeQuery(root, bb, func);
        }
        public void subtreeQuery(Node subtree, cpBB bb, Func<object, object, object> func)
        {
            //if(bbIntersectsBB(subtree.bb, bb)){
            if (subtree.bb.Intersects(bb))
            {
                if (subtree.IsLeaf)
                {
                    func(subtree.obj, null);
                }
                else
                {
                    subtreeQuery(subtree.A, bb, func);
                    subtreeQuery(subtree.B, bb, func);
                }
            }
        }

        public void each(Action<IObjectBox> func)
        {
            foreach (var item in leaves)
                func(item.Value.obj);
        }

        public void Optimize()
        {
            //TODO: REVISE OPTIMIZATION
            var nodes = new Dictionary<int, Leaf>(this.Count);
            var i = 0;
            foreach (var hashid in leaves)
                nodes.Add(i++, hashid.Value);
            SubtreeRecycle(root);

            root = cpEnvironment.partitionNodes(this, nodes, 0, nodes.Count);
        }

        public void PairInsert(Leaf a, Leaf b)
        {
            Pair nextA = a.pairs, nextB = b.pairs;
            var pair = MakePair(a, nextA, b, nextB);
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

        public void Log()
        {
            if (this.root != null)
                cpEnvironment.nodeRender(this.root, 0);
        }



        //================================= LAST CHECK ===================================







        //public cpBBTree GetMasterTree()
        //{
        //    return (dynamicIndex != null ? dynamicIndex : this);
        //}


        //public static cpBBTree cpSpatialIndexInit(cpBBTree index, cpSpatialIndexBBFunc bbfunc, cpBBTree staticIndex)
        //{
        //    //index.klass = klass;
        //    index.bbfunc = bbfunc;
        //    index.staticIndex = staticIndex;

        //    if (staticIndex != null)
        //    {
        //        cpEnvironment.AssertHard(staticIndex.dynamicIndex != null, "This static index is already associated with a dynamic index.");
        //        staticIndex.dynamicIndex = index;
        //    }

        //    return index;
        //}




        //public void PairRecycle(Pair pair)
        //{
        //    // Share the pool of the master tree.
        //    // TODO would be lovely to move the pairs stuff into an external data structure.
        //    //tree = GetMasterTree(tree);

        //    pair.nextA = pooledPairs;
        //    pooledPairs = pair;
        //}





        //public void PairsClear(Node leaf)//, cpBBTree tree)
        //{
        //    Pair pair = leaf.pairs;
        //    leaf.pairs = null;

        //    while (pair != null)
        //    {
        //        if (pair.a.leaf == leaf)
        //        {
        //            Pair next = pair.a.next;
        //            ThreadUnlink(pair.b);
        //            PairRecycle(pair);
        //            pair = next;
        //        }
        //        else
        //        {
        //            Pair next = pair.b.next;
        //            ThreadUnlink(pair.a);
        //            PairRecycle(pair);
        //            pair = next;
        //        }
        //    }
        //}

        //public void PairInsert(Node a, Node b)
        //{

        //    Pair nextA = a.pairs, nextB = b.pairs;
        //    var pair = MakePair(a, nextA, b, nextB);
        //    a.pairs = b.pairs = pair;

        //    if (nextA != null)
        //    {
        //        if (nextA.a.leaf == a) nextA.a.prev = pair; else nextA.b.prev = pair;
        //    }

        //    if (nextB != null)
        //    {
        //        if (nextB.a.leaf == b) nextB.a.prev = pair; else nextB.b.prev = pair;
        //    }

        //    //Pair nextA = a.pairs, nextB = b.pairs;
        //    //Pair pair = PairFromPool();
        //    //Pair temp = new Pair(new Thread(null, a, nextA), new Thread(null, b, nextB), 0);

        //    //a.pairs = b.pairs = pair;
        //    //pair = temp;

        //    //if (nextA != null)
        //    //{
        //    //    if (nextA.a.leaf == a) nextA.a.prev = pair; else nextA.b.prev = pair;
        //    //}

        //    //if (nextB != null)
        //    //{
        //    //    if (nextB.a.leaf == b) nextB.a.prev = pair; else nextB.b.prev = pair;
        //    //}
        //}




        ////MARK: Node Functions

        //public void NodeRecycle(Node node)
        //{
        //    node.parent = pooledNodes;
        //    pooledNodes = node;
        //}

        //public static Node NodeOther(Node node, Node child)
        //{
        //    return (node.A == child ? node.B : node.A);
        //}

        //public void NodeReplaceChild(Node parent, Node child, Node value)
        //{
        //    cpEnvironment.AssertSoft(!parent.IsLeaf, "Internal Error: Cannot replace child of a leaf.");
        //    cpEnvironment.AssertSoft(child == parent.A || child == parent.B, "Internal Error: Node is not a child of parent.");

        //    if (parent.A == child)
        //    {
        //        NodeRecycle(parent.A);
        //        NodeSetA(parent, value);
        //    }
        //    else
        //    {
        //        NodeRecycle(parent.B);
        //        NodeSetB(parent, value);
        //    }

        //    for (Node node = parent; node != null; node = node.parent)
        //    {
        //        node.bb = cpBB.Merge(node.A.bb, node.B.bb);
        //    }
        //}


        ////MARK: Subtree Functions

        //public static float cpBBProximity(cpBB a, cpBB b)
        //{
        //    return cpEnvironment.cpfabs(a.l + a.r - b.l - b.r) + cpEnvironment.cpfabs(a.b + a.t - b.b - b.t);
        //}

        //public Node SubtreeInsert(Node subtree, Node leaf)
        //{
        //    if (subtree == null)
        //    {
        //        return leaf;
        //    }
        //    else if (subtree.IsLeaf())
        //    {
        //        return MakeNode(leaf, subtree);
        //    }
        //    else
        //    {
        //        float cost_a = subtree.B.bb.Area() + subtree.A.bb.MergedArea(leaf.bb);
        //        float cost_b = subtree.A.bb.Area() + subtree.B.bb.MergedArea(leaf.bb);// cpBB.MergedArea(subtree.B.bb, leaf.bb);

        //        if (cost_a == cost_b)
        //        {
        //            cost_a = cpBBProximity(subtree.A.bb, leaf.bb);
        //            cost_b = cpBBProximity(subtree.B.bb, leaf.bb);
        //        }

        //        if (cost_b < cost_a)
        //        {
        //            NodeSetB(subtree, SubtreeInsert(subtree.B, leaf));
        //        }
        //        else
        //        {
        //            NodeSetA(subtree, SubtreeInsert(subtree.A, leaf));
        //        }

        //        subtree.bb = subtree.bb.Merge(leaf.bb);
        //        return subtree;
        //    }
        //}



        ////public void CollideStatic(cpBBTree staticIndex, cpSpatialIndexQueryFunc func)
        ////{
        ////    if (staticIndex != null && staticIndex.Count() > 0)
        ////    {
        ////        //  var query = 

        ////        dynamicToStaticContext context = new dynamicToStaticContext(staticIndex.bbfunc, staticIndex, func, null);
        ////        foreach (var item in leaves)
        ////        {
        ////            staticIndex.Query(item.Value, item.Value.bb, func, context);
        ////        }
        ////    }
        ////}

        //public void dynamicToStaticIter(object obj, dynamicToStaticContext context)
        //{
        //    context.staticIndex.Query(obj, context.bbfunc(obj), context.queryFunc, context.data);
        //    //IndexQuery(, );
        //}

        ////MARK: Query

        //public static void SubtreeQuery(Node subtree, object obj, cpBB bb, cpSpatialIndexQueryFunc func, object data)
        //{
        //    if (subtree.bb.Intersects(bb))
        //    {
        //        if (subtree.IsLeaf)
        //        {
        //            func(obj, subtree.obj, 0, data);
        //        }
        //        else
        //        {
        //            SubtreeQuery(subtree.A, obj, bb, func, data);
        //            SubtreeQuery(subtree.B, obj, bb, func, data);
        //        }
        //    }
        //}

        //public static float SubtreeSegmentQuery(Node subtree, object obj, cpVect a, cpVect b, float t_exit, cpSpatialIndexSegmentQueryFunc func, object data)
        //{
        //    if (subtree.IsLeaf)
        //    {
        //        return func(obj, subtree.obj, data);
        //    }
        //    else
        //    {
        //        float t_a = subtree.A.bb.SegmentQuery(a, b);
        //        float t_b = subtree.B.bb.SegmentQuery(a, b);

        //        if (t_a < t_b)
        //        {
        //            if (t_a < t_exit) t_exit = cpEnvironment.cpfmin(t_exit, SubtreeSegmentQuery(subtree.A, obj, a, b, t_exit, func, data));
        //            if (t_b < t_exit) t_exit = cpEnvironment.cpfmin(t_exit, SubtreeSegmentQuery(subtree.B, obj, a, b, t_exit, func, data));
        //        }
        //        else
        //        {
        //            if (t_b < t_exit) t_exit = cpEnvironment.cpfmin(t_exit, SubtreeSegmentQuery(subtree.B, obj, a, b, t_exit, func, data));
        //            if (t_a < t_exit) t_exit = cpEnvironment.cpfmin(t_exit, SubtreeSegmentQuery(subtree.A, obj, a, b, t_exit, func, data));
        //        }

        //        return t_exit;
        //    }
        //}


        //public Node SubtreeRemove(Node subtree, Node leaf)
        //{
        //    if (leaf == subtree)
        //    {
        //        return null;
        //    }
        //    else
        //    {
        //        Node parent = leaf.parent;
        //        if (parent == subtree)
        //        {
        //            Node other = NodeOther(subtree, leaf);
        //            other.parent = subtree.parent;
        //            NodeRecycle(subtree);
        //            return other;
        //        }
        //        else
        //        {
        //            NodeReplaceChild(parent.parent, parent, NodeOther(parent, leaf));
        //            return subtree;
        //        }
        //    }
        //}

        //public void AddShapes(cpShape shape)
        //{
        //    Insert(shape.hashid, shape);
        //}

        //public bool LeafUpdate(Node leaf)
        //{
        //    cpBB bb = bbfunc(leaf.obj);

        //    if (!leaf.bb.ContainsBB(bb))
        //    {
        //        leaf.bb = GetBB(leaf.obj);

        //        root = SubtreeRemove(root, leaf);
        //        root = SubtreeInsert(root, leaf);

        //        PairsClear(leaf);
        //        leaf.STAMP = stamp;
        //        return true;
        //    }
        //    else
        //    {
        //        return false;
        //    }
        //}

        ////public static int VoidQueryFunc(object obj1, object obj2, int id, object data) { return id; }
        //public Action<object, object> voidQueryFunc = (o1, o2) => { };


        //public static void LeafAddPairs(Node leaf, cpBBTree tree)
        //{
        //    cpBBTree dynamicIndex = tree.dynamicIndex;
        //    if (dynamicIndex != null)
        //    {
        //        var dynamicRoot = dynamicIndex.root;
        //        if (dynamicRoot != null)
        //        {
        //            MarkContext context = new MarkContext(dynamicIndex, null, null, null);
        //            dynamicRoot.MarkLeafQuery(leaf, true, context);
        //        }
        //    }
        //    else
        //    {
        //        //Node staticRoot = tree.staticIndex.root; // GetRootIfTree(tree.staticIndex);
        //        MarkContext context = new MarkContext(tree, tree.staticIndex.root, VoidQueryFunc, null);
        //        leaf.MarkLeaf(context);
        //    }
        //}
        public Node SubtreeInsert(Node subtree, Leaf leaf)
        {
            if (subtree == null)
            {
                return leaf;
            }
            else if (subtree.IsLeaf)
            {
                return makeNode((Node)leaf, subtree);
            }
            else
            {
                var cost_a = subtree.B.bb.Area() + subtree.A.bb.MergedArea(leaf.bb);
                var cost_b = subtree.A.bb.Area() + subtree.B.bb.MergedArea(leaf.bb);

                if (cost_a == cost_b)
                {
                    cost_a = cpEnvironment.bbProximity(subtree.A, leaf);
                    cost_b = cpEnvironment.bbProximity(subtree.B, leaf);
                }

                if (cost_b < cost_a)
                {
                    subtree.SetB(SubtreeInsert(subtree.B, leaf));
                }
                else
                {
                    subtree.SetA(SubtreeInsert(subtree.A, leaf));
                }

                subtree.bb.l = Math.Min(subtree.bb.l, leaf.bb.l);
                subtree.bb.b = Math.Min(subtree.bb.b, leaf.bb.b);
                subtree.bb.r = Math.Max(subtree.bb.r, leaf.bb.r);
                subtree.bb.t = Math.Max(subtree.bb.t, leaf.bb.t);

                return subtree;
            }


        }

        public object GetValue(int arbHash)
        {
            Leaf dev = Get(arbHash);
            if (dev != null)
                return dev.obj;
            return null;
        }

        public Leaf Get(int arbHash)
        {
            Leaf dev;
            if (TryGetValue(arbHash, out dev))
                return dev;
            return null;
        }
    }






}