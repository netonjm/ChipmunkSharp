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
        float bb_l { get; set; }
        float bb_b { get; set; }
        float bb_r { get; set; }
        float bb_t { get; set; }
    }

    //MARK: Spatial Index

    /// Bounding box tree velocity callback function.
    /// This function should return an estimate for the object's velocity.
    //public delegate cpVect cpBBTreeVelocityFunc(object obj);

    public class Node : IObjectBox
    {

        #region MyRegion

        #endregion

        public bool isLeaf;

        private Node a { get; set; }
        private Node b { get; set; }

        public Node A { get { return a; } }
        public Node B { get { return b; } }

        public int stamp;

        public int STAMP { get { return stamp; } set { stamp = value; } }

        public IObjectBox obj;

        //public cpBB bb;
        public float bb_l { get; set; }
        public float bb_b { get; set; }
        public float bb_r { get; set; }
        public float bb_t { get; set; }


        public Node parent;

        public Pair pairs;
        public Pair PAIRS { get { return pairs; } set { pairs = value; } }
        public Node(Node a, Node b, cpBBTree tree)
        {
            this.obj = null;

            bb_l = Math.Min(a.bb_l, b.bb_l);
            bb_b = Math.Min(a.bb_b, b.bb_b);
            bb_r = Math.Max(a.bb_r, b.bb_r);
            bb_t = Math.Max(a.bb_t, b.bb_t);

            parent = null;

            this.setA(a);
            this.setB(b);
        }

        public Node()
        {
            // TODO: Complete member initialization
        }

        public Node otherChild(Node child)
        {
            return (this.A == child ? this.B : this.A);
        }

        public void ReplaceChild(Node child, Node value, cpBBTree tree)
        {

            cp.assertSoft(child == this.A || child == this.B, "Node is not a child of parent.");

            if (this.A == child)
            {
                this.A.recycle(tree);
                this.setA(value);
            }
            else
            {
                this.B.recycle(tree);
                this.setB(value);
            }

            for (var node = this; node != null; node = node.parent)
            {
                //node.bb = bbMerge(node.A.bb, node.B.bb);
                var a = node.A;
                var b = node.B;

                node.bb_l = Math.Min(a.bb_l, b.bb_l);
                node.bb_b = Math.Min(a.bb_b, b.bb_b);
                node.bb_r = Math.Max(a.bb_r, b.bb_r);
                node.bb_t = Math.Max(a.bb_t, b.bb_t);


            }
        }

        public virtual void markLeafQuery(Leaf leaf, bool left, cpBBTree tree, Action<object, object> func)
        {

            if (cp.bbTreeIntersectsNode(leaf, this))
            {
                this.A.markLeafQuery(leaf, left, tree, func);
                this.B.markLeafQuery(leaf, left, tree, func);
            }
        }

        public virtual void markSubtree(cpBBTree tree, Node staticRoot, Action<object, object> func)
        {
            this.A.markSubtree(tree, staticRoot, func);
            this.B.markSubtree(tree, staticRoot, func);
        }

        public float bbArea()
        {
            return (this.bb_r - this.bb_l) * (this.bb_t - this.bb_b);
        }

        public void setA(Node value)
        {
            this.a = value;
            value.parent = this;
        }

        public void setB(Node value)
        {
            this.b = value;
            value.parent = this;
        }

        public virtual void recycle(cpBBTree tree)
        {
            this.parent = tree.pooledNodes;
            tree.pooledNodes = this;
        }

        public bool intersectsBB(cpBB bb)
        {
            return (this.bb_l <= bb.r && bb.l <= this.bb_r && this.bb_b <= bb.t && bb.b <= this.bb_t);
        }

        //public float bb_l
        //{
        //    get
        //    {
        //        if (obj != null)
        //            return obj.bb_l;

        //        return 0;
        //    }
        //    set
        //    {
        //        if (obj != null)
        //            obj.bb_l = value;
        //    }
        //}

        //public float bb_b
        //{
        //    get
        //    {
        //        if (obj != null)
        //            return obj.bb_b;

        //        return 0;
        //    }
        //    set
        //    {
        //        if (obj != null)
        //            obj.bb_b = value;
        //    }
        //}

        //public float bb_r
        //{
        //    get
        //    {
        //        if (obj != null)
        //            return obj.bb_r;

        //        return 0;
        //    }
        //    set
        //    {
        //        if (obj != null)
        //            obj.bb_r = value;
        //    }
        //}

        //public float bb_t
        //{
        //    get
        //    {
        //        if (obj != null)
        //            return obj.bb_t;

        //        return 0;
        //    }
        //    set
        //    {
        //        if (obj != null)
        //            obj.bb_t = value;
        //    }
        //}
    }

    public class Leaf : Node, IObjectBox
    {

        /// <summary>
        /// 
        /// </summary>
        /// <param name="cpBBTree"></param>
        /// <param name="value"></param>

        public Leaf(cpBBTree tree, IObjectBox obj)
            : base()
        {

            isLeaf = true;

            this.obj = obj; //THIS IS THE GENERIC REAL VALUE

            //bb = tree.getBB(obj);
            tree.getBB(obj, this);

            this.parent = null;

            this.stamp = 1;
            this.pairs = null;

            cp.numLeaves++;
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
                    cp.unlinkThread(pair.prevB, pair.leafB, pair.nextB);
                }
                else
                {
                    next = pair.nextB;
                    cp.unlinkThread(pair.prevA, pair.leafA, pair.nextA);
                }
                pair.recycle(tree);
                pair = next;
            }

        }

        public override void recycle(cpBBTree tree)
        {
        }



        public override void markLeafQuery(Leaf leaf, bool left, cpBBTree tree, Action<object, object> func)
        {
            if (cp.bbTreeIntersectsNode(leaf, this))
            {
                if (left)
                {
                    cp.pairInsert(leaf, this, tree);
                }
                else
                {
                    if (this.stamp < leaf.stamp)
                        cp.pairInsert(this, leaf, tree);
                    if (func != null)
                        func(leaf.obj, this.obj);
                }
            }
        }

        public override void markSubtree(cpBBTree tree, Node staticRoot, Action<object, object> func)
        {
            if (this.stamp == tree.getStamp())
            {
                if (staticRoot != null) staticRoot.markLeafQuery(this, false, tree, func);

                for (Node node = this; node.parent != null; node = node.parent)
                {
                    if (node == node.parent.A)
                    {
                        node.parent.B.markLeafQuery(this, true, tree, func);
                    }
                    else
                    {
                        node.parent.A.markLeafQuery(this, false, tree, func);
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
                        if (func != null)
                            func(pair.leafA.obj, this.obj);

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

            return (this.bb_l <= obj.bb_l && this.bb_r >= obj.bb_r && this.bb_b <= obj.bb_b && this.bb_t >= obj.bb_t);
        }

        //MARK: Marking Functions

        public bool update(cpBBTree tree)
        {

            var root = tree.root;

            var obj = this.obj;

            if (!this.containsObj(obj))
            {

                tree.getBB(this.obj, this);

                root = cp.SubtreeRemove(root, this, tree);
                tree.root = cp.subtreeInsert(root, this, tree);//tree.root = SubtreeInsert(root, this, tree);
                this.clearPairs(tree);
                this.stamp = tree.getStamp();

                return true;
            }
            return false;
        }

        public void addPairs(cpBBTree tree)
        {
            var dynamicIndex = tree.dynamicIndex;
            if (dynamicIndex != null)
            {
                var dynamicRoot = dynamicIndex.root;
                if (dynamicRoot != null)
                {
                    dynamicRoot.markLeafQuery(this, true, dynamicIndex, null);
                }
            }
            else
            {
                var staticRoot = tree.staticIndex.root;
                this.markSubtree(tree, staticRoot, null);
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
            // this.prevB = this.prevA = null;
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
        public void CollideStatic(cpBBTree staticIndex, Action<object, object> func)
        {
            if (staticIndex.Count > 0)
            {

                each((obj) =>
                {
                    staticIndex.query(new cpBB(obj.bb_l, obj.bb_b, obj.bb_r, obj.bb_t), func);
                });
            }
        }

        public Node root { get; set; }

        public cpBBTree staticIndex { get; set; }
        public cpBBTree dynamicIndex { get; set; }

        public Dictionary<string, Leaf> leaves { get; set; }

        public Node pooledNodes { get; set; }
        public Pair pooledPairs { get; set; }

        public int stamp { get; set; }

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
            leaves = new Dictionary<string, Leaf>();

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
                cp.numPairs++;
                return new Pair(leafA, nextA, leafB, nextB);
            }
        }

        public void SubtreeRecycle(Node node)
        {
            if (node.isLeaf)
            {
                SubtreeRecycle(node.A);
                SubtreeRecycle(node.B);
                node.recycle(this);
            }
        }

        public Leaf insert(string hashid, IObjectBox obj)
        {
            var leaf = new Leaf(this, obj);
            this.leaves.Add(hashid, leaf);

            this.root = cp.subtreeInsert(root, leaf, this);

            leaf.stamp = getStamp();
            leaf.addPairs(this);
            incrementStamp();
            return leaf;
        }

        public void remove(string key)
        {
            Leaf leaf;
            if (TryGetValue(key, out leaf))
            {
                //remove elements adds more functionality than simple array
                leaves.Remove(key);

                //if (root != null)
                   
				this.root = cp.SubtreeRemove(this.root, leaf, this);
                
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

        public bool TryGetValue(string key, out Leaf value)
        {
            return leaves.TryGetValue(key, out value);
        }

        public bool ContainsHash(string hashid)
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
                cp.numNodes++;
                return new Node(a, b, this);
            }
        }

        public void getBB(object objElement, IObjectBox dest)
        {

            //TODO: GETBB

            IObjectBox obj = objElement as IObjectBox;
            var velocityFunc = this.velocityFunc;
            if (velocityFunc != null)
            {
                float coef = 0.1f;
                float x = (obj.bb_r - obj.bb_l) * coef;
                float y = (obj.bb_t - obj.bb_b) * coef;

                var v = cpVect.cpvmult(velocityFunc(obj), 0.1f);

                dest.bb_l = obj.bb_l + Math.Min(-x, v.x);
                dest.bb_b = obj.bb_b + Math.Min(-y, v.y);
                dest.bb_r = obj.bb_r + Math.Max(x, v.x);
                dest.bb_t = obj.bb_t + Math.Max(y, v.y);
            }
            else
            {
                dest.bb_l = obj.bb_l;
                dest.bb_b = obj.bb_b;
                dest.bb_r = obj.bb_r;
                dest.bb_t = obj.bb_t;
            }



        }

        public int getStamp()
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


        public void reindexQuery(Action<object, object> func)
        {

            if (this.root == null) return;

            // LeafUpdate() may modify tree->root. Don't cache it.
            foreach (var item in leaves)
                item.Value.update(this);//  LeafUpdate()

            var staticIndex = this.staticIndex;
            Node staticRoot = staticIndex != null ? staticIndex.root : null;

            this.root.markSubtree(this, staticRoot, func);

            if (staticIndex != null && staticRoot == null)
                CollideStatic(staticIndex, func); //, data);

            incrementStamp();

        }

        public void reindex()
        {
            reindexQuery(voidQueryFunc); //cpBBTreeReindexQuery(tree, VoidQueryFunc, null);
        }

        public void voidQueryFunc(object obj1, object obj2) { }

        public void reindexObject(string key, object obj)
        {
            Leaf leaf;
            if (leaves.TryGetValue(key, out leaf))
            {

                if (leaf.update(this))
                    leaf.addPairs(this);

                incrementStamp();
            }
        }

        public void PointQuery(cpVect point, Action<object, object> func)
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
            if (subtree.isLeaf)
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
            float tx1 = (node.bb_l == a.x ? -cp.Infinity : (node.bb_l - a.x) * idx);
            float tx2 = (node.bb_r == a.x ? cp.Infinity : (node.bb_r - a.x) * idx);
            float txmin = Math.Min(tx1, tx2);
            float txmax = Math.Max(tx1, tx2);

            float idy = 1 / (b.y - a.y);
            float ty1 = (node.bb_b == a.y ? -cp.Infinity : (node.bb_b - a.y) * idy);
            float ty2 = (node.bb_t == a.y ? cp.Infinity : (node.bb_t - a.y) * idy);
            float tymin = Math.Min(ty1, ty2);
            float tymax = Math.Max(ty1, ty2);

            if (tymin <= txmax && txmin <= tymax)
            {
                var min_ = Math.Max(txmin, tymin);
                var max_ = Math.Min(txmax, tymax);

                if (0.0 <= max_ && min_ <= 1.0f) return Math.Max(min_, 0.0f);
            }

            return cp.Infinity;
        }

        public void query(cpBB bb, Action<object, object> func)
        {
            if (root != null)
                subtreeQuery(root, bb, func);
        }
        public void subtreeQuery(Node subtree, cpBB bb, Action<object, object> func)
        {
            //if(bbIntersectsBB(subtree.bb, bb)){
            if (subtree.intersectsBB(bb))
            {
                if (subtree.isLeaf)
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

            root = cp.partitionNodes(this, nodes, 0, nodes.Count);
        }

        //public void PairInsert(Leaf a, Leaf b)
        //{
        //    Pair nextA = a.pairs, nextB = b.pairs;
        //    var pair = MakePair(a, nextA, b, nextB);
        //    a.pairs = b.pairs = pair;

        //    if (nextA != null)
        //    {
        //        if (nextA.leafA == a) nextA.prevA = pair; else nextA.prevB = pair;
        //    }

        //    if (nextB != null)
        //    {
        //        if (nextB.leafA == b) nextB.prevA = pair; else nextB.prevB = pair;
        //    }
        //}

        public void Log()
        {
            if (this.root != null)
                cp.nodeRender(this.root, 0);
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
        //public Node SubtreeInsert(Node subtree, Leaf leaf)
        //{
        //    if (subtree == null)
        //    {
        //        return leaf;
        //    }
        //    else if (subtree.isLeaf)
        //    {
        //        return makeNode((Node)leaf, subtree);
        //    }
        //    else
        //    {
        //        var cost_a = subtree.B.bb.Area() + subtree.A.bb.MergedArea(leaf.bb);
        //        var cost_b = subtree.A.bb.Area() + subtree.B.bb.MergedArea(leaf.bb);

        //        if (cost_a == cost_b)
        //        {
        //            cost_a = cpEnvironment.bbProximity(subtree.A, leaf);
        //            cost_b = cpEnvironment.bbProximity(subtree.B, leaf);
        //        }

        //        if (cost_b < cost_a)
        //        {
        //            subtree.setB(SubtreeInsert(subtree.B, leaf));
        //        }
        //        else
        //        {
        //            subtree.setA(SubtreeInsert(subtree.A, leaf));
        //        }

        //        subtree.bb.l = Math.Min(subtree.bb.l, leaf.bb.l);
        //        subtree.bb.b = Math.Min(subtree.bb.b, leaf.bb.b);
        //        subtree.bb.r = Math.Max(subtree.bb.r, leaf.bb.r);
        //        subtree.bb.t = Math.Max(subtree.bb.t, leaf.bb.t);

        //        return subtree;
        //    }


        //}

        public object GetValue(string arbHash)
        {
            Leaf dev = Get(arbHash);
            if (dev != null)
                return dev.obj;
            return null;
        }

        public Leaf Get(string arbHash)
        {
            Leaf dev;
            if (TryGetValue(arbHash, out dev))
                return dev;
            return null;
        }

        //public void getBB(IObjectBox obj, Node dest)
        //{
        //    var velocityFunc = this.velocityFunc;
        //    if (velocityFunc != null)
        //    {
        //        float coef = 0.1f;
        //        float x = (obj.bb.r - obj.bb.l) * coef;
        //        float y = (obj.bb.t - obj.bb.b) * coef;

        //        var v = cpVect.cpvmult(velocityFunc(obj), 0.1f);

        //        dest.bb.l = obj.bb.l + cpEnvironment.cpfmin(-x, v.x);
        //        dest.bb.b = obj.bb.b + cpEnvironment.cpfmin(-y, v.y);
        //        dest.bb.r = obj.bb.r + cpEnvironment.cpfmax(x, v.x);
        //        dest.bb.t = obj.bb.t + cpEnvironment.cpfmax(y, v.y);
        //    }
        //    else
        //    {
        //        dest.bb.l = obj.bb.l;
        //        dest.bb.b = obj.bb.b;
        //        dest.bb.r = obj.bb.r;
        //        dest.bb.t = obj.bb.t;
        //    }
        //}
    }






}