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
		//float bb_l { get; set; }
		//float bb_b { get; set; }
		//float bb_r { get; set; }
		//float bb_t { get; set; }
	}

	//MARK: Spatial Index

	/// Bounding box tree velocity callback function.
	/// This function should return an estimate for the object's velocity.
	//public delegate cpVect cpBBTreeVelocityFunc(object obj);

	public class Node : IObjectBox
	{

		public bool isLeaf;

		private Node a { get; set; }
		private Node b { get; set; }

		public Node A { get { return a; } }
		public Node B { get { return b; } }

		public int stamp;

		public int STAMP { get { return stamp; } set { stamp = value; } }

		public IObjectBox obj;

		public cpBB bb { get; set; }

		public Node parent;

		public Pair pairs;
		public Pair PAIRS { get { return pairs; } set { pairs = value; } }
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

			this.SetA(a);
			this.SetB(b);
		}

		public Node()
		{
			// TODO: Complete member initialization
			this.bb = new cpBB(0, 0, 0, 0);
		}

		public Node OtherChild(Node child)
		{
			return (this.A == child ? this.B : this.A);
		}

		public void ReplaceChild(Node child, Node value, cpBBTree tree)
		{

			cp.assertSoft(child == this.A || child == this.B, "Node is not a child of parent.");

			if (this.A == child)
			{
				this.A.Recycle(tree);
				this.SetA(value);
			}
			else
			{
				this.B.Recycle(tree);
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

		public virtual void MarkLeafQuery(Leaf leaf, bool left, cpBBTree tree, Action<object, object> func)
		{

			if (cp.bbTreeIntersectsNode(leaf, this))
			{
				this.A.MarkLeafQuery(leaf, left, tree, func);
				this.B.MarkLeafQuery(leaf, left, tree, func);
			}
		}

		public virtual void MarkSubtree(cpBBTree tree, Node staticRoot, Action<object, object> func)
		{
			this.A.MarkSubtree(tree, staticRoot, func);
			this.B.MarkSubtree(tree, staticRoot, func);
		}

		public float bbArea()
		{
			return (this.bb.r - this.bb.l) * (this.bb.t - this.bb.b);
		}

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

		public virtual void Recycle(cpBBTree tree)
		{
			this.parent = tree.pooledNodes;
			tree.pooledNodes = this;
		}

		public bool IntersectsBB(cpBB bb)
		{
			return (this.bb.l <= bb.r && bb.l <= this.bb.r && this.bb.b <= bb.t && bb.b <= this.bb.t);
		}

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
			tree.GetBB(obj, this);

			this.parent = null;

			this.stamp = 1;
			this.pairs = null;

			cp.numLeaves++;
		}

		public void ClearPairs(cpBBTree tree)
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

		public override void Recycle(cpBBTree tree)
		{
		}



		public override void MarkLeafQuery(Leaf leaf, bool left, cpBBTree tree, Action<object, object> func)
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

		public override void MarkSubtree(cpBBTree tree, Node staticRoot, Action<object, object> func)
		{
			if (this.stamp == tree.GetStamp())
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

		public bool ContainsObj(object objData)
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

			var obj = this.obj;

			if (!this.ContainsObj(obj))
			{

				tree.GetBB(this.obj, this);

				root = cp.SubtreeRemove(root, this, tree);
				tree.root = cp.subtreeInsert(root, this, tree);//tree.root = SubtreeInsert(root, this, tree);
				this.ClearPairs(tree);
				this.stamp = tree.GetStamp();

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
				var staticRoot = tree.staticIndex.root;
				this.MarkSubtree(tree, staticRoot, null);
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

				Each((obj) =>
				{
					staticIndex.Query(
						new cpBB(obj.bb.l, obj.bb.b, obj.bb.r, obj.bb.t),
						func);
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
				node.Recycle(this);
			}
		}

		public Leaf Insert(string hashid, IObjectBox obj)
		{
			var leaf = new Leaf(this, obj);
			this.leaves.Add(hashid, leaf);

			this.root = cp.subtreeInsert(root, leaf, this);

			leaf.stamp = GetStamp();
			leaf.AddPairs(this);
			IncrementStamp();
			return leaf;
		}

		public void Remove(string key)
		{
			Leaf leaf;
			if (TryGetValue(key, out leaf))
			{
				//remove elements adds more functionality than simple array
				leaves.Remove(key);

				//if (root != null)

				this.root = cp.SubtreeRemove(this.root, leaf, this);

				leaf.ClearPairs(this);
				leaf.Recycle(this);
			}
		}

		public bool ContainsValue(object obj)
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

		public Node MakeNode(Node a, Node b)
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

		public void GetBB(object objElement, IObjectBox dest)
		{

			//TODO: GETBB

			IObjectBox obj = objElement as IObjectBox;
			var velocityFunc = this.velocityFunc;
			if (velocityFunc != null)
			{
				float coef = 0.1f;
				float x = (obj.bb.r - obj.bb.l) * coef;
				float y = (obj.bb.t - obj.bb.b) * coef;

				var v = cpVect.cpvmult(velocityFunc(obj), 0.1f);

				dest.bb.l = obj.bb.l + Math.Min(-x, v.x);
				dest.bb.b = obj.bb.b + Math.Min(-y, v.y);
				dest.bb.r = obj.bb.r + Math.Max(x, v.x);
				dest.bb.t = obj.bb.t + Math.Max(y, v.y);
			}
			else
			{
				dest.bb.l = obj.bb.l;
				dest.bb.b = obj.bb.b;
				dest.bb.r = obj.bb.r;
				dest.bb.t = obj.bb.t;
			}



		}

		public int GetStamp()
		{
			var dynamic = this.dynamicIndex;
			return (dynamic != null && dynamic.stamp != 0 ? dynamic.stamp : this.stamp);
		}

		public void IncrementStamp()
		{
			//  cpBBTree dynamicTree = tree.dynamicIndex;
			if (dynamicIndex != null && this.dynamicIndex.stamp != 0)
				dynamicIndex.stamp++;
			else
				stamp++;
		}


		public void ReindexQuery(Action<object, object> func)
		{

			if (this.root == null) return;

			// LeafUpdate() may modify tree->root. Don't cache it.
			foreach (var item in leaves)
				item.Value.Update(this);//  LeafUpdate()

			var staticIndex = this.staticIndex;
			Node staticRoot = staticIndex != null ? staticIndex.root : null;

			this.root.MarkSubtree(this, staticRoot, func);

			if (staticIndex != null && staticRoot == null)
				CollideStatic(staticIndex, func); //, data);

			IncrementStamp();

		}

		public void Reindex()
		{
			ReindexQuery(VoidQueryFunc);
		}

		public void VoidQueryFunc(object obj1, object obj2) { }

		public void ReindexObject(string key, object obj)
		{
			Leaf leaf;
			if (leaves.TryGetValue(key, out leaf))
			{

				if (leaf.Update(this))
					leaf.AddPairs(this);

				IncrementStamp();
			}
		}

		public void PointQuery(cpVect point, Action<object, object> func)
		{
			Query(new cpBB(point.x, point.y, point.x, point.y), func);
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
			float tx1 = (node.bb.l == a.x ? -cp.Infinity : (node.bb.l - a.x) * idx);
			float tx2 = (node.bb.r == a.x ? cp.Infinity : (node.bb.r - a.x) * idx);
			float txmin = Math.Min(tx1, tx2);
			float txmax = Math.Max(tx1, tx2);

			float idy = 1 / (b.y - a.y);
			float ty1 = (node.bb.b == a.y ? -cp.Infinity : (node.bb.b - a.y) * idy);
			float ty2 = (node.bb.t == a.y ? cp.Infinity : (node.bb.t - a.y) * idy);
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

		public void Query(cpBB bb, Action<object, object> func)
		{
			if (root != null)
				SubtreeQuery(root, bb, func);
		}
		public void SubtreeQuery(Node subtree, cpBB bb, Action<object, object> func)
		{
			//if(bbIntersectsBB(subtree.bb, bb)){
			if (subtree.IntersectsBB(bb))
			{
				if (subtree.isLeaf)
				{
					func(subtree.obj, null);
				}
				else
				{
					SubtreeQuery(subtree.A, bb, func);
					SubtreeQuery(subtree.B, bb, func);
				}
			}
		}

		public void Each(Action<IObjectBox> func)
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

		public void Log()
		{
			if (this.root != null)
				cp.nodeRender(this.root, 0);
		}

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

	}

}