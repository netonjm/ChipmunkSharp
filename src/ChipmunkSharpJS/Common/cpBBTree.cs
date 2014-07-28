﻿/* Copyright (c) 2007 Scott Lembcke ported by Jose Medrano (@netonjm)
  
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
	//public delegate cpVect cpBBTreeVelocityFunc(object obj);
	public struct MarkContext
	{
		public cpBBTree tree;
		public Node staticRoot;
		public Func<object, object, string, object, string> func;
		public object data;

		public MarkContext(cpBBTree tree, Node staticRoot, Func<object, object, string, object, string> func, object data)
		{
			this.tree = tree;
			this.staticRoot = staticRoot;
			this.func = func;
			this.data = data;
		}

	}

	public class Node : IObjectBox
	{

		public bool isLeaf;

		private Node a { get; set; }
		private Node b { get; set; }

		public IObjectBox obj;

		public cpBB bb { get; set; }

		public Node parent;

		public Pair pairs;

		public int stamp;

		public Node A { get { return a; } }
		public Node B { get { return b; } }

		public int STAMP { get { return stamp; } set { stamp = value; } }
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


		public Node(Node a, Node b, cpBBTree tree)
		{
			this.obj = null;

			bb = cpBB.Merge(a.bb, b.bb);

			parent = null;

			this.SetA(a);
			this.SetB(b);
		}

		public Node()
		{
			// TODO: Complete member initialization
			this.bb = new cpBB(0, 0, 0, 0);
		}

		public Node Other(Node child)
		{
			return (this.A == child ? this.B : this.A);
		}

		public void ReplaceChild(Node child, Node value, cpBBTree tree)
		{

			cp.assertSoft(child == this.A || child == this.B, "Node is not a child of parent.");

			if (this.A == child)
			{
				tree.NodeRecycle(this.A);//.Recycle(tree);
				this.SetA(value);
			}
			else
			{
				tree.NodeRecycle(this.B);
				this.SetB(value);
			}

			for (var node = this; node != null; node = node.parent)
			{
				node.bb = node.A.bb.Merge(node.B.bb);
			}
		}

		//MARK: Subtree Functions

		public virtual void MarkLeafQuery(Node leaf, bool left, MarkContext context)
		{

			if (leaf.bb.Intersects(this.bb))
			{
				if (this.isLeaf)
				{
					if (left)
					{
						context.tree.PairInsert(leaf, this);
					}
					else
					{
						if (this.STAMP < leaf.STAMP)
							context.tree.PairInsert(this, leaf);// this.PairInsert(leaf, );

						context.func(leaf.obj, this.obj, "0", context.data);
					}
				}
				else
				{
					this.A.MarkLeafQuery(leaf, left, context);
					this.B.MarkLeafQuery(leaf, left, context);
				}
			}
		}

		public void MarkLeaf(MarkContext context)
		{
			cpBBTree tree = context.tree;

			if (this.STAMP == tree.GetMasterTree().stamp)   //tree. GetMasterTree(tree).stamp)
			{

				Node staticRoot = context.staticRoot;
				if (staticRoot != null)
					staticRoot.MarkLeafQuery(this, false, context);

				for (Node node = this; node.parent != null; node = node.parent)
				{
					if (node == node.parent.A)
					{
						node.parent.B.MarkLeafQuery(this, true, context); // tree, context);
					}
					else
					{
						node.parent.A.MarkLeafQuery(this, false, context);
					}
				}
			}
			else
			{
				Pair pair = this.PAIRS;
				while (pair != null)
				{
					if (this == pair.b.leaf) // leafB)
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


		public virtual void MarkSubtree(MarkContext context)
		{
			if (isLeaf)
				MarkLeaf(context);
			else
			{
				this.A.MarkSubtree(context);
				this.B.MarkSubtree(context);
			}
		}



		/// ////////////////////////////////////////////////////////////////////




		[Obsolete]
		public virtual void MarkLeafQuery(Leaf leaf, bool left, cpBBTree tree, Action<object, object> func)
		{

			if (cp.bbTreeIntersectsNode(leaf, this))
			{
				this.A.MarkLeafQuery(leaf, left, tree, func);
				this.B.MarkLeafQuery(leaf, left, tree, func);
			}
		}

		//MARK: Marking Functions
		public float bbArea()
		{
			return (this.bb.r - this.bb.l) * (this.bb.t - this.bb.b);
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

			this.STAMP = 0;
			this.PAIRS = null;

			cp.numLeaves++;
		}



		//MARK: Marking Functions
		//public bool Update(cpBBTree tree)
		//{

		//	var root = tree.root;

		//	var obj = this.obj;

		//	if (!this.ContainsObj(obj))
		//	{

		//		tree.GetBB(this.obj, this);

		//		root = cp.SubtreeRemove(root, this, tree);
		//		tree.root = cp.subtreeInsert(root, this, tree);//tree.root = SubtreeInsert(root, this, tree);
		//		this.PairsClear(tree);
		//		this.stamp = tree.GetStamp();

		//		return true;
		//	}
		//	return false;
		//}

		////////////////////////////////////////////////////////////////////////////////////

		//public override void Recycle(cpBBTree tree)
		//{
		//}



		public override void MarkLeafQuery(Leaf leaf, bool left, cpBBTree tree, Action<object, object> func)
		{
			if (cp.bbTreeIntersectsNode(leaf, this))
			{
				if (left)
				{
					tree.PairInsert(leaf, this);
					//cp.pairInsert(leaf, this, tree);
				}
				else
				{
					if (this.stamp < leaf.stamp)
						tree.PairInsert(this, leaf);
					//cp.pairInsert(this, leaf, tree);
					if (func != null)
						func(leaf.obj, this.obj);
				}
			}
		}


		//public override void MarkSubtree(cpBBTree tree, Node staticRoot, Action<object, object> func)
		//{
		//	if (this.stamp == tree.GetStamp())
		//	{
		//		if (staticRoot != null) staticRoot.MarkLeafQuery(this, false, tree, func);

		//		for (Node node = this; node.parent != null; node = node.parent)
		//		{
		//			if (node == node.parent.A)
		//			{
		//				node.parent.B.MarkLeafQuery(this, true, tree, func);
		//			}
		//			else
		//			{
		//				node.parent.A.MarkLeafQuery(this, false, tree, func);
		//			}
		//		}
		//	}
		//	else
		//	{
		//		var pair = this.pairs;
		//		while (pair != null)
		//		{
		//			if (this == pair.b.leaf)
		//			{
		//				if (func != null)
		//					func(pair.a.leaf.obj, this.obj);

		//				pair = pair.b.next;
		//			}
		//			else
		//			{
		//				pair = pair.a.next;
		//			}
		//		}
		//	}
		//}

		public bool ContainsObj(object objData)
		{
			var obj = objData as IObjectBox;
			if (obj == null)
				return false;

			return (this.bb.l <= obj.bb.l && this.bb.r >= obj.bb.r && this.bb.b <= obj.bb.b && this.bb.t >= obj.bb.t);
		}

	}

	public class Thread
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

		public void Unlink()
		{
			ThreadUnlink(this);
		}

		public static void ThreadUnlink(Thread thread)
		{

			Pair next = thread.next;
			Pair prev = thread.prev;

			if (next != null)
			{
				if (next.a.leaf == thread.leaf)
					next.a.prev = prev;
				else next.b.prev = prev;
			}

			if (prev != null)
			{
				if (prev.a.leaf == thread.leaf)
					prev.a.next = next;
				else prev.b.next = next;
			}
			else
			{
				thread.leaf.PAIRS = next;
			}
		}

	}

	public class Pair
	{


		public Thread a, b;
		public string id;

		// Objects created with constructors are faster than object literals. :(
		public Pair(Node leafA, Pair nextA, Node leafB, Pair nextB)
		{
			a = new Thread(null, leafA, nextA);
			b = new Thread(null, leafB, nextB);
			id = "0";
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
		public Node root { get; set; }

		public cpBBTree staticIndex { get; set; }
		public cpBBTree dynamicIndex { get; set; }

		public Dictionary<string, Leaf> leaves { get; set; }

		public Node pooledNodes { get; set; }
		public Pair pooledPairs { get; set; }

		public int stamp { get; set; }

		public Func<object, cpVect> velocityFunc { get; set; }

		public int Count
		{
			get
			{
				return leaves.Count;
			}
		}


		public cpBB GetBB(IObjectBox obj)
		{
			cpBB bb = obj.bb;

			var velocityFunc = this.velocityFunc;// tree->velocityFunc;
			if (velocityFunc != null)
			{
				float coef = 0.1f;
				float x = (bb.r - bb.l) * coef;
				float y = (bb.t - bb.b) * coef;

				cpVect v = cpVect.cpvmult(velocityFunc(obj), 0.1f);
				return new cpBB(bb.l + cp.cpfmin(-x, v.x), bb.b + cp.cpfmin(-y, v.y), bb.r + cp.cpfmax(x, v.x), bb.t + cp.cpfmax(y, v.y));
			}
			else
			{
				return bb;
			}
		}

		//MARK: Misc Functions
		[Obsolete]
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

		public Node GetRootIfTree()
		{
			return root;
		}

		public cpBBTree GetMasterTree()
		{
			cpBBTree dynamicTree = this.dynamicIndex;
			return (dynamicTree != null ? dynamicTree : this);
		}

		public void IncrementStamp()
		{
			//  cpBBTree dynamicTree = tree.dynamicIndex;
			if (dynamicIndex != null && this.dynamicIndex.stamp != 0)
				dynamicIndex.stamp++;
			else
				stamp++;
		}

		public void PairRecycle(Pair pair)
		{

			//TODO: CHECK IF WORKS
			// Share the pool of the master tree.
			// TODO: would be lovely to move the pairs stuff into an external data structure.
			var tree = GetMasterTree();

			pair.a.next = tree.pooledPairs;
			tree.pooledPairs = pair;
		}

		public void PairsClear(Node leaf)
		{

			Pair pair = leaf.pairs;
			Pair next;
			leaf.PAIRS = null;

			while (pair != null)
			{
				if (pair.a.leaf == leaf)
				{
					next = pair.a.next;
					pair.b.Unlink();
					PairRecycle(pair);
					pair = next;

				}
				else
				{
					next = pair.b.next;
					pair.a.Unlink();// ThreadUnlink();
					PairRecycle(pair);
					pair = next;
				}

			}

		}


		public Pair MakePair(Node leafA, Pair nextA, Node leafB, Pair nextB)
		{
			var pair = this.pooledPairs;
			if (pair != null)
			{
				this.pooledPairs = pair.a.prev;

				pair.a.prev = null;
				pair.a.leaf = leafA;
				pair.a.next = nextA;

				pair.b.prev = null;
				pair.b.leaf = leafB;
				pair.b.next = nextB;

				return pair;
			}
			else
			{
				cp.numPairs++;
				return new Pair(leafA, nextA, leafB, nextB);
			}
		}


		public void PairInsert(Node a, Node b)
		{
			Pair nextA = a.PAIRS, nextB = b.PAIRS;
			Pair pair = MakePair(a, nextA, b, nextB);


			a.pairs = b.pairs = pair;

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

		public virtual void NodeRecycle(Node node)
		{

			node.parent = this.pooledNodes;
			this.pooledNodes = node;
		}

		public Node SubtreeInsert(Node subtree, Leaf leaf)
		{
			if (subtree == null)
			{
				return leaf;
			}
			else if (subtree.isLeaf)
			{
				return new Node(leaf, subtree, this);
			}
			else
			{
				float cost_a = subtree.B.bb.Area() + subtree.A.bb.MergedArea(leaf.bb);// cpBBMergedArea(subtree->A->bb, leaf->bb);
				float cost_b = subtree.A.bb.Area() + subtree.B.bb.MergedArea(leaf.bb);  //cpBBArea(subtree->A->bb) + cpBBMergedArea(subtree->B->bb, leaf->bb);

				if (cost_a == cost_b)
				{
					cost_a = subtree.A.bb.Proximity(leaf.bb);
					cost_b = subtree.B.bb.Proximity(leaf.bb);// cpBBProximity(subtree->B->bb, leaf->bb);
				}

				if (cost_b < cost_a)
				{
					subtree.SetB(SubtreeInsert(subtree.B, leaf));
				}
				else
				{
					subtree.SetA(SubtreeInsert(subtree.A, leaf));
				}

				subtree.bb = subtree.bb.Merge(leaf.bb);// cpBBMerge(subtree->bb, leaf->bb);
				return subtree;
			}

		}

		public void SubtreeQuery(Node subtree, object obj, cpBB bb, Func<object, object, string, object, string> func, object data)
		{
			//if(bbIntersectsBB(subtree.bb, bb)){
			if (subtree.bb.Intersects(bb))
			{
				if (subtree.isLeaf)
				{
					func(obj, subtree.obj, "0", data);
				}
				else
				{
					SubtreeQuery(subtree.A, obj, bb, func, data);
					SubtreeQuery(subtree.B, obj, bb, func, data);
				}
			}
		}

		public float SubtreeSegmentQuery(Node subtree, object obj, cpVect a, cpVect b, float t_exit, Func<object, object, object, float> func, object data)
		{
			if (subtree.isLeaf)
			{
				return func(obj, subtree.obj, data);
			}
			else
			{
				float t_a = subtree.A.bb.SegmentQuery(a, b);
				float t_b = subtree.B.bb.SegmentQuery(a, b);

				if (t_a < t_b)
				{
					if (t_a < t_exit) t_exit = cp.cpfmin(t_exit, SubtreeSegmentQuery(subtree.A, obj, a, b, t_exit, func, data));
					if (t_b < t_exit) t_exit = cp.cpfmin(t_exit, SubtreeSegmentQuery(subtree.B, obj, a, b, t_exit, func, data));
				}
				else
				{
					if (t_b < t_exit) t_exit = cp.cpfmin(t_exit, SubtreeSegmentQuery(subtree.B, obj, a, b, t_exit, func, data));
					if (t_a < t_exit) t_exit = cp.cpfmin(t_exit, SubtreeSegmentQuery(subtree.A, obj, a, b, t_exit, func, data));
				}

				return t_exit;
			}
		}

		public void SubtreeRecycle(Node node)
		{
			if (!node.isLeaf)
			{
				SubtreeRecycle(node.A);
				SubtreeRecycle(node.B);
				NodeRecycle(node);
			}
		}

		public Node SubtreeRemove(Node subtree, Leaf leaf)
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
					var other = subtree.Other(leaf);
					other.parent = subtree.parent;
					NodeRecycle(subtree);
					return other;
				}
				else
				{
					if (parent == null)
						return null;

					parent.parent.ReplaceChild(parent, parent.Other(leaf), this);
					return subtree;
				}
			}
		}


		//MARK: Marking Functions


		public bool LeafUpdate(Node leaf)
		{
			Node root = this.root;
			cpBB bb = leaf.obj.bb;// spatialIndex.bbfunc(leaf->obj);

			if (!leaf.bb.ContainsBB(bb))
			{
				leaf.bb = GetBB(leaf.obj);

				root = SubtreeRemove(root, leaf as Leaf);
				this.root = SubtreeInsert(root, leaf as Leaf);

				PairsClear(leaf);
				leaf.STAMP = GetMasterTree().stamp;

				return true;
			}
			else
			{
				return false;
			}
		}

		static string VoidQueryFunc(object obj1, object obj2, string id, object data) { return id; }

		public void LeafAddPairs(Node leaf)
		{
			cpBBTree dynamicIndex = this.dynamicIndex;

			if (dynamicIndex != null)
			{
				Node dynamicRoot = dynamicIndex.GetRootIfTree();

				if (dynamicRoot != null)
				{
					cpBBTree dynamicTree = dynamicIndex;
					MarkContext context = new MarkContext(dynamicTree, null, null, null);
					//dynamicRoot.MarkLeafQuery(this, true, dynamicIndex, null);
					dynamicRoot.MarkLeafQuery(leaf, true, context);
				}
			}
			else
			{
				var staticRoot = this.staticIndex.GetRootIfTree();
				MarkContext context = new MarkContext(this, staticRoot, VoidQueryFunc, null);
				leaf.MarkLeaf(context);
			}
		}
		public Leaf Insert(string hashid, IObjectBox obj)
		{
			Leaf leaf = new Leaf(this, obj);
			this.leaves.Add(hashid, leaf);

			Node root = this.root;
			this.root = SubtreeInsert(root, leaf);


			//this.root = cp.subtreeInsert(root, leaf, this);

			leaf.STAMP = GetMasterTree().stamp;
			LeafAddPairs(leaf); //.AddPairs(this);
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

				this.root = this.SubtreeRemove(this.root, leaf);// cp.SubtreeRemove(this.root, leaf, this);
				PairsClear(leaf);
				NodeRecycle(leaf);
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

		public void SetVelocityFunc(Func<object, cpVect> func)
		{
			this.velocityFunc = func;

		}

		public void LeafUpdateWrap(Node leaf)
		{
			LeafUpdate(leaf);
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

		public void ReindexQuery(Func<object, object, string, object, string> func, object data)
		{

			if (this.root == null) return;

			// LeafUpdate() may modify tree->root. Don't cache it.
			foreach (var item in leaves)
				this.LeafUpdateWrap(item.Value);//  LeafUpdate()

			var staticIndex = this.staticIndex;
			Node staticRoot = staticIndex != null ? staticIndex.root : null;


			MarkContext context = new MarkContext(this, staticRoot, func, data);
			this.root.MarkSubtree(context);

			if (staticIndex != null && staticRoot == null)
				CollideStatic(staticIndex, func, data);

			IncrementStamp();
		}

		// Collide the objects in an index against the objects in a staticIndex using the query callback function.
		public void CollideStatic(cpBBTree staticIndex, Func<object, object, string, object, string> func, object data)
		{
			if (staticIndex != null && staticIndex.Count > 0)
			{
				Each((obj) =>
				{
					//	dynamicToStaticContext context = new dynamicToStaticContext(dynamicIndex->bbfunc, staticIndex, func, data);
					staticIndex.Query(staticIndex,
						new cpBB(obj.bb.l, obj.bb.b, obj.bb.r, obj.bb.t),
						func, data);
				});
			}
		}

		public void Reindex()
		{
			ReindexQuery(VoidQueryFunc, null);
		}


		public void ReindexObject(object obj, string hashid)
		{
			Leaf leaf;
			if (leaves.TryGetValue(hashid, out leaf))
			{

				if (this.LeafUpdate(leaf))
					LeafAddPairs(leaf);

				IncrementStamp();
			}
		}

		//MARK: Query
		public void SegmentQuery(object obj, cpVect a, cpVect b, float t_exit, Func<object, object, object, float> func, object data)
		{
			//Node* root = root;
			if (root != null)
				SubtreeSegmentQuery(this.root, obj, a, b, t_exit, func, data);
		}


		public void Query(object context, cpBB bb, Func<object, object, string, object, string> func, object node)
		{
			if (root != null)
				SubtreeQuery(root, context, bb, func, node);
		}


		public Node PartitionNodes(Dictionary<int, Leaf> nodes, int offset, int count)
		{
			//int count = nodes.Count;
			//int offset = 0;

			if (count == 1)
			{
				return nodes[0];
			}
			else if (count == 2)
			{
				return MakeNode(nodes[offset], nodes[offset + 1]);
			}

			// Find the AABB for these nodes

			cpBB bb = nodes[0].bb;
			for (int i = 1; i < count; i++)
				bb = bb.Merge(nodes[i].bb);


			// Split it on it's longest axis
			var splitWidth = (bb.r - bb.l > bb.t - bb.b);

			// Sort the bounds and use the median as the splitting point
			float[] bounds = new float[count * 2];
			if (splitWidth)
			{
				for (var i = offset; i < count; i++)
				{
					bounds[2 * i + 0] = nodes[i].bb.l;
					bounds[2 * i + 1] = nodes[i].bb.r;
				}
			}
			else
			{
				for (var i = offset; i < count; i++)
				{
					bounds[2 * i + 0] = nodes[i].bb.b;
					bounds[2 * i + 1] = nodes[i].bb.t;
				}
			}

			//TODO: ¿?

			float split = (bounds[count - 1] + bounds[count]) * 0.5f; // use the median as the split

			// Generate the child BBs
			//var a = bb, b = bb;
			cpBB a = bb, b = bb;
			if (splitWidth) a.r = b.l = split; else a.t = b.b = split;

			// Partition the nodes
			var right = count;

			for (var left = offset; left < right; )
			{
				Node node = nodes[left];
				//	if(bbMergedArea(node.bb, b) < bbMergedArea(node.bb, a)){
				if (node.bb.MergedArea(b) < node.bb.MergedArea(a))
				{
					right--;
					nodes[left] = nodes[right];
					nodes[right] = node as Leaf;
				}
				else
				{
					left++;
				}
			}

			if (right == count)
			{
				Node tmp = null;
				for (var i = offset; i < count; i++)
					tmp = SubtreeInsert(tmp, nodes[i]);
				return tmp;
			}

			// Recurse and build the node!
			return new Node(
				PartitionNodes(nodes, offset, right - offset),
				PartitionNodes(nodes, right, count - right), this
			);


		}

		public void Optimize()
		{
			//TODO: REVISE OPTIMIZATION
			var nodes = new Dictionary<int, Leaf>(this.Count);
			var i = 0;
			foreach (var hashid in leaves)
				nodes.Add(i++, hashid.Value);
			SubtreeRecycle(root);

			root = PartitionNodes(nodes, 0, nodes.Count);
		}


		/////////////////////////////////////////////////



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



		public int GetStamp()
		{
			var dynamic = this.dynamicIndex;
			return (dynamic != null && dynamic.stamp != 0 ? dynamic.stamp : this.stamp);
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






		public void Each(Action<IObjectBox> func)
		{
			foreach (var item in leaves)
				func(item.Value.obj);
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

