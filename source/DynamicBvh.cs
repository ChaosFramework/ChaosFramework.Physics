/* This file is based on Erin Catto's implementation of dynamic bounding volume hierarchies in box2D.
 * See
 * - https://github.com/erincatto/box2d
 * - https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf
 *
 * For license information check the 'License' directory of this repository.
 */

using ChaosFramework.Collections;
using ChaosFramework.Math;
using SysCol = System.Collections.Generic;
using static ChaosFramework.Math.Clamping;

namespace ChaosFramework.Physics
{
    public partial class DynamicBvh
    {
        static void Combine(ref Bounds3f target, Bounds3f a, Bounds3f b)
        {
            target.low = Min(a.low, b.low);
            target.high = Max(a.high, b.high);
        }

        BvhNode root;

        public bool empty
            => root == null;

        internal SysCol.IEnumerable<BvhNode> EnumerateLeaves()
            => EnumerateLeaves(root);

        SysCol.IEnumerable<BvhNode> EnumerateLeaves(BvhNode n)
        {
            if (n == null)
                yield break;

            if (n.isLeaf)
                yield return n;
            else
            {
                foreach (BvhNode n1 in EnumerateLeaves(n.child1))
                    yield return n1;

                foreach (BvhNode n2 in EnumerateLeaves(n.child2))
                    yield return n2;
            }
        }

        public SysCol.IEnumerable<BvhNode> RecursiveDetect(BvhNode leaf)
            => RecursiveDetect(leaf, root);

        SysCol.IEnumerable<BvhNode> RecursiveDetect(BvhNode leaf, BvhNode node)
        {
            if (node == null || leaf == node || !leaf.aabb.Intersects(node.aabb))
                yield break;

            if (node.isLeaf)
                yield return node;
            else
            {
                foreach (BvhNode n1 in RecursiveDetect(leaf, node.child1))
                    yield return n1;

                foreach (BvhNode n2 in RecursiveDetect(leaf, node.child2))
                    yield return n2;
            }
        }

        public SysCol.IEnumerable<BvhNode> RecursiveDetect(Ray ray)
            => RecursiveDetect(ray, root);

        SysCol.IEnumerable<BvhNode> RecursiveDetect(Ray ray, BvhNode node)
        {
            if (Intersection.Intersect(ray, node.aabb))
                if (node.isLeaf)
                    yield return node;
                else
                {
                    foreach (BvhNode n1 in RecursiveDetect(ray, node.child1))
                        yield return n1;

                    foreach (BvhNode n2 in RecursiveDetect(ray, node.child2))
                        yield return n2;
                }
        }

        internal bool ContainsNode(BvhNode leaf)
            => ContainsNode(leaf, root);

        bool ContainsNode(BvhNode leaf, BvhNode n)
        {
            if (n == null)
                return false;

            if (n == leaf)
                return true;
            else
                return ContainsNode(leaf, n.child1) 
                    || ContainsNode(leaf, n.child2);
        }

        public void InsertLeaf(BvhNode leaf)
        {
            AssertInsertOnce(leaf);
            leaf.parent = null;

            if (root == null)
            {
                root = leaf;
                return;
            }

            // Find the best sibling for this node
            Bounds3f leafAabb = leaf.aabb;
            BvhNode index = root;
            while (!index.isLeaf)
            {
                BvhNode child1 = index.child1;
                BvhNode child2 = index.child2;

                float volume = index.aabb.volume;

                Bounds3f combinedAabb = Bounds3f.Union(index.aabb, leafAabb);
                float combinedVolume = combinedAabb.volume;

                // Cost of creating a new parent for this node and the new leaf
                float cost = 2.0f * combinedVolume;

                // Minimum cost of pushing the leaf further down the tree
                float inheritanceCost = 2.0f * (combinedVolume - volume);

                // Cost of descending into child1
                float cost1;
                if (child1.isLeaf)
                    cost1 = Bounds3f.Union(leafAabb, child1.aabb).volume + inheritanceCost;
                else
                {
                    float oldVolume = child1.aabb.volume;
                    float newVolume = Bounds3f.Union(leafAabb, child1.aabb).volume;
                    cost1 = (newVolume - oldVolume) + inheritanceCost;
                }

                // Cost of descending into child2
                float cost2;
                if (child2.isLeaf)
                    cost2 = Bounds3f.Union(leafAabb, child2.aabb).volume + inheritanceCost;
                else
                {
                    float oldVolume = child2.aabb.volume;
                    float newVolume = Bounds3f.Union(leafAabb, child2.aabb).volume;
                    cost2 = newVolume - oldVolume + inheritanceCost;
                }

                // Descend according to the minimum cost.
                if (cost < cost1 && cost < cost2)
                    break;

                // Descend
                index = cost1 < cost2 ? child1 : child2;
            }

            BvhNode sibling = index;

            // Create a new parent.
            BvhNode oldParent = sibling.parent;
            BvhNode newParent = new BvhNode();
            newParent.parent = oldParent;
            Combine(ref newParent.aabb, leafAabb, sibling.aabb);
            newParent.height = sibling.height + 1;

            if (oldParent != null)
            {
                // The sibling was not the root.
                if (oldParent.child1 == sibling)
                    oldParent.child1 = newParent;
                else
                    oldParent.child2 = newParent;

                newParent.child1 = sibling;
                newParent.child2 = leaf;
                sibling.parent = newParent;
                leaf.parent = newParent;
            }
            else
            {
                // The sibling was the root.
                newParent.child1 = sibling;
                newParent.child2 = leaf;
                sibling.parent = newParent;
                leaf.parent = newParent;
                root = newParent;
            }

            // Walk back up the tree fixing heights and AABBs
            index = leaf.parent;
            while (index != null)
            {
                index = Balance(index);

                BvhNode child1 = index.child1;
                BvhNode child2 = index.child2;

                Assert(child1 != null);
                Assert(child2 != null);

                index.height = 1 + Max(child1.height, child2.height);
                Combine(ref index.aabb, child1.aabb, child2.aabb);

                index = index.parent;
            }

            Validate();
        }

        public void RemoveLeaf(BvhNode leaf)
        {
            if (leaf == root)
            {
                root = null;
                return;
            }

            BvhNode parent = leaf.parent;
            BvhNode grandParent = parent.parent;
            BvhNode sibling = parent.child1 == leaf ? parent.child2 : parent.child1;
            if (grandParent != null)
            {
                // Destroy parent and connect sibling to grandParent.
                if (grandParent.child1 == parent)
                    grandParent.child1 = sibling;
                else
                    grandParent.child2 = sibling;

                sibling.parent = grandParent;

                // Adjust ancestor bounds.
                BvhNode index = grandParent;
                while (index != null)
                {
                    index = Balance(index);

                    BvhNode child1 = index.child1;
                    BvhNode child2 = index.child2;

                    Combine(ref index.aabb, child1.aabb, child2.aabb);
                    index.height = 1 + Max(child1.height, child2.height);

                    index = index.parent;
                }
            }
            else
            {
                root = sibling;
                sibling.parent = null;
            }

            Validate();
        }

        BvhNode Balance(BvhNode a)
        {
            Assert(a != null);

            if (a.isLeaf || a.height < 2)
                return a;

            BvhNode b = a.child1;
            BvhNode c = a.child2;

            int balance = c.height - b.height;

            // Rotate C up
            if (balance > 1)
            {
                BvhNode f = c.child1;
                BvhNode g = c.child2;

                // Swap A and C
                c.child1 = a;
                c.parent = a.parent;
                a.parent = c;

                // A's old parent should point to C
                if (c.parent != null)
                {
                    if (c.parent.child1 == a)
                        c.parent.child1 = c;
                    else
                    {
                        Assert(c.parent.child2 == a);
                        c.parent.child2 = c;
                    }
                }
                else
                    root = c;

                // Rotate
                if (f.height > g.height)
                {
                    c.child2 = f;
                    a.child2 = g;
                    g.parent = a;
                    Combine(ref a.aabb, b.aabb, g.aabb);
                    Combine(ref c.aabb, a.aabb, f.aabb);

                    a.height = 1 + Max(b.height, g.height);
                    c.height = 1 + Max(a.height, f.height);
                }
                else
                {
                    c.child2 = g;
                    a.child2 = f;
                    f.parent = a;
                    Combine(ref a.aabb, b.aabb, f.aabb);
                    Combine(ref c.aabb, a.aabb, g.aabb);

                    a.height = 1 + Max(b.height, f.height);
                    c.height = 1 + Max(a.height, g.height);
                }

                return c;
            }

            // Rotate B up
            if (balance < -1)
            {
                BvhNode d = b.child1;
                BvhNode e = b.child2;

                // Swap A and B
                b.child1 = a;
                b.parent = a.parent;
                a.parent = b;

                // A's old parent should point to B
                if (b.parent != null)
                {
                    if (b.parent.child1 == a)
                        b.parent.child1 = b;
                    else
                    {
                        Assert(b.parent.child2 == a);
                        b.parent.child2 = b;
                    }
                }
                else
                    root = b;

                // Rotate
                if (d.height > e.height)
                {
                    b.child2 = d;
                    a.child1 = e;
                    e.parent = a;
                    Combine(ref a.aabb, c.aabb, e.aabb);
                    Combine(ref b.aabb, a.aabb, d.aabb);

                    a.height = 1 + Max(c.height, e.height);
                    b.height = 1 + Max(a.height, d.height);
                }
                else
                {
                    b.child2 = e;
                    a.child1 = d;
                    d.parent = a;
                    Combine(ref a.aabb, c.aabb, d.aabb);
                    Combine(ref b.aabb, a.aabb, e.aabb);

                    a.height = 1 + Max(c.height, d.height);
                    b.height = 1 + Max(a.height, e.height);
                }

                return b;
            }

            return a;
        }
    }
}
