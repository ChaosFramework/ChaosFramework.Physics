/* This file is based on Erin Catto's implementation of dynamic bounding volume hierarchies in box2D.
 * See
 * - https://github.com/erincatto/box2d
 * - https://box2d.org/files/ErinCatto_DynamicBVH_Full.pdf
 *
 * For license information check the 'License' directory of this repository.
 */

using ChaosFramework.Math;
using System;
using System.Diagnostics;
using static ChaosFramework.Math.Clamping;

namespace ChaosFramework.Physics
{
    partial class DynamicBvh
    {
        const string ASSERT_BVH = "AssertBvh";
        
        [Conditional(ASSERT_BVH)]
        void ValidateStructure(BvhNode index)
        {
            if (index == null)
                return;

            if (index == root)
                Assert(index.parent == null);

            BvhNode node = index;
            BvhNode child1 = node.child1;
            BvhNode child2 = node.child2;

            if (node.isLeaf)
            {
                Assert(child1 == null);
                Assert(child2 == null);
                Assert(node.height == 0);
                return;
            }

            Assert(node.child1.parent == node);
            Assert(node.child2.parent == node);

            ValidateStructure(child1);
            ValidateStructure(child2);
        }

        [Conditional(ASSERT_BVH)]
        void ValidateMetrics(BvhNode index)
        {
            if (index == null)
                return;

            BvhNode node = index;
            BvhNode child1 = node.child1;
            BvhNode child2 = node.child2;

            if (node.isLeaf)
            {
                Assert(child1 == null);
                Assert(child2 == null);
                Assert(node.height == 0);
                return;
            }

            int height1 = child1.height;
            int height2 = child2.height;
            int height = 1 + Max(height1, height2);
            Assert(node.height == height);

            Bounds3f aabb = Bounds3f.Union(child1.aabb, child2.aabb);
            Assert(aabb.low == node.aabb.low);
            Assert(aabb.high == node.aabb.high);

            ValidateMetrics(child1);
            ValidateMetrics(child2);
        }

        [Conditional(ASSERT_BVH)]
        void Validate()
        {
            ValidateStructure(root);
            ValidateMetrics(root);
        }

        [Conditional(ASSERT_BVH)]
        void Assert(bool condition)
        {
            if (!condition)
                throw new Exception();
        }

        [Conditional(ASSERT_BVH)]
        void AssertInsertOnce(BvhNode leaf)
        {
            if (ContainsNode(leaf))
                throw new InvalidOperationException();
        }
    }
}
