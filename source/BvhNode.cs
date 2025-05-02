using ChaosFramework.Math;

namespace ChaosFramework.Physics
{
    public class BvhNode
    {
        public object userData;

        internal Bounds3f aabb;
        internal BvhNode child1, child2;
        internal BvhNode parent;
        internal int height;

        internal bool isLeaf
            => child1 == null;

        internal BvhNode()
        {
            aabb = new Bounds3f();
        }

        public BvhNode(Bounds3f aabb, object userData)
        {
            this.aabb = aabb;
            this.userData = userData;
        }
    }
}
