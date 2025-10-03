using ChaosFramework.Shapes.Convex;
using ChaosFramework.Math.Vectors;

namespace ChaosFramework.Physics
{
    public class RayImpact
    {
        internal static int Compare(RayImpact a, RayImpact b)
            => a.distanceSq.CompareTo(b.distanceSq);

        public readonly Vector3f origin;
        public readonly Vector3f direction;
        public Vector3f impact;
        public Physical physical;
        public Shape shape;
        public float distanceSq;

        public RayImpact(Vector3f origin, Vector3f direction)
        {
            this.origin = origin;
            this.direction = direction;
        }
    }
}
