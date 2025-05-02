using ChaosFramework.Math.Vectors;
using ChaosFramework.Physics.Shapes;
using static ChaosFramework.Math.Exponentials;
using static ChaosFramework.Math.Signs;

namespace ChaosFramework.Physics
{
    public static partial class Collision
    {
        const float CLOSEST_DISTANCE_THRESHOLD = 0.0001f;

        // TODO: Make these configuration parameters non-static somehow.
        internal static float interFrameCollisionVelocityThresholdSq = 100;
        internal static float interFrameCollisionVelocityThreshold = 10;

        internal static bool useInterFrameCollisionDetection { get; private set; }

        private static Vector3f _relativeVelocity;
        internal static Vector3f relativeVelocity
        {
            get { return _relativeVelocity; }
            set
            {
                _relativeVelocity = value;
                useInterFrameCollisionDetection = value.LengthSq() > interFrameCollisionVelocityThresholdSq;
                interFrameCollisionVelocityThreshold = Sqrt(interFrameCollisionVelocityThresholdSq);
            }
        }

        public static bool CheckHullIntersection(PhysicalShape s1, PhysicalShape s2)
        {
            float hullSum = s1.hullRadius + s2.hullRadius;
            return hullSum * hullSum >= (s1.transformedHullCenter - s2.transformedHullCenter).LengthSq();
        }

        public static bool CheckBoundingBoxIntersection(Vector3f low1, Vector3f high1, Vector3f low2, Vector3f high2)
        {
            Vector3f centerA = (low1 + high1) * 0.5f;
            Vector3f sizeA = (high1 - low1) * 0.5f;
            Vector3f centerB = (low2 + high2) * 0.5f;
            Vector3f sizeB = (high2 - low2) * 0.5f;
            return (Abs(centerA.x - centerB.x) <= sizeA.x + sizeB.x
                 && Abs(centerA.y - centerB.y) <= sizeA.y + sizeB.y
                 && Abs(centerA.z - centerB.z) <= sizeA.z + sizeB.z
                 );
        }

        public static bool CheckBoundingBoxIntersection(PhysicalShape s1, PhysicalShape s2)
            => CheckBoundingBoxIntersection(s1.boundingBoxLow, s1.boundingBoxHigh, s2.boundingBoxLow, s2.boundingBoxHigh);
    }
}
