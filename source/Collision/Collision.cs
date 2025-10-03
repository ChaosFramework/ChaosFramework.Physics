using ChaosFramework.Math.Vectors;
using static ChaosFramework.Math.Exponentials;

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
    }
}
