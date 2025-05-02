using ChaosFramework.Math.Vectors;

namespace ChaosFramework.Physics
{
    static partial class Collision
    {
        public class Support
        {
            public readonly Vector3f a, b;
            public readonly Vector3f minkowski;

            public Support(Vector3f a, Vector3f b)
            {
                this.a = a;
                this.b = b;
                minkowski = a - b;
            }

            public bool IsEqual(Support compare)
                => minkowski == compare.minkowski;
        }
    }
}
