using ChaosFramework.Collections;
using ChaosFramework.Math.Vectors;
using ChaosFramework.Shapes.Convex;

namespace ChaosFramework.Physics
{
    public class CollisionData
    {
        public class ShapeCollisionData
        {
            public readonly Shape s1, s2;
            public readonly Vector3f v1, v2;

            /// <summary> The normalized direction of the penetration of s1 in s2. </summary>
            public readonly Vector3f penetrationNormal;

            public float remainingPenetration;
            public float processedPenetration;

            /// <summary> The estimated point of impact in absolute world coordinates. </summary>
            public readonly Vector3f absolutePointOfImpact;
            public readonly Vector3f impact1, impact2;

            public ShapeCollisionData(
                Shape s1,
                Shape s2,
                Vector3f v1,
                Vector3f v2,
                Vector3f penetration,
                Vector3f impact1,
                Vector3f impact2)
            {
                this.s1 = s1;
                this.s2 = s2;
                this.v1 = v1;
                this.v2 = v2;
                this.impact1 = impact1;
                this.impact2 = impact2;

                remainingPenetration = penetration.Length();
                penetrationNormal = penetration * (1 / remainingPenetration);
                absolutePointOfImpact = (impact1 + impact2) * 0.5f;
                processedPenetration = 0;
            }
        }

        public readonly Physical p1, p2;
        public readonly AdvancedLinkedList<ShapeCollisionData> data;

        public CollisionData(Physical p1, Physical p2)
        {
            this.p1 = p1;
            this.p2 = p2;
            data = new AdvancedLinkedList<ShapeCollisionData>();
        }
    }
}
