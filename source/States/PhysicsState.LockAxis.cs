using ChaosFramework.Math.Vectors;

namespace ChaosFramework.Physics.States
{
    partial class PhysicsState
    {
        internal class LockAxis
        {
            internal readonly Vector3f axis;
            internal readonly CollisionData.ShapeCollisionData cause;
            internal int frame;

            public LockAxis(Vector3f axis, CollisionData.ShapeCollisionData cause)
            {
                this.axis = axis;
                this.cause = cause;
                frame = 0;
            }

            public int GetFrameThenIncrement() 
                => frame++;
        }
    }
}
