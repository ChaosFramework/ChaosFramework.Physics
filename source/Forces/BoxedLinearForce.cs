using ChaosFramework.Math.Vectors;
using ChaosFramework.Shapes.Intersection;

namespace ChaosFramework.Physics.Forces
{
    public class BoxedLinearForce : LinearForce
    {
        Vector3f low, high;

        public BoxedLinearForce(Vector3f force, Vector3f low, Vector3f high)
            : base(force)
        {
            this.low = low;
            this.high = high;
        }

        public override void ApplyTo(Physical p, float time)
        {
            if (Bounds.CheckBoundingBoxIntersection(p.boundingBoxLow, p.boundingBoxHigh, low, high))
                base.ApplyTo(p, time);
        }
    }
}
