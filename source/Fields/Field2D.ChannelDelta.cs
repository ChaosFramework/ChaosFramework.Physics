using ChaosFramework.Math.Vectors;

namespace ChaosFramework.Physics.Fields
{
    partial class Field2D
    {
        protected struct ChannelDelta
        {
            public readonly Vector2f position;
            public readonly float value;
            public readonly float fallOff;
            public readonly float radius;
            public readonly DeltaOp operation;

            public ChannelDelta(Vector2f position, float value, float radius, float fallOff, DeltaOp operation)
            {
                this.position = position;
                this.value = value;
                this.radius = radius;
                this.fallOff = fallOff;
                this.operation = operation;
            }
        }
    }
}
