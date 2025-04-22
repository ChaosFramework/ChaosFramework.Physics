using ChaosFramework.Math.Vectors;

namespace ChaosFramework.Physics.Forces
{
    public class RadialForceField : ForceField
    {
        public Vector3f position;
        public float force, falloff, massEffect = 1;

        protected float centerSize = 0;

        private Vector3f _lockedAxis = Vector3f.EMPTY;
        public Vector3f lockedAxis
        {
            get { return _lockedAxis; }
            set { _lockedAxis = Vector3f.Normalize(value); }
        }

        public RadialForceField() { }

        public RadialForceField(Vector3f position, float force, float falloff = 1)
        {
            this.force = force;
            this.position = position;
            this.falloff = falloff;
        }

        public void ApplyTo(Physical p, float time)
        {
            float effectiveForce = force * (float)System.Math.Pow(p.state.mass, massEffect);
            Vector3f relativePosition = position - p.state.position;
            relativePosition -= Vector3f.Dot(lockedAxis, relativePosition) * lockedAxis;
            float distance = relativePosition.Length();
            if (distance <= float.Epsilon)
                return;

            Vector3f direction = relativePosition * (1 / (distance + centerSize));
            float strength = effectiveForce / (float)System.Math.Pow(distance, falloff);
            p.state.ApplyForce(strength * direction, p.state.position, time);
        }
    }
}
