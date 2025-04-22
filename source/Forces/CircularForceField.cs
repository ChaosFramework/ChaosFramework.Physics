using ChaosFramework.Math.Vectors;

namespace ChaosFramework.Physics.Forces
{
    public class CircularForceField : ForceField
    {
        public Vector3f position, axis;
        public float force, falloff, massEffect = 1;

        public CircularForceField() { }

        public CircularForceField(Vector3f position, Vector3f axis, float force, float falloff)
        {
            this.position = position;
            this.axis = Vector3f.Normalize(axis);
            this.force = force;
            this.falloff = falloff;
        }
        
        public void ApplyTo(Physical p, float time)
        {
            Vector3f d = p.state.position - position;
            float distance = d.Length();
            if (distance <= float.Epsilon)
                return;

            float effectiveForce = force * (float)System.Math.Pow(p.state.mass, massEffect);
            Vector3f direction = Vector3f.Normalize(Vector3f.Cross(d, axis));
            float strength = effectiveForce / (float)System.Math.Pow(distance, falloff);
            p.state.ApplyForce(direction * strength, p.state.position, time);
        }
    }
}
