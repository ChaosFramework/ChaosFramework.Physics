using ChaosFramework.Math.Vectors;

namespace ChaosFramework.Physics.Forces
{
    public class LinearForce : ForceField
    {
        public Vector3f force;
        public float massEffect = 1;

        public LinearForce() { }

        public LinearForce(float x, float y, float z)
            : this(new Vector3f(x, y, z))
        { }

        public LinearForce(Vector3f force)
        {
            this.force = force;
        }

        public virtual void ApplyTo(Physical p, float time)
        {
            Vector3f massEffectValue = force * (float)System.Math.Pow(p.state.mass, massEffect);
            p.state.ApplyForce(massEffectValue, p.state.position, time);
        }
    }
}
