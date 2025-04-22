namespace ChaosFramework.Physics.Forces
{
    public interface ForceField
    {
        void ApplyTo(Physical p, float time);
    }
}
