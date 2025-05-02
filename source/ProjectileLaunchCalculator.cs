using ChaosFramework.Math.Vectors;
using static ChaosFramework.Math.Exponentials;

namespace ChaosFramework.Physics
{
    public static class ProjectileLaunchCalculator
    {
        public enum AimMode
        {
            LowestSpeed,
            MaxTime,
            MinTime
        }

        public static Vector3f GetLaunchVelocity(Vector3f relativeTargetPosition, float maxLaunchSpeed, Vector3f gravity, AimMode mode)
        {
            float gSquared = gravity.LengthSq();
            float q = maxLaunchSpeed * maxLaunchSpeed + Vector3f.Dot(relativeTargetPosition, gravity);

            float discriminant = q * q - gSquared * relativeTargetPosition.LengthSq();
            if (discriminant < 0)
                return Vector3f.NAN;

            discriminant = Sqrt(discriminant);
            float t = 0;
            switch (mode)
            {
                case AimMode.MaxTime:
                    {
                        t = Sqrt((q + discriminant) * 2 / gSquared);
                        break;
                    }
                case AimMode.MinTime:
                    {
                        t = Sqrt((q - discriminant) * 2 / gSquared);
                        break;
                    }
                case AimMode.LowestSpeed:
                    {
                        t = Sqrt(Sqrt(relativeTargetPosition.LengthSq() * 4 / gSquared));
                        break;
                    }
            }

            return relativeTargetPosition * (1 / t) - gravity * (t * 0.5f);
        }

        public static float GetAngle(Vector3f planeNormal, Vector3f direction)
        {
            float dot = Vector3f.Dot(planeNormal, direction);
            return (float)System.Math.Atan(dot / (direction - planeNormal * dot).Length());
        }
    }
}
