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

        /// <summary>
        ///     Attempts to calculate the launch vector necessary to hit
        ///     the given <paramref name="relativeTargetPosition"/>.
        /// </summary>
        /// <param name="relativeTargetPosition"> The target to hit. </param>
        /// <param name="maxLaunchSpeed"> Maximum length of the launch vector. </param>
        /// <param name="gravity"> Forces that affect the launch angle. </param>
        /// <param name="mode"> What kind of trajectory to aim for if multiple angles are possible. </param>
        /// <param name="result">
        ///     The exact launch vector to hit the target, if successful;
        ///     A best guess otherwise.
        /// </param>
        /// <returns>
        ///     <see langword="true"/> if the calculation was successful;
        ///     <see langword="false"/> if the target can't be hit with the provided configuration.
        /// </returns>
        public static bool GetLaunchVelocity(
            Vector3f relativeTargetPosition,
            float maxLaunchSpeed,
            Vector3f gravity,
            AimMode mode,
            out Vector3f result
            )
        {
            bool success = true;

            float gSquared = gravity.LengthSq();
            float q = maxLaunchSpeed * maxLaunchSpeed + Vector3f.Dot(relativeTargetPosition, gravity);

            float discriminant = q * q - gSquared * relativeTargetPosition.LengthSq();
            if (discriminant < 0)
            {
                success = false;
                discriminant = 0;
            }

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

            result = relativeTargetPosition * (1 / t) - gravity * (t * 0.5f);
            return success;
        }

        public static float GetAngle(Vector3f planeNormal, Vector3f direction)
        {
            float dot = Vector3f.Dot(planeNormal, direction);
            return (float)System.Math.Atan(dot / (direction - planeNormal * dot).Length());
        }
    }
}
