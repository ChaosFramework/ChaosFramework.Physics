#if DEBUG
using ChaosFramework.Math.Vectors;
using ChaosFramework.Math.Vectors.Operations;
#endif

namespace ChaosFramework.Physics
{
    internal static partial class Assert
    {
#if !DEBUG
        /// <summary>
        ///     This method only exists to make this signature legal for non-debug builds
        ///     and no invocations of this method shall ever be compiled into any code.
        /// </summary>
        /// <typeparam name="T"> Never to be supplied. </typeparam>
        /// <param name="value"> Never to be supplied. </param>
        [System.Diagnostics.Conditional("__F_A_L_S_E__")]
        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
        public static void NaNOrInfinity<T>(T value) { }
#else
        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaNOrInfinity<T>(T value)
            where T : PotentiallyNaNVector
                    , PotentiallyInfiniteVector
        {
            NaN(value);
            Infinity(value);
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaNOrInfinity(float f)
        {
            NaN(f);
            Infinity(f);
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaNOrInfinity(Vector2f v)
        {
            NaN(v);
            Infinity(v);
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaNOrInfinity(Vector3f v)
        {
            NaN(v);
            Infinity(v);
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaNOrInfinity(Vector4f v)
        {
            NaN(v);
            Infinity(v);
        }
#endif
    }
}
