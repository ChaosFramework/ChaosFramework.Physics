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
        public static void Infinity<T>(T value) { }
#else
        [System.Diagnostics.Conditional("DEBUG")]
        public static void Infinity<T>(T value)
            where T : PotentiallyInfiniteVector
        {
            if (value.IsInfinite())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void Infinity(float v)
        {
            if (float.IsInfinity(v))
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void Infinity(Vector2f v)
        {
            if (v.IsInfinite())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void Infinity(Vector3f v)
        {
            if (v.IsInfinite())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void Infinity(Vector4f v)
        {
            if (v.IsInfinite())
                throw new System.Exception();
        }
#endif
    }
}
