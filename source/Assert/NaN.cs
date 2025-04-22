#if DEBUG
using ChaosFramework.Math;
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
        public static void NaN<T>(T value) { }
#else
        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaN<T>(T value)
            where T : PotentiallyNaNVector
        {
            if (value.IsNaN())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaN(float f)
        {
            if (float.IsNaN(f))
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaN(Vector2f v)
        {
            if (v.IsNaN())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaN(Vector3f v)
        {
            if (v.IsNaN())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaN(Vector4f v)
        {
            if (v.IsNaN())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaN(Matrix m)
        {
            if (m.IsNaN())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaN(Plane p)
        {
            if (p.IsNaN())
                throw new System.Exception();
        }

        [System.Diagnostics.Conditional("DEBUG")]
        public static void NaN(Ray ray)
        {
            if (ray.IsNaN())
                throw new System.Exception();
        }
#endif
    }
}
