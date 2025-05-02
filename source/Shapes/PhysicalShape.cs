using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using static ChaosFramework.Math.Clamping;

namespace ChaosFramework.Physics.Shapes
{
    public abstract class PhysicalShape
    {
        public delegate Vector3f CustomEpaSupport(Vector3f searchDirection);

        public float bounce = 0.5f;
        public string name;
        public CustomEpaSupport epaSupportFunction = null;

        Matrix preTransform = Matrix.NAN;

        public PhysicalShape previousFrameShape { get; internal set; }

        public Vector3f boundingBoxLow { get; protected set; }
        public Vector3f boundingBoxHigh { get; protected set; }

        public Vector3f interFrameBoundingBoxLow { get; private set; }
        public Vector3f interFrameBoundingBoxHigh { get; private set; }

        public float hullRadius { get; internal set; }

        public Vector3f transformedHullCenter { get; internal set; }

        public PhysicalShape()
        {
            previousFrameShape = this;
            epaSupportFunction = Support;
        }

        public abstract Vector3f Support(Vector3f searchDirection);

        public Vector3f InterFrameSupport(Vector3f searchDirection)
        {
            if (previousFrameShape == null)
                return Support(searchDirection);

            Vector3f a = Support(searchDirection);
            Vector3f b = previousFrameShape.Support(searchDirection);
            return Vector3f.Dot(a, searchDirection) > Vector3f.Dot(b, searchDirection) ? a : b;
        }

        internal float GetFarthestDistSq(Vector3f center)
        {
            float farthestDist = (transformedHullCenter - center).Length() + hullRadius;
            return farthestDist * farthestDist;
        }

        public void Update(Matrix transform, bool forceUpdate = false)
        {
            if (!forceUpdate && transform == preTransform)
                return;

            ForceUpdate(transform);

            if (previousFrameShape != null)
            {
                interFrameBoundingBoxLow = Min(boundingBoxLow, previousFrameShape.boundingBoxLow);
                interFrameBoundingBoxHigh = Max(boundingBoxHigh, previousFrameShape.boundingBoxHigh);
            }
            else
            {
                interFrameBoundingBoxLow = boundingBoxLow;
                interFrameBoundingBoxHigh = boundingBoxHigh;
            }

            preTransform = transform;
        }

        public void ApplyTransform(Matrix transform)
        {
            ForceUpdate(transform);
            ApplyTransformedData();
            ForceUpdate(Matrix.IDENTITY);
        }

        protected abstract void ApplyTransformedData();

        public abstract PhysicalShape CloneTypeless();

        public abstract PhysicalShape CloneTypelessTransformed();

        internal abstract void ForceUpdate(Matrix transform);

        public abstract Vector3f GetRandomPositionInShape();

        public abstract Vector3f GetRandomPositionOnShape();

        /// <summary>
        ///     If the line starting at <paramref name="origin"/> in <param name="direction"/> hits the surface of this shape,
        ///     this will return at least one intersection point.
        ///     <para>
        ///         If multiple intersection points are returned, they are ordered by signed distance along <paramref name="direction"/>.
        ///     </para>
        /// </summary>
        /// <remarks> Currently, this makes no guarantee that all intersection points are returned. </remarks>
        public abstract Vector3f[] CheckLine(Vector3f origin, Vector3f direction);
    }

    public abstract class PhysicalShape<T>
        : PhysicalShape
        where T : PhysicalShape<T>
    {
        public sealed override PhysicalShape CloneTypeless() 
            => Clone();

        public sealed override PhysicalShape CloneTypelessTransformed() 
            => CloneTransformed();

        protected abstract T CloneSpecialized();
        protected abstract T CloneSpecializedTransformed();

        public T Clone()
        {
            T clone = CloneSpecialized();
            clone.name = name;
            return clone;
        }

        public T CloneTransformed()
        {
            T clone = CloneSpecializedTransformed();
            clone.name = name;
            return clone;
        }
    }
}
