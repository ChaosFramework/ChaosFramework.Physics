using ChaosFramework.Collections;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using ChaosUtil.Primitives;
using System.Linq;
using static ChaosFramework.Math.Clamping;
using static ChaosFramework.Math.Exponentials;

namespace ChaosFramework.Physics.Shapes
{
    public class MeshShape : PhysicalShape<MeshShape>
    {
        static readonly Vector3f DOWN = new Vector3f(0, -1, 0);
        
        static bool LineHitsTriangle(Vector3f origin, Vector3f direction, Vector3f a, Vector3f b, Vector3f c, out Vector3f projection)
        {
            Vector3f normal = Vector3f.Normalize(Vector3f.Cross(b - a, c - a));
            float d = Vector3f.Dot(origin, normal) - Vector3f.Dot(a, normal);
            projection = origin - direction * (d / Vector3f.Dot(direction, normal));

            Vector3f prismA = Vector3f.Cross(b - a, normal);
            Vector3f prismB = Vector3f.Cross(c - b, normal);
            Vector3f prismC = Vector3f.Cross(a - c, normal);

            return Vector3f.Dot(projection, prismA) - Vector3f.Dot(a, prismA) <= 0
                && Vector3f.Dot(projection, prismB) - Vector3f.Dot(b, prismB) <= 0
                && Vector3f.Dot(projection, prismC) - Vector3f.Dot(c, prismC) <= 0;
        }

        public Vector3f[] baseVerts;
        public Vector3f[] transformedVerts;

        MeshShape() { }

        public MeshShape(Vector3f[] baseVerts, bool optimize = false)
        {
            this.baseVerts = baseVerts;
            if (optimize)
                Optimize();

            Update(Matrix.IDENTITY);
        }

        public void Optimize()
        {
            AdvancedLinkedList<Vector3f> newVerts = new AdvancedLinkedList<Vector3f>();
            newVerts.AddUnique(baseVerts);

            foreach (Vector3f pos in newVerts)
            {
                MeshShape s1 = new MeshShape(newVerts.Where(v => v != pos).ToArray());
                PointShape s2 = new PointShape(pos);

                if (Collision.Gjk.CheckIntersection(s1, s2) != null)
                    newVerts.RemoveCurrent();
            }

            baseVerts = newVerts.ToArray();
            Update(Matrix.IDENTITY);
        }

        public override Vector3f Support(Vector3f searchDirection)
            => transformedVerts?.Max(v => Vector3f.Dot(v, searchDirection)) ?? DOWN;

        internal override void ForceUpdate(Matrix transform)
        {
            transformedVerts = Vector3f.TransformCoordinate(baseVerts, transform);
            transformedHullCenter = Vector3f.EMPTY;
            foreach (Vector3f pos in transformedVerts)
                transformedHullCenter += pos;

            transformedHullCenter *= 1f / transformedVerts.Length;

            float radSq = 0;
            boundingBoxHigh = transformedHullCenter;
            boundingBoxLow = transformedHullCenter;
            foreach (Vector3f pos in transformedVerts)
            {
                float posSq = (pos - transformedHullCenter).LengthSq();
                if (posSq > radSq)
                    radSq = posSq;

                boundingBoxHigh = Max(boundingBoxHigh, pos);
                boundingBoxLow = Min(boundingBoxLow, pos);
            }

            hullRadius = Sqrt(radSq);
        }

        public override Vector3f GetRandomPositionInShape()
        {
            const uint MAX_TRIES = 100;

            SphereShape shape = new SphereShape(Vector3f.EMPTY, 0);
            for (int i = 0; i < MAX_TRIES; i++)
            {
                Vector3f rndPos = Random.instance.RndVector3(Random.instance.Rnd(hullRadius)) + transformedHullCenter;
                shape.transformedHullCenter = rndPos;
                if (Collision.Gjk.CheckIntersection(shape, this) != null)
                    return rndPos;
            }

            return transformedHullCenter;
        }

        public override Vector3f GetRandomPositionOnShape()
        {
            const int MAX_TRIES = 3;

            Vector3f randomDirection = Random.instance.RndVector3();
            for (int i = 0; i < MAX_TRIES; i++, randomDirection = Random.instance.RndVector3())
            {
                Vector3f[] result = CheckLine(transformedHullCenter, randomDirection);
                if (result.Length > 0)
                    return result[0];
            }

            return Support(randomDirection);
        }

        protected override MeshShape CloneSpecializedTransformed()
        {
            MeshShape clone = new MeshShape();
            clone.transformedVerts = (Vector3f[])transformedVerts.Clone();
            clone.hullRadius = hullRadius;
            clone.boundingBoxHigh = boundingBoxHigh;
            clone.boundingBoxLow = boundingBoxLow;
            clone.transformedHullCenter = transformedHullCenter;
            return clone;
        }

        protected override MeshShape CloneSpecialized()
        {
            MeshShape clone = CloneTransformed();
            clone.baseVerts = (Vector3f[])baseVerts.Clone();
            return clone;
        }

        protected override void ApplyTransformedData()
            => baseVerts = (Vector3f[])transformedVerts.Clone();

        public override Vector3f[] CheckLine(Vector3f origin, Vector3f direction)
        {
            // TODO: this is entirely broken. Fix it.
            direction.Normalize();

            Vector3f support0 = Support(-direction);
            Vector3f support0Projection = origin + direction * Vector3f.Dot(support0 - origin, direction);
            Vector3f support1 = Support(support0Projection - support0);
            if (support1.IsNaN()
             || support0.IsNaN()
             || Vector3f.Dot(support1 - support0Projection, support0 - support0Projection) >= 0
             )
                return Array<Vector3f>.empty;

            Vector3f support2Direction = Vector3f.Cross(direction, support1 - support0);
            if (Vector3f.Dot(support2Direction, support0Projection) < Vector3f.Dot(support2Direction, support0))
                support2Direction *= -1;

            Vector3f support2 = Support(support2Direction);
            if (support2.IsNaN() || Vector3f.Dot(support2 - origin, support2Direction) <= 0)
                return Array<Vector3f>.empty;

            const uint MAX_TRIES = 100;
            int numIterations = 0;
            while (true)
            {
                Vector3f normal = Vector3f.Normalize(Vector3f.Cross(support2 - support0, support1 - support0));

                if (Vector3f.Dot(normal, direction) > 0)
                {
                    normal *= -1;
                    Vector3f tmp = support1;
                    support1 = support2;
                    support2 = tmp;
                }

                Vector3f newSupport = Support(normal);
                if (numIterations++ > MAX_TRIES
                 || newSupport == support0
                 || newSupport == support1
                 || newSupport == support2
                 || Vector3f.Dot(newSupport, normal) <= Vector3f.Dot(support0, normal) + 0.0001f)
                {
                    // found triangle. compute point of impact.
                    float d = Vector3f.Dot(origin, normal) - Vector3f.Dot(support0, normal);
                    Vector3f projected = origin - (d / Vector3f.Dot(direction, normal)) * direction;
                    if (numIterations >= MAX_TRIES)
                        return Array<Vector3f>.empty;

                    // TODO: Figure out why this can happen
                    if (Collision.Gjk.CheckIntersection(new SphereShape(projected, 0.02f), this) == null)
                        return Array<Vector3f>.empty;

                    return new Vector3f[] { projected };
                }
                else
                {
                    Vector3f proj;
                    if (LineHitsTriangle(origin, direction, support0, support1, newSupport, out proj))
                        support2 = newSupport;
                    else if (LineHitsTriangle(origin, direction, support1, support2, newSupport, out proj))
                        support0 = newSupport;
                    else
                        support1 = newSupport;
                }
            }
        }
    }
}
