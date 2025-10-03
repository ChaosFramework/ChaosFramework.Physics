using ChaosFramework.Math.Vectors;
using ChaosFramework.Shapes.Intersection;

namespace ChaosFramework.Physics
{
    static partial class Collision
    {
        static partial class Epa
        {
            class Face
            {
                public readonly float distance;
                public readonly Vector3f normal;
                public readonly Support a;
                public readonly Support b;
                public readonly Support c;

                public Face(Edge e, Support c)
                    : this(e.a, e.b, c)
                { }

                public Face(Support a, Support b, Support c)
                {
                    this.a = a;
                    this.b = b;
                    this.c = c;
                    normal = Vector3f.Normalize(Vector3f.Cross(this.b.minkowski - this.a.minkowski, this.c.minkowski - this.a.minkowski));
                    distance = Vector3f.Dot(a.minkowski, normal);
                }

                public bool IsVisibleFromPoint(Support point)
                    => Vector3f.Dot(normal, point.minkowski) > distance;
            }
        }
    }
}
