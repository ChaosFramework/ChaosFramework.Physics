using ChaosFramework.Shapes.Intersection;

namespace ChaosFramework.Physics
{
    static partial class Collision
    {
        static partial class Epa
        {
            class Edge
            {
                public readonly Support a;
                public readonly Support b;

                public Edge(Support a, Support b)
                {
                    this.a = a;
                    this.b = b;
                }

                public bool IsEqual(Edge compare)
                    => a.IsMinkowskiEqual(compare.b) && b.IsMinkowskiEqual(compare.a);
            }
        }
    }
}
