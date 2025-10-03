using ChaosFramework.Collections;
using ChaosFramework.Math.Vectors;
using ChaosFramework.Shapes.Convex;
using ChaosFramework.Shapes.Intersection;
using System;
using static ChaosFramework.Math.Clamping;

namespace ChaosFramework.Physics
{
    static partial class Collision
    {
        public static partial class Epa
        {
            const short MAX_ITERATIONS = 100;

            public static Vector3f GetPenetrationVector(
                Shape s1,
                Shape s2,
                LinkedList<Support> simplex,
                out Vector3f pointOfImpactA,
                out Vector3f pointOfImpactB
                )
            {
                if (simplex.length != 4)
                    throw new ArgumentException($"{nameof(simplex)} needs to have 4 vertices.");

                LinkedList<Edge> edges = new LinkedList<Edge>();
                LinkedList<Face> faces = new LinkedList<Face>(
                    new Face(simplex[0], simplex[1], simplex[2]),
                    new Face(simplex[0], simplex[2], simplex[3]),
                    new Face(simplex[0], simplex[3], simplex[1]),
                    new Face(simplex[1], simplex[3], simplex[2])
                    );

                for (int i = 0; i < MAX_ITERATIONS; i++)
                {
                    Face closest = FindClosestPlane(faces);
                    if (closest == null)
                        return pointOfImpactA = pointOfImpactB = Vector3f.EMPTY;

                    Support newPoint = new Support(
                        s1.epaSupportFunction(closest.normal),
                        s2.epaSupportFunction(-closest.normal)
                        );

                    float newPointDistance = Vector3f.Dot(newPoint.minkowski, closest.normal);
                    if (newPointDistance - closest.distance < CLOSEST_DISTANCE_THRESHOLD)
                    {
                        // valid termination
                        float u, v, w;
                        Vector3f penetration = closest.distance * closest.normal;
                        Barycentric(penetration, closest.a.minkowski, closest.b.minkowski, closest.c.minkowski, out u, out v, out w);
                        pointOfImpactA = closest.a.a * u + closest.b.a * v + closest.c.a * w;
                        pointOfImpactB = closest.a.b * u + closest.b.b * v + closest.c.b * w;
                        return penetration;
                    }

                    // remove internal edges and faces
                    int visibleFaces = 0;
                    AdvancedLinkedList<Edge> borderEdges = new AdvancedLinkedList<Edge>();
                    foreach (Face face in faces)
                        if (face.IsVisibleFromPoint(newPoint))
                        {
                            visibleFaces++;
                            faces.Remove(face);
                            Edge[] edgesToRemove = new Edge[] {
                                new Edge(face.a, face.b),
                                new Edge(face.b, face.c),
                                new Edge(face.c, face.a),
                            };

                            foreach (Edge edge in edgesToRemove)
                            {
                                foreach (Edge compare in borderEdges)
                                    if (compare.IsEqual(edge))
                                    {
                                        borderEdges.RemoveCurrent();
                                        goto continue2;
                                    }
                                borderEdges.Add(edge);

                            continue2:;
                            }
                        }

                    // add new faces
                    foreach (Edge edge in borderEdges)
                        faces.Add(new Face(edge, newPoint));
                }

                System.Diagnostics.Debug.WriteLine("EPA failed hardcore...");
                return pointOfImpactA = pointOfImpactB = Vector3f.EMPTY;
            }

            static Face FindClosestWeightedPlane(LinkedList<Face> faces)
            {
                Face closestFace = null;
                float closestWeightedFaceValue = float.MaxValue;
                foreach (Face face in faces)
                {
                    float invScale = Max(0.001f, Vector3f.Dot(relativeVelocity, face.normal) + interFrameCollisionVelocityThreshold * 100);
                    float compare = face.distance / invScale;
                    if (compare < closestWeightedFaceValue)
                    {
                        closestFace = face;
                        closestWeightedFaceValue = compare;
                    }
                }

                return closestFace;
            }

            static Face FindClosestUnweightedPlane(LinkedList<Face> faces)
            {
                Face closestFace = null;
                float closestWeightedFaceValue = float.MaxValue;
                foreach (Face face in faces)
                    if (face.distance < closestWeightedFaceValue)
                    {
                        closestFace = face;
                        closestWeightedFaceValue = face.distance;
                    }

                return closestFace;
            }

            static Face FindClosestPlane(LinkedList<Face> faces)
                => useInterFrameCollisionDetection
                 ? FindClosestWeightedPlane(faces)
                 : FindClosestUnweightedPlane(faces);

            static void Barycentric(Vector3f p, Vector3f a, Vector3f b, Vector3f c, out float u, out float v, out float w)
            {
                Vector3f ab = b - a;
                Vector3f ac = c - a;
                Vector3f ap = p - a;

                float abab = Vector3f.Dot(ab, ab);
                float abac = Vector3f.Dot(ab, ac);
                float acac = Vector3f.Dot(ac, ac);
                float apab = Vector3f.Dot(ap, ab);
                float apac = Vector3f.Dot(ap, ac);

                float denominator = abab * acac - abac * abac;
                v = (acac * apab - abac * apac) / denominator;
                w = (abab * apac - abac * apab) / denominator;
                u = 1 - v - w;
            }
        }
    }
}
