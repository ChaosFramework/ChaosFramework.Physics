using ChaosFramework.Collections;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using SysCol = System.Collections.Generic;
using static ChaosFramework.Math.Clamping;
using static ChaosFramework.Math.Exponentials;

namespace ChaosFramework.Physics
{
    using Shapes;
    using States;

    public class Physical
    {
        public delegate bool ValidateCollision(Physical collisionPartner, CollisionData data);
        public delegate SysCol.IEnumerable<PhysicalShape> GetRelevantShapes(Physical collisionPartner);

        public static bool ValidateCollisionAlways(Physical collisionPartner, CollisionData data)
            => true;

        static int CompareShapeCollisions(CollisionData.ShapeCollisionData col1, CollisionData.ShapeCollisionData col2)
            => col1.remainingPenetration > col2.remainingPenetration ? -1 : 1;

        /// <summary>
        ///     Determines whether detected collisions will be resolved.
        ///     Collisions between non-solid physicals will not be detected.
        /// </summary>
        public readonly bool isSolid;

        public readonly object creator;
        public readonly LinkedList<CollisionData> currentCollisions = new LinkedList<CollisionData>();
        public readonly AdvancedLinkedList<PhysicalShape> shapes = new AdvancedLinkedList<PhysicalShape>();

        public ValidateCollision validateCollision = ValidateCollisionAlways;
        public GetRelevantShapes getRelevantShapes;

        /// <summary>
        ///     Determines whether this object will be moved via collision resolution.
        ///     These are also used as a starting point for collision detection.
        /// </summary>
        public bool isStatic = false;

        /// <summary>
        ///     If <see cref="isStatic"/> is <see langword="true"/>,
        ///     this flag determines if this object will check other static physicals for intersections.
        /// </summary>
        public bool checkStaticCollisions = true;

        public PhysicsState state;

        internal Vector3f interFrameVelocity;
        internal Matrix lastOldShapeUpdateTransform = Matrix.NAN;
        internal bool isInCurrentCollisionLayer = false;

        public float hullRadius { get; private set; }
        public Vector3f boundingBoxHigh { get; private set; }
        public Vector3f boundingBoxLow { get; private set; }

        internal Matrix appliedTransform { get; private set; }

        public Physical(object creator, bool isSolid = true)
            : this(new PhysicsState(), creator, isSolid)
        { }

        public Physical(PhysicsState state, object creator, bool isSolid = true)
        {
            this.state = state;
            this.creator = creator;
            this.isSolid = isSolid;
            getRelevantShapes = GetAllShapes;
        }

        public LinkedList<PhysicalShape> GetAllShapes(Physical collisionPartner)
            => shapes;

        public void AddShape(params PhysicalShape[] shapes)
            => this.shapes.Add(shapes);

        public void UpdateShapes(bool forceUpdate = false)
        {
            appliedTransform = state.GetTransform();

            float farthestDistanceSq = 0;
            boundingBoxHigh = Vector3f.MIN_VALUE;
            boundingBoxLow = Vector3f.MAX_VALUE;
            foreach (PhysicalShape shape in shapes)
            {
                shape.Update(appliedTransform, forceUpdate);
                boundingBoxHigh = Max(boundingBoxHigh, shape.boundingBoxHigh);
                boundingBoxLow = Min(boundingBoxLow, shape.boundingBoxLow);
                farthestDistanceSq = Max(farthestDistanceSq, shape.GetFarthestDistSq(state.position));
            }

            hullRadius = Sqrt(farthestDistanceSq);
        }

        public Vector3f Support(Vector3f searchDirection)
        {
            if (shapes.empty)
                return new Vector3f(0, -1, 0);

            Vector3f max = Vector3f.EMPTY;
            Vector3f v;

            float maxDist = float.MinValue;
            float dot;

            foreach (PhysicalShape shape in shapes)
                if ((dot = Vector3f.Dot((v = shape.Support(searchDirection)), searchDirection)) > maxDist)
                {
                    max = v;
                    maxDist = dot;
                }

            return max;
        }

        public CollisionData CheckCollision(Physical collisionPartner)
        {
            CollisionData data = new CollisionData(this, collisionPartner);

            Collision.relativeVelocity = interFrameVelocity - collisionPartner.interFrameVelocity;
            if (!Collision.useInterFrameCollisionDetection)
            {
                float hullSum = hullRadius + collisionPartner.hullRadius;
                if ((state.position - collisionPartner.state.position).LengthSq() > hullSum * hullSum)
                    return data;
            }

            SysCol.IEnumerable<PhysicalShape> myShapes = getRelevantShapes(collisionPartner);
            if (myShapes == null || myShapes.Empty())
                return data;

            SysCol.IEnumerable<PhysicalShape> otherShapes = collisionPartner.getRelevantShapes(this);
            if (otherShapes == null || otherShapes.Empty())
                return data;

            foreach (PhysicalShape otherShape in otherShapes)
                foreach (PhysicalShape myShape in myShapes)
                {
                    LinkedList<Collision.Support> simplex = Collision.Gjk.CheckIntersection(myShape, otherShape);
                    if (simplex != null)
                    {
                        Vector3f pointOfImpact1, pointOfImpact2;
                        Vector3f penetration = Collision.Epa.GetPenetrationVector(
                            myShape,
                            otherShape,
                            simplex,
                            out pointOfImpact1,
                            out pointOfImpact2
                            );

                        CollisionData.ShapeCollisionData newCollision = new CollisionData.ShapeCollisionData(
                            myShape,
                            otherShape,
                            state.GetVelocityAt(pointOfImpact1, myShape),
                            collisionPartner.state.GetVelocityAt(pointOfImpact2, otherShape),
                            penetration,
                            pointOfImpact1,
                            pointOfImpact2
                            );

                        if (newCollision.remainingPenetration > 0 && !newCollision.absolutePointOfImpact.IsNaN())
                            data.data.AddSorted(newCollision, CompareShapeCollisions);
                    }
                }

            return data;
        }

        public void Teleport(Vector3f targetPosition)
        {
            foreach (PhysicalShape shape in shapes)
                shape.previousFrameShape = shape;

            interFrameVelocity = Vector3f.EMPTY;
            state.lockedAxis.Clear();
            state.position = targetPosition;
            Assert.NaN(state.position);
            UpdateShapes(true);
        }

        public Physical Clone(object newCreator)
        {
            Physical clone = new Physical(state.Clone(), newCreator, isSolid);
            clone.isStatic = isStatic;
            clone.getRelevantShapes = getRelevantShapes;
            clone.validateCollision = validateCollision;
            foreach (PhysicalShape s in shapes)
                clone.shapes.Add(s.CloneTypeless());

            return clone;
        }
    }
}
