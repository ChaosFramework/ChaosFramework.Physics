using ChaosFramework.IO;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using System.IO;
using static ChaosFramework.Math.Clamping;
using static ChaosFramework.Math.Exponentials;

namespace ChaosFramework.Physics.States
{
    public class SquishablePhysicsState : PhysicsState
    {
        static readonly Vector3f UP = new Vector3f(0, 1, 0);

        static SquishablePhysicsState Read(BinaryReader rd)
        {
            SquishablePhysicsState output = new SquishablePhysicsState();
            output.Load(rd);
            return output;
        }

        static void Write(BinaryWriter wr, SquishablePhysicsState obj)
            => obj.Save(wr);

        [ChaosIO.RegisterType]
        static void InitIO()
            => ChaosIO.AddType(Read, Write);

        public Matrix squishMatrix = Matrix.IDENTITY;
        public bool lockPosition = false;
        public float stiffness = 0.17f;
        public float sponginess = 1.0f;

        public override Vector3f Resolve(CollisionData.ShapeCollisionData data, Physical collisionPartner, bool flip, bool lockAxis)
        {
            Vector3f normal = flip ? data.penetrationNormal : -data.penetrationNormal;
            if (data.remainingPenetration <= 0 || normal == Vector3f.EMPTY)
                return Vector3f.EMPTY;

            Vector3f penetration = data.remainingPenetration * normal;
            if (lockAxis)
                lockedAxis.Add(new LockAxis(-normal, data));

            Vector3f actualMovement = penetration;
            foreach (LockAxis axis in lockedAxis)
            {
                if (Vector3f.Dot(axis.axis, normal) > 0)
                {
                    Assert.NaN(position);

                    actualMovement -= Vector3f.Dot(actualMovement, axis.axis) * axis.axis;
                    Vector3f resolvedPosition;
                    if (axis.cause == null)
                        resolvedPosition = flip ? data.s2.Support(-axis.axis) : data.s1.Support(axis.axis);
                    else
                        resolvedPosition = axis.cause.s1 == data.s1 ? axis.cause.impact2 : axis.cause.impact1;

                    float len1 = (data.impact1 - resolvedPosition).Length();
                    float len2 = (data.impact2 - resolvedPosition).Length();

                    SquishablePhysicsState otherSquishy = collisionPartner.state as SquishablePhysicsState;
                    float otherStiffness = otherSquishy?.stiffness ?? float.PositiveInfinity;
                    if (!float.IsInfinity(stiffness))
                    {
                        float lesserSquish = Sqrt(Max((data.impact1 - position).LengthSq(), (data.impact2 - position).LengthSq()));
                        Assert.NaN(lesserSquish);

                        if (lesserSquish < data.remainingPenetration)
                            lesserSquish = data.remainingPenetration + 0.01f;

                        float squish = 1 - (lesserSquish - data.remainingPenetration) / lesserSquish;
                        if (float.IsInfinity(otherStiffness))
                            squish /= stiffness;
                        else
                            squish *= otherStiffness / ((stiffness + otherStiffness) * stiffness);

                        if (squish > 1)
                            squish = 0.99f;

                        squish *= squishMatrix.determinant;
                        Assert.NaN(squish);

                        squishMatrix *= Squish(normal, 1 - squish);
                        Assert.NaN(squishMatrix);

                        penetration *= 1 - squish / 2;
                    }
                }
            }

            if (lockPosition)
                actualMovement = Vector3f.EMPTY;

            if (data.processedPenetration != 0)
                data.processedPenetration += Vector3f.Dot(actualMovement, normal)
                                           * collisionPartner.state.mass
                                           / (mass + collisionPartner.state.mass)
                                           / data.remainingPenetration;

            Vector3f v1 = flip ? data.v1 : data.v2;
            Vector3f v2 = flip ? data.v2 : data.v1;

            float bounce = (data.s1.bounce + data.s2.bounce) * 0.5f;
            if (collisionPartner.isStatic)
            {
                if (!lockPosition)
                {
                    position -= penetration;
                    Assert.NaN(position);
                }

                // TODO: Reimplement torque impact
                Vector3f collisionPartnerVelocity = v2; // collisionPartner.state.GetVelocityAt(data.absolutePointOfImpact);
                Vector3f relativeVelocity = v1 - v2; // GetVelocityAt(data.absolutePointOfImpact) - collisionPartnerVelocity;

                float relativeVelocityAlongNormal = Vector3f.Dot(relativeVelocity, normal);
                float deltaV = Max(0, relativeVelocityAlongNormal * (1 + bounce));
                float deltaVPlastic = Max(0, relativeVelocityAlongNormal);
                if (deltaV - deltaVPlastic < MINIMUM_BOUNCE_THRESHOLD)
                    deltaV = deltaVPlastic;

                float mixedContactFriction = contactFriction * collisionPartner.state.contactFriction;
                Vector3f flat = relativeVelocity - relativeVelocityAlongNormal * normal;
                relativeVelocity -= flat * mixedContactFriction;
                SetVelocityAt(
                    data.absolutePointOfImpact,
                    relativeVelocity - deltaV * normal + collisionPartnerVelocity,
                    flip ? data.s1 : data.s2
                    );

                return -penetration;
            }
            else
            {
                position -= actualMovement;
                Assert.NaN(position);

                Vector3f bounceNormal, flat;
                float deltaV, mixedContactFriction;
                ResolveVelocities(
                    actualMovement, normal, v1, v2, this, collisionPartner.state, bounce,
                    out deltaV, out bounceNormal, out flat, out mixedContactFriction
                    );

                // TODO: sample velocity at surface instead of using v1 directly, i.e. correct GetVelocityAt implementation
                SetVelocityAt(
                    data.absolutePointOfImpact,
                    v1 - deltaV * bounceNormal - flat * mixedContactFriction,
                    flip ? data.s1 : data.s2
                    );

                return -actualMovement;
            }
        }

        public override void Move(float time)
        {
            base.Move(time);
            squishMatrix += (Matrix.IDENTITY - squishMatrix) * EaseIn(time * sponginess);
            Assert.NaN(squishMatrix);
        }

        Matrix Squish(Vector3f axis, float squish)
        {
            if (axis == UP)
                return Matrix.Scaling(1, squish, 1);

            float angle = (float)System.Math.Acos(Vector3f.Dot(axis, UP));
            Vector3f rotAxis = Vector3f.Cross(axis, UP);

            if (rotAxis.LengthSq() < 0.000001f)
                return Matrix.Scaling(1, squish, 1);

            Matrix m = Matrix.RotationAxis(rotAxis, angle)
                     * Matrix.Scaling(1, squish, 1)
                     * Matrix.RotationAxis(rotAxis, -angle);

            Assert.NaN(m);
            return m;
        }

        public override Matrix GetTransform()
            => baseTransform
             * squishMatrix
             * Matrix.Translation(position);

        public override void Save(BinaryWriter wr)
        {
            base.Save(wr);
            wr.WriteAs(squishMatrix);
            wr.WriteAs(stiffness);
            wr.WriteAs(lockPosition);
        }

        public override void Load(BinaryReader rd)
        {
            base.Load(rd);
            squishMatrix = rd.Read<Matrix>();
            stiffness = rd.Read<float>();
            lockPosition = rd.Read<bool>();
        }
    }
}
