using ChaosFramework.Collections;
using ChaosFramework.IO;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using System.IO;
using static ChaosFramework.Math.Clamping;
using SysCol = System.Collections.Generic;

namespace ChaosFramework.Physics.States
{
    public partial class PhysicsState
    {
        public delegate SysCol.IEnumerable<Vector3f> EnumerateConstraints();

        protected const float MINIMUM_BOUNCE_THRESHOLD = 0.3f;

        protected static void ResolveVelocities(
            Vector3f actualMovement,
            Vector3f normal,
            Vector3f v1,
            Vector3f v2,
            PhysicsState p1,
            PhysicsState p2,
            float bounce,
            out float deltaV,
            out Vector3f bounceNormal,
            out Vector3f flat,
            out float mixedContactFriction
            )
        {
            foreach (LockAxis axis in p1.lockedAxis)
            {
                float f = Vector3f.Dot(axis.axis, normal);
                if (f > 0)
                    normal -= f * axis.axis;
            }

            bounceNormal = Vector3f.Normalize(normal);

            float dot1 = Vector3f.Dot(v1, bounceNormal);
            float dot2 = Vector3f.Dot(v2, bounceNormal);

            float totalMass = p1.mass + p2.mass;

            deltaV = Max(0, (dot1 - ((p1.mass * dot1 + p2.mass * dot2 - p2.mass * (dot1 - dot2) * bounce) / totalMass)));
            float deltaVPlastic = Max(0, (dot1 - ((p1.mass * dot1 + p2.mass * dot2) / totalMass)));
            if (deltaV - deltaVPlastic < MINIMUM_BOUNCE_THRESHOLD)
                deltaV = deltaVPlastic;

            // TODO: Reimplement torque impact
            // Vector3f collisionPartnerVelocity = v2; // collisionPartner.state.GetVelocityAt(data.absolutePointOfImpact);
            Vector3f relativeVelocity = v1 - v2; // GetVelocityAt(data.absolutePointOfImpact) - collisionPartnerVelocity;
            float relativeVelocityAlongNormal = Vector3f.Dot(relativeVelocity, bounceNormal);
            mixedContactFriction = p1.contactFriction * p2.contactFriction;
            flat = relativeVelocity - relativeVelocityAlongNormal * bounceNormal;
        }

        static PhysicsState Read(BinaryReader rd)
        {
            PhysicsState output = new PhysicsState();
            output.Load(rd);
            return output;
        }

        static void Write(BinaryWriter wr, PhysicsState obj)
            => obj.Save(wr);

        [ChaosIO.RegisterType]
        static void InitIO()
            => ChaosIO.AddType(Read, Write);

        public readonly SysCol.HashSet<EnumerateConstraints> constraintProviders = new SysCol.HashSet<EnumerateConstraints>();

        public Vector3f position;
        public Vector3f velocity;
        public float mass = 1f;
        public float contactFriction = 0.5f;
        public Matrix baseTransform = Matrix.IDENTITY;

        internal AdvancedLinkedList<LockAxis> lockedAxis = new AdvancedLinkedList<LockAxis>();

        public virtual Vector3f Resolve(CollisionData.ShapeCollisionData data, Physical collisionPartner, bool flip, bool lockAxis)
        {
            Vector3f normal = flip ? data.penetrationNormal : -data.penetrationNormal;
            if (data.remainingPenetration <= 0 || normal == Vector3f.EMPTY)
                return Vector3f.EMPTY;

            Vector3f penetration = data.remainingPenetration * normal;
            if (lockAxis)
                lockedAxis.Add(new LockAxis(-normal, data));

            Vector3f actualMovement = penetration * (1 - data.processedPenetration);
            foreach (LockAxis axis in lockedAxis)
            {
                if (Vector3f.Dot(axis.axis, normal) > 0)
                    actualMovement -= Vector3f.Dot(actualMovement, axis.axis) * axis.axis;
            }

            data.processedPenetration += Vector3f.Dot(actualMovement, normal)
                                       * collisionPartner.state.mass
                                       / (mass + collisionPartner.state.mass)
                                       / data.remainingPenetration;

            Vector3f v1 = flip ? data.v1 : data.v2;
            Vector3f v2 = flip ? data.v2 : data.v1;

            float bounce = (data.s1.bounce + data.s2.bounce) * 0.5f;
            if (collisionPartner.isStatic)
            {
                position -= penetration;
                Assert.NaN(position);

                // TODO: Reimplement torque impact
                Vector3f collisionPartnerVelocity = v2; // collisionPartner.state.GetVelocityAt(data.absolutePointOfImpact);
                Vector3f relativeVelocity = v1 - v2;    // GetVelocityAt(data.absolutePointOfImpact) - collisionPartnerVelocity;

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

                Vector3f ownImpactVelocity = GetVelocityAt(data.absolutePointOfImpact, flip ? data.s1 : data.s2);
                SetVelocityAt(
                    data.absolutePointOfImpact,
                    ownImpactVelocity - deltaV * bounceNormal - flat * mixedContactFriction,
                    flip ? data.s1 : data.s2
                    );

                return -actualMovement;
            }
        }

        public virtual void ApplyForce(Vector3f force, Vector3f absolutePosition, float time)
        {
            Vector3f delta = force * (time / mass);
            foreach (LockAxis axis in lockedAxis)
                if (Vector3f.Dot(force, axis.axis) < 0)
                    delta -= Vector3f.Dot(axis.axis, delta) * axis.axis;

            velocity += delta;
            Assert.NaN(velocity);
        }

        public virtual Matrix GetTransform()
            => baseTransform * Matrix.Translation(position);

        public virtual Vector3f GetVelocityAt(Vector3f absolutePosition, Shapes.PhysicalShape shape)
            => velocity;

        public virtual void Move(float time)
        {
            position += velocity * time;
            Assert.NaN(position);
        }

        public virtual void Save(BinaryWriter wr)
        {
            wr.WriteAs(position);
            wr.WriteAs(velocity);
            wr.WriteAs(baseTransform);
            wr.WriteAs(mass);
            wr.WriteAs(contactFriction);
        }

        public virtual void Load(BinaryReader rd)
        {
            position = rd.Read<Vector3f>();
            velocity = rd.Read<Vector3f>();
            baseTransform = rd.Read<Matrix>();
            mass = rd.Read<float>();
            contactFriction = rd.Read<float>();
        }

        public PhysicsState Clone()
        {
            PhysicsState clone = System.Activator.CreateInstance(GetType()) as PhysicsState;
            using (MemoryStream str = new MemoryStream())
            using (BinaryWriter wr = new BinaryWriter(str))
            {
                Save(wr);
                wr.Flush();
                str.Position = 0;
                using (BinaryReader rd = new BinaryReader(str))
                    clone.Load(rd);
            }

            return clone;
        }

        public virtual void SetVelocityAt(Vector3f absolutePosition, Vector3f velocity, Shapes.PhysicalShape shape)
            => this.velocity = velocity;
    }
}
