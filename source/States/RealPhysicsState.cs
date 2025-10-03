using ChaosFramework.IO;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using ChaosFramework.Shapes.Convex;
using System.IO;
using static ChaosFramework.Math.Exponentials;
using static ChaosFramework.Math.Transforms;

namespace ChaosFramework.Physics.States
{
    public class RealPhysicsState : SquishablePhysicsState
    {
        static RealPhysicsState Read(BinaryReader rd)
        {
            RealPhysicsState output = new RealPhysicsState();
            output.Load(rd);
            return output;
        }

        static void Write(BinaryWriter wr, RealPhysicsState obj)
            => obj.Save(wr);

        [ChaosIO.RegisterType]
        static void InitIO()
            => ChaosIO.AddType(Read, Write);

        public Quaternion orientation = Quaternion.IDENTITY;
        public Vector3f angularVelocity, angularAcceleration;
        public float angularFriction = 0.5f;
        float inertia;

        public override void Move(float time)
        {
            base.Move(time);
            inertia = mass / 6;
            angularVelocity += angularAcceleration * (time / inertia);
            angularVelocity -= angularVelocity * EaseIn(time * angularFriction);
            angularAcceleration = Vector3f.EMPTY;
            orientation = IncrementOrientation(orientation, angularVelocity, time);
        }

        public override void ApplyForce(Vector3f force, Vector3f absolutePosition, float time)
        {
            base.ApplyForce(force, absolutePosition, time);
            angularAcceleration += Vector3f.Cross(force, absolutePosition - position);
        }

        public override Matrix GetTransform()
            => baseTransform
             * Matrix.RotationQuaternion(orientation)
             * squishMatrix
             * Matrix.Translation(position);

        public override Vector3f GetVelocityAt(Vector3f absolutePosition, Shape shape)
            => velocity + Vector3f.Cross(position - absolutePosition, angularVelocity);

        public override void SetVelocityAt(Vector3f absolutePosition, Vector3f velocity, Shape shape)
        {
            Vector3f currentVelocity = GetVelocityAt(absolutePosition, shape);
            Vector3f difference = velocity - currentVelocity;
            Vector3f rotationAxis = Vector3f.Normalize(absolutePosition - position);
            float impulseFactor = Vector3f.Dot(rotationAxis, Vector3f.Normalize(-difference));
            float distSq = (absolutePosition - position).LengthSq();
            if (distSq > 0.01f)
                angularVelocity -= Vector3f.Cross(difference, absolutePosition - position) * (1 / distSq);

            if (impulseFactor < 0)
                return;

            this.velocity += difference * impulseFactor;
            Assert.NaN(velocity);
        }

        public override void Save(BinaryWriter wr)
        {
            base.Save(wr);
            wr.WriteAs(orientation);
            wr.WriteAs(angularVelocity);
            wr.WriteAs(angularFriction);
        }

        public override void Load(BinaryReader rd)
        {
            base.Load(rd);
            orientation = rd.Read<Quaternion>();
            angularVelocity = rd.Read<Vector3f>();
            angularFriction = rd.Read<float>();
        }
    }
}
