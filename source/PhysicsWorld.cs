using ChaosFramework.Collections;
using ChaosFramework.Components;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using ChaosFramework.Shapes.Convex;
using System;
using System.Diagnostics;
using static ChaosFramework.Math.Clamping;
using static ChaosFramework.Math.Exponentials;
using SysCol = System.Collections.Generic;

namespace ChaosFramework.Physics
{
    using Forces;
    using States;

    public class PhysicsWorld : Component
    {
        public readonly LinkedList<ForceField> forces = new LinkedList<ForceField>();

        public float interFrameCollisionVelocityThresholdSq = 100;

        public float speedThreshold = 0.01f;
        public Vector3f[] globalLockedAxis = new Vector3f[] { };
        public float bvhMargin = 0.5f;

        LinkedList<Action> updateLayer;

        DynamicBvh rayCastBvh = new DynamicBvh();
        SysCol.Dictionary<Physical, BvhNode> rayCastNodes = new SysCol.Dictionary<Physical, BvhNode>();

        AdvancedLinkedList<Physical> rigids = new AdvancedLinkedList<Physical>();
        AdvancedLinkedList<Physical> nonSolids = new AdvancedLinkedList<Physical>();

        protected override void Create(CreateParameters cparams)
            => updateLayer = scene.updateLayers[CreateParameters.RequireAs<int>(cparams).v1];

        public void Add(Physical p)
        {
            if (p.isSolid)
            {
                if (rigids.AddUnique(p))
                {
                    BvhNode rayCastNode = new BvhNode(new Bounds3f(p.boundingBoxLow, p.boundingBoxHigh), p);
                    rayCastNodes[p] = rayCastNode;
                    rayCastBvh.InsertLeaf(rayCastNode);
                }
            }
            else
                nonSolids.AddUnique(p);
        }

        public void Remove(Physical p)
        {
            if (p.isSolid)
            {
                BvhNode node;
                if (p != null && rayCastNodes.TryGetValue(p, out node))
                {
                    // TODO: Do this more sensibly
                    p.currentCollisions.Clear();
                    rayCastBvh.RemoveLeaf(node);
                    rayCastNodes.Remove(p);
                }
                rigids.Remove(p);
            }
            else
                nonSolids.Remove(p);
        }

        public override void SetUpdateCalls()
        {
            base.SetUpdateCalls();
            updateLayer.Add(Update);
        }

        void Update()
        {
            ApplyForces();
            UpdateCollisions();
            float speedThresholdSq = speedThreshold * speedThreshold;
            foreach (Physical p in rigids)
                if (p.state.velocity.LengthSq() < speedThresholdSq)
                    p.state.velocity = Vector3f.EMPTY;
        }

        void ApplyForces()
        {
            foreach (ForceField f in forces)
                foreach (Physical p in rigids)
                    if (!p.isStatic)
                        f.ApplyTo(p, ftime);
        }

        void ReInsert(DynamicBvh bvh, BvhNode leaf, float margin)
        {
            Physical p = (Physical)leaf.userData;
            if (leaf.aabb.Fits(p.boundingBoxLow, p.boundingBoxHigh))
                return;

            bvh.RemoveLeaf(leaf);
            leaf.aabb.low = p.boundingBoxLow - margin;
            leaf.aabb.high = p.boundingBoxHigh + margin;
            bvh.InsertLeaf(leaf);
        }

        void UpdateCollisions()
        {
            foreach (Physical p in rigids)
            {
                Matrix appliedTransform = p.appliedTransform;
                if (p.lastOldShapeUpdateTransform != appliedTransform)
                {
                    foreach (Shape shape in p.shapes)
                        shape.AdvanceFrame();

                    p.lastOldShapeUpdateTransform = appliedTransform;
                }
            }

            // TODO: something else
            Collision.interFrameCollisionVelocityThresholdSq = interFrameCollisionVelocityThresholdSq;
            Collision.interFrameCollisionVelocityThreshold = Sqrt(interFrameCollisionVelocityThresholdSq);

            DynamicBvh currentBvh = new DynamicBvh();
            DynamicBvh restBvh = new DynamicBvh();
            SysCol.HashSet<BvhNode> currentNodes = new SysCol.HashSet<BvhNode>();

            for (int i = 0; i < globalLockedAxis.Length; i++)
                globalLockedAxis[i].Normalize();

            foreach (Physical p in rigids)
            {
                foreach (PhysicsState.EnumerateConstraints constraintProvider in p.state.constraintProviders)
                    foreach (Vector3f constraint in constraintProvider())
                        p.state.lockedAxis.Add(new PhysicsState.LockAxis(constraint, null));

                foreach (PhysicsState.LockAxis axis in p.state.lockedAxis)
                    if (axis.GetFrameThenIncrement() > 0)
                        p.state.lockedAxis.RemoveCurrent();

                p.UpdateShapes();

                foreach (Vector3f globalLock in globalLockedAxis)
                {
                    if (Vector3f.Dot(globalLock, p.state.velocity) > 0)
                    {
                        p.state.velocity -= globalLock * Vector3f.Dot(p.state.velocity, globalLock);
                        Assert.NaN(p.state.velocity);
                    }

                    p.state.lockedAxis.Add(new PhysicsState.LockAxis(globalLock, null));
                }

                p.interFrameVelocity = p.state.velocity;
                p.currentCollisions.Clear();

                BvhNode node = new BvhNode();
                node.userData = p;
                node.aabb = p.isStatic
                            ? new Bounds3f(p.boundingBoxLow, p.boundingBoxHigh)
                            : new Bounds3f(p.boundingBoxLow - bvhMargin, p.boundingBoxHigh + bvhMargin);

                if (p.isStatic)
                {
                    currentNodes.Add(node);
                    if (p.checkStaticCollisions)
                    {
                        currentBvh.InsertLeaf(node);
                        foreach (BvhNode other in currentBvh.RecursiveDetect(node))
                        {
                            CollisionData data = p.CheckCollision((Physical)other.userData);
                            if (data.data.length > 0)
                            {
                                data.p1.currentCollisions.Add(data);
                                data.p2.currentCollisions.Add(data);
                            }
                        }
                    }
                }
                else
                    restBvh.InsertLeaf(node);
            }

            const int MAX_RESOLUTION_STEPS = 100;
            if (!restBvh.empty)
                for (int i = 0; i < MAX_RESOLUTION_STEPS && currentNodes.Count > 0; i++)
                {
                    SysCol.HashSet<BvhNode> nextCurrentNodes = new SysCol.HashSet<BvhNode>();
                    foreach (BvhNode node in currentNodes)
                    {
                        Physical physical = (Physical)node.userData;

                        // Detect and resolve collisions within this iteration.
                        // This is only necessary for any iteration past the first,
                        // since the first iteration consists of only static objects.
                        if (i > 0)
                        {
                            // Remove from the current collision tree to avoid redundant pairings.
                            currentBvh.RemoveLeaf(node);
                            foreach (BvhNode other in currentBvh.RecursiveDetect(node))
                                if (ResolveCollision(physical.CheckCollision((Physical)other.userData)))
                                    ReInsert(currentBvh, other, bvhMargin);
                        }

                        // Detect and resolve collisions that will go to the next iteration.
                        foreach (BvhNode other in restBvh.RecursiveDetect(node))
                            if (ResolveCollision(physical.CheckCollision((Physical)other.userData)))
                            {
                                ReInsert(restBvh, other, bvhMargin);
                                nextCurrentNodes.Add(other);
                            }
                    }

                    if (i == 0)
                        currentBvh = new DynamicBvh();

                    // Migrate collided objects from rest to current
                    foreach (BvhNode node in nextCurrentNodes)
                    {
                        restBvh.RemoveLeaf(node);
                        currentBvh.InsertLeaf(node);
                    }

                    currentNodes = nextCurrentNodes;
                }

            // Process collisions between remaining objects
            foreach (BvhNode node in restBvh.EnumerateLeaves())
            {
                restBvh.RemoveLeaf(node);
                foreach (BvhNode other in restBvh.RecursiveDetect(node))
                    if (ResolveCollision(((Physical)node.userData).CheckCollision((Physical)other.userData), false))
                        ReInsert(restBvh, other, bvhMargin);
            }

            foreach (Physical p in rigids)
            {
                Debug.Assert(p.isSolid);

                // Update inter-frame shapes
                if (p.state.velocity.LengthSq() > interFrameCollisionVelocityThresholdSq)
                    foreach (Shape shape in p.shapes)
                        shape.AdvanceFrame();

                // Update ray bvh
                const float RAY_BVH_MARGIN = 1;
                UpdateRayBvh(p, RAY_BVH_MARGIN);
            }

            // Detect collisions for non-solid Physicals
            foreach (Physical nonSolid in nonSolids)
            {
                Debug.Assert(!nonSolid.isSolid);

                nonSolid.UpdateShapes();
                nonSolid.currentCollisions.Clear();
                foreach (Physical solid in rigids)
                {
                    CollisionData data = nonSolid.CheckCollision(solid);
                    if (data.data.length > 0)
                    {
                        nonSolid.currentCollisions.Add(data);
                        solid.currentCollisions.Add(data);
                    }
                }
            }

            Collision.relativeVelocity = Vector3f.EMPTY;
        }

        private bool ResolveCollision(CollisionData data, bool lockAxis = true)
        {
            Debug.Assert(!nonSolids.Contains(data.p1));
            Debug.Assert(rigids.Contains(data.p1));
            Debug.Assert(!nonSolids.Contains(data.p2));
            Debug.Assert(rigids.Contains(data.p2));
            Debug.Assert(!(data.p1.isStatic && data.p2.isStatic));

            if (data.data.length == 0)
                return false;

            if (!(data.p1.validateCollision(data.p2, data) && data.p2.validateCollision(data.p1, data)))
                return false;

            data.p1.currentCollisions.Add(data);
            data.p2.currentCollisions.Add(data);

            foreach (CollisionData.ShapeCollisionData shapeData in data.data)
            {
                Vector3f offset = Vector3f.EMPTY;

                if (!data.p1.isStatic)
                    offset -= data.p1.state.Resolve(shapeData, data.p2, true, lockAxis);

                if (!data.p2.isStatic)
                    offset += data.p2.state.Resolve(shapeData, data.p1, false, lockAxis);

                data.data.SetSubEnumerator();
                foreach (CollisionData.ShapeCollisionData toBeModified in data.data)
                {
                    toBeModified.remainingPenetration -= Max(0, Vector3f.Dot(toBeModified.penetrationNormal, offset));
                    if (toBeModified.remainingPenetration < 0)
                        data.data.RemoveCurrent();
                }
            }

            if (!data.p1.isStatic)
                data.p1.UpdateShapes();

            if (!data.p2.isStatic)
                data.p2.UpdateShapes();

            return true;
        }

        void UpdateRayBvh(Physical p, float margin)
        {
            BvhNode rayCastNode;
            if (rayCastNodes.TryGetValue(p, out rayCastNode))
                if (!rayCastNode.aabb.Fits(p.boundingBoxLow, p.boundingBoxHigh))
                {
                    rayCastBvh.RemoveLeaf(rayCastNode);
                    rayCastNode.aabb = new Bounds3f(
                        p.boundingBoxLow - margin,
                        p.boundingBoxHigh + margin
                        );

                    rayCastBvh.InsertLeaf(rayCastNode);
                }
        }

        public RayImpact ShootRay(Ray ray)
            => ShootRay(ray, Linq.PredicateTrue);

        public RayImpact ShootRay(Ray ray, Predicate<Physical> validTargetSelector)
            => ShootRay(this, ray, validTargetSelector);

        public RayImpact ShootRay(object shooter, Ray ray)
            => ShootRay(shooter, ray, Linq.PredicateTrue);

        public RayImpact ShootRay(object shooter, Ray ray, Predicate<Physical> validTargetSelector)
        {
            Physical rayShape = new Physical(shooter, false);
            rayShape.AddShape(new Shape[] { null });
            rayShape.state.position = ray.origin;

            Assert.NaN(rayShape.state.position);

            // TODO: Properly return null if nothing is hit and handle correctly for all callers
            RayImpact impact = new RayImpact(ray.origin, ray.direction);
            impact.impact = ray.origin + ray.direction;
            impact.distanceSq = ray.direction.LengthSq();
            Vector3f direction = ray.direction / Sqrt(impact.distanceSq);

            foreach (BvhNode bvhNode in rayCastBvh.RecursiveDetect(ray))
            {
                Physical physical = bvhNode.userData as Physical;

                if (!validTargetSelector(physical))
                    continue;

                if ((ray.Project(physical.state.position) - physical.state.position).LengthSq() > physical.hullRadius * physical.hullRadius)
                    continue;

                rayShape.shapes[0] = new MeshShape(new Vector3f[] { Vector3f.EMPTY, direction * Sqrt(impact.distanceSq) });
                rayShape.UpdateShapes(true);
                foreach (Shape s in physical.getRelevantShapes(rayShape))
                    foreach (Vector3f hit in s.CheckLine(ray.origin, direction))
                    {
                        if (Vector3f.Dot(hit - ray.origin, direction) < 0)
                            continue;

                        float impactDistSq = (hit - ray.origin).LengthSq();
                        if (impactDistSq < impact.distanceSq)
                        {
                            impact.impact = hit;
                            impact.distanceSq = impactDistSq;
                            impact.physical = physical;
                            impact.shape = s;
                        }
                    }
            }

            return impact;
        }
    }
}
