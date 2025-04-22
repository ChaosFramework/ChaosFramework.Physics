using ChaosFramework.Collections;
using ChaosFramework.Components;
using ChaosFramework.Math.Vectors;
using SysCol = System.Collections.Generic;

namespace ChaosFramework.Physics.Fields
{
    public abstract partial class Field2D : Component
    {
        protected readonly ChannelDescriptor[] channels;
        protected readonly SysCol.Dictionary<int, LinkedList<ChannelDelta>> channelDeltas;

        public int updateLayer { get; private set; }
        public Vector2i dimensions { get; protected set; }
        public Vector2f lowBounds { get; protected set; }
        public float interval { get; protected set; }
        protected float intervalFactor { get; private set; }

        protected int numChannels => channels.Length;

        public Field2D(params ChannelDescriptor[] channels)
        {
            channelDeltas = new SysCol.Dictionary<int, LinkedList<ChannelDelta>>();
            this.channels = channels;
            for (int i = 0; i < channels.Length; i++)
                channels[i].channel = i;
        }

        protected override void Create(CreateParameters cparams)
            => updateLayer = CreateParameters.RequireAs<int>(cparams).v1;

        public override void SetUpdateCalls()
        {
            base.SetUpdateCalls();
            scene.updateLayers[updateLayer].Add(Update);
        }

        public virtual void InitializeField(Vector2f low, Vector2f high, float interval = 1, int outOfBoundsPoints = 0)
        {
            this.interval = interval;
            intervalFactor = 1 / (interval * interval);
            
            lowBounds = low - outOfBoundsPoints * interval;
            dimensions = new Vector2i((high - low) / interval) + outOfBoundsPoints * 2;
        }

        void Update()
        {
            if (alive)
                NormalizeField();
        }

        protected void AddChannelDelta(
            int channel,
            Vector2f position,
            float value,
            float radius,
            float fallOff,
            DeltaOp operation = DeltaOp.Add
            )
        {
            if (doUpdate)
            {
                LinkedList<ChannelDelta> deltaList;
                if (!channelDeltas.TryGetValue(channel, out deltaList))
                    channelDeltas[channel] = deltaList = new LinkedList<ChannelDelta>();

                deltaList.Add(new ChannelDelta(position, value, radius, fallOff, operation));
            }
        }

        public bool EvaluateRequest(object requestClient, out float[] channels)
        {
            channels = new float[this.channels.Length];
            for (int i = 0; i < channels.Length; i++)
                if (EvaluateRequest(i, requestClient, out channels[i]))
                    channels[i] = this.channels[i].projection(this, channels[i]);
                else
                    return false;

            return true;
        }

        public abstract bool EvaluateRequest(int channel, object requestClient, out float value);
        public abstract void RegisterRequest(object requestClient, Vector2f texCoords);
        protected abstract void ClearChannel(int channel, float value);
        protected abstract void ApplyExternalChanges();
        protected abstract void NormalizeField();
    }
}
