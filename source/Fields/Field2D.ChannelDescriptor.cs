namespace ChaosFramework.Physics.Fields
{
    partial class Field2D
    {
        public class ChannelDescriptor
        {
            public delegate float SampleProjection(Field2D field, float rawValue);

            static float ProjectIdentity(Field2D field, float rawValue)
                => rawValue;

            public readonly SampleProjection projection;

            internal int channel;

            public ChannelDescriptor(SampleProjection projection = null)
            {
                this.projection = projection ?? ProjectIdentity;
            }

            public bool Sample(Field2D field, object requestClient, out float value)
            {
                if (field.EvaluateRequest(channel, requestClient, out value))
                {
                    value = projection(field, value);
                    return true;
                }

                return false;
            }
        }
    }
}
