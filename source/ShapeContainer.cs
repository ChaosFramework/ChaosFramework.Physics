using ChaosFramework.Collections;
using ChaosFramework.Core;
using ChaosFramework.IO.Containers;
using ChaosFramework.IO.Streams;
using ChaosFramework.Math;
using ChaosFramework.Math.Vectors;
using System;
using System.IO;
using static System.Text.Encoding;

namespace ChaosFramework.Physics
{
    using Shapes;

    public class ShapeContainer : AssetContainer<PhysicalShape[]>
    {
        class ParserException : Exception
        {
            public readonly int line;

            public ParserException(string message, int line)
                : base($"{message} (Line: {line})")
            {
                this.line = line;
            }
        }

        static readonly char[] LINEBREAK_CHARACTERS = new[] { '\n', '\r' };
        static readonly char[] VECTOR_COMPONENT_SEPARATOR = new[] { ' ' };
        static readonly char[] LINE_TYPE_VALUE_SEPARATOR = new[] { ' ' };

        static string TrimString(string s)
            => s.Trim();

        static void Kill(PhysicalShape[] e)
        { }

        static AssetLoadException<PhysicalShape[]> CreateAssetLoadException(Key key, string msg, int line)
            => new AssetLoadException<PhysicalShape[]>(key, new ParserException(msg, line));

        static PhysicalShape[] ParseObjFile(Key key, string text)
        {
            bool parseIndividualFaces = false;
            float faceThickness = 1;
            string[] lines = Array.ConvertAll(
                text.Split(LINEBREAK_CHARACTERS, StringSplitOptions.RemoveEmptyEntries),
                TrimString
                );

            string currentObj = "#";
            LinkedList<PhysicalShape> shapes = new LinkedList<PhysicalShape>();
            LinkedList<Vector3f> shapeVertList = new LinkedList<Vector3f>();
            LinkedList<Vector3f> vertList = new LinkedList<Vector3f>();
            LinkedList<Vector3f> normalList = new LinkedList<Vector3f>();
            Vector3f[] vertCache = new Vector3f[0];
            Vector3f[] normalCache = new Vector3f[0];

            for (int i = 0; i < lines.Length; i++)
            {
                if (lines[i].StartsWith("#"))
                    continue;

                string[] line = lines[i].Split(LINE_TYPE_VALUE_SEPARATOR, 2);
                switch (line[0])
                {
                    case "o":
                        {
                            if (currentObj != "#" && !parseIndividualFaces)
                                CreateShape(currentObj, shapeVertList, shapes);

                            currentObj = line[1];
                            const string FACES_PREFIX = "Faces(";
                            int facesPrefixOffset = currentObj.IndexOf(FACES_PREFIX);
                            if (parseIndividualFaces = (facesPrefixOffset != -1))
                            {
                                int seekIndex = facesPrefixOffset + FACES_PREFIX.Length;
                                string thicknessStr = currentObj.Substring(seekIndex, currentObj.IndexOf(')') - seekIndex);
                                if (!float.TryParse(thicknessStr, out faceThickness))
                                    throw CreateAssetLoadException(key, "Failed to parse face thickness.", i);
                            }
                            break;
                        }

                    case "v":
                        {
                            if (currentObj == "#")
                                throw CreateAssetLoadException(key, "Unexpected token 'v'. Start an object first.", i);

                            Vector3f v;
                            if (Vector3f.TryParse(line[1], VECTOR_COMPONENT_SEPARATOR, out v))
                            {
                                vertList.Add(new Vector3f(-v.x, v.y, v.z));
                                if (!parseIndividualFaces)
                                    shapeVertList.Add(vertList.last);
                            }
                            else
                                throw CreateAssetLoadException(key, "Whatever you tried to disguise as a vertex here. It didn't work.", i);

                            break;
                        }

                    case "vn":
                        {
                            if (currentObj == "#")
                                throw CreateAssetLoadException(key, "Unexpected token 'vn'. Start an object first.", i);

                            Vector3f v;
                            if (Vector3f.TryParse(line[1], VECTOR_COMPONENT_SEPARATOR, out v))
                                normalList.Add(new Vector3f(-v.x, v.y, v.z));
                            else
                                throw CreateAssetLoadException(key, "Whatever you tried to disguise as a normal here. It didn't work.", i);

                            break;
                        }

                    case "f":
                        if (!parseIndividualFaces)
                            break;

                        string[] vertSplits = line[1].Split(' ');
                        Vector3f[] faceVertices = new Vector3f[vertSplits.Length * 2];
                        for (int k = 0; k < vertSplits.Length; k++)
                        {
                            string[] tmp = vertSplits[k].Split('/');
                            int vertIndex, normalIndex;
                            if (tmp.Length < 3
                             || !int.TryParse(tmp[0], out vertIndex)
                             || !int.TryParse(tmp[2], out normalIndex))
                                throw CreateAssetLoadException(key, $"Whatever you tried to disguise as a face here. It didn't work.", i);

                            if (vertIndex > vertCache.Length && vertIndex <= vertList.length)
                                vertCache = vertList.ToArray();

                            if (normalIndex > normalCache.Length && normalIndex <= normalList.length)
                                normalCache = normalList.ToArray();

                            faceVertices[k] = vertCache[vertIndex - 1];
                            faceVertices[k + vertSplits.Length] = faceVertices[k] - normalCache[normalIndex - 1] * faceThickness;
                        }

                        shapes.Add(new MeshShape(faceVertices));
                        shapes.last.name = currentObj;
                        break;

                    default:
                        // Ignore irrelevant obj tags.
                        break;
                }
            }

            if (!parseIndividualFaces)
                CreateShape(currentObj, shapeVertList, shapes);

            return shapes.ToArray();
        }

        static void CreateShape(string name, LinkedList<Vector3f> pos, LinkedList<PhysicalShape> shapes)
        {
            Vector3f[] posArr = pos.ToArray();
            shapes.Add(name.EndsWith("_Sphere") ? (PhysicalShape)SphereShape.FromPointCloud(posArr) : new MeshShape(posArr, true));
            shapes.last.name = name;

            pos.Clear();
        }

        public ShapeContainer(StreamSource streamSource)
            : base(streamSource, false)
        { }

        protected override void DisposeItem(PhysicalShape[] obj)
        { }

        protected override PhysicalShape[] LoadFromStream(Key key, Stream resource, CancellationToken cancel)
        {
            if (key.key.EndsWith(".obj"))
            {
                byte[] buffer = new byte[resource.Length];
                resource.Read(buffer, 0, buffer.Length);
                return ParseObjFile(key, ASCII.GetString(buffer));
            }
            else if (key.key.EndsWith(".gmdl") || key.key.EndsWith(".agmdl"))
                using (BinaryReader rd = new BinaryReader(resource))
                    return new[] { new MeshShape(MeshData.FromStream(rd).pos, true) };

            throw new Exception("Invalid file extension.");
        }

        public override Entry Load(string key, Disposable monitor1, params Disposable[] monitors)
        {
            PhysicalShape[] baseShapes = base.Load(key, monitor1, monitors);
            PhysicalShape[] output = new PhysicalShape[baseShapes.Length];
            for (int i = 0; i < output.Length; i++)
                output[i] = baseShapes[i].CloneTypeless();

            return Entry.Mock((_, __) => output, Kill);
        }
    }
}
