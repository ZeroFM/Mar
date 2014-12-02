using Microsoft.Ccr.Core;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;
using System;
using System.Collections.Generic;
using W3C.Soap;
using Microsoft.Dss.Core.DsspHttp;

using Microsoft.Robotics.RoboChamps.MarsChallenger.CommonLib;
using stereo = Microsoft.Robotics.RoboChamps.MarsChallenger.StereoVision;
using processor = Microsoft.Robotics.RoboChamps.MarsChallenger.ImageProcessing;
using comlink = Microsoft.Robotics.RoboChamps.MarsComLink.Proxy;
using spectro = Microsoft.Robotics.RoboChamps.MarsSpectrometer.Proxy;

namespace Microsoft.Robotics.RoboChamps.MarsChallenger
{
    public sealed class Contract
    {
        [DataMember]
        public const String Identifier = "http://www.robochamps.com/2009/02/marschallenger.html";
    }

    [DataContract]
    public class MarsChallengerState
    {
        /// <summary>
        /// a list store the mission need to do
        /// </summary>
        public LinkedList<Mission> MissionList;

        /// <summary>
        /// a memory of conlink data
        /// </summary>
        public ComlinkMemory ComlinkMemory;
        
        /// <summary>
        /// a memory use to draw the map and path plan
        /// </summary>
        public Memory Memory;
        
        /// <summary>
        /// a list store the vision data
        /// </summary>
        public VisionMemory VisionMemory;
        
        /// <summary>
        /// current mision
        /// </summary>
        [DataMember]
        public Mission current_M;
        
        /// <summary>
        /// Current rover state
        /// </summary>
        [DataMember]
        public RoverState State { get; set; }

        /// <summary>
        /// Last received spectrometer data
        /// </summary>
        [DataMember]
        public spectro.MarsSpectrometerState SpectrometerData { get; set; }

        /// <summary>
        /// Last image processing result for front cam
        /// </summary>
        [DataMember]
        public processor.ImageResult FrontCamData { get; set; }

        /// <summary>
        /// Last image processing result for rear cam
        /// </summary>
        [DataMember]
        public processor.ImageResult RearCamData { get; set; }

        /// <summary>
        /// state of the drive
        /// </summary>
        [DataMember]
        public DriveState DriveState { get; set; }
        
        [DataMember]
        public HeadState HeadState { get; set; }
        
        [DataMember]
        public DateTime BeginAt { get; set; }
        
        /// <summary>
        /// sencers send message interval
        /// </summary>
        [DataMember]
        public SensorInterval SI { get; set; }
    }

    public class ComlinkMemory
    {
        public LinkedList<comlink.MarsComLinkState> Data;
        const int DataSize = 10;
        public ComlinkMemory() { Data = new LinkedList<comlink.MarsComLinkState>(); }
        public void Update(comlink.MarsComLinkState data) 
        { 
            Data.AddLast(data);
            if (Data.Count > DataSize)
                Data.RemoveFirst();
        }
        public comlink.MarsComLinkState Get(DateTime time)
        {
            if (Data.Last.Value.Timestamp.CompareTo(time) < 0 || Data.Count <= 1)
                return Data.Last.Value;
            else if (Data.First.Value.Timestamp.CompareTo(time) >= 0)
                return Data.First.Value;
            LinkedListNode<comlink.MarsComLinkState> data = Data.First;
            comlink.MarsComLinkState res = new comlink.MarsComLinkState();
            res.Position=new comlink.Position();
            while(data.Next!=null)
            {
                if (data.Next.Value.Timestamp.CompareTo(time) < 0)
                {
                    data = data.Next;
                    continue;
                }
                float p = (float)(data.Next.Value.Timestamp.Subtract(time).TotalMilliseconds / data.Next.Value.Timestamp.Subtract(data.Value.Timestamp).TotalMilliseconds);
                res.Position.X = p * data.Value.Position.X + (1 - p) * data.Next.Value.Position.X;
                res.Position.Y = p * data.Value.Position.Y + (1 - p) * data.Next.Value.Position.Y;
                res.Position.Z = p * data.Value.Position.Z + (1 - p) * data.Next.Value.Position.Z;
                res.Yaw = p * data.Value.Yaw + (1 - p) * data.Next.Value.Yaw;
                res.Pitch = p * data.Value.Pitch + (1 - p) * data.Next.Value.Pitch;
                res.Roll = p * data.Value.Roll + (1 - p) * data.Next.Value.Roll;
                break;
            }
            return res;
        }
        public comlink.MarsComLinkState Last() { return Data.Last.Value; }
        public Vector3D Check()
        {
            return new Vector3D(MarsChallengerService.Distance(Data.Last.Previous.Value.Position, Data.Last.Value.Position),
              Math.Abs(Data.Last.Previous.Value.Yaw - Data.Last.Value.Yaw) % 180, 
              Data.Last.Value.Timestamp.Subtract(Data.Last.Previous.Value.Timestamp).TotalMilliseconds);
        }
    }

    public class Memory
    {
        public const int Size = 500;
        public const float Unit = 0.5f;  //m
        public float[,] Map { get; set; }
        public float[,] Reliability { get; set; }
        public float Center_x { get; set; }
        public float Center_z { get; set; }
        public LinkedList<float[,]> Maps;
        public LinkedList<Vector2D> Maps_center;

        const int imagesize = 128;
        const float view_threshold = 20;//m
        Vector3D[,] ImageCoo;

        DateTime UpdateTime;
        public Memory(float x, float z)
        {
            Center_x = x;
            Center_z = z; 
            Map = new float[Size*2+1, Size*2+1]; 
            Reliability = new float[Size*2+1, Size*2+1]; 
            Maps = new LinkedList<float[,]>();
            Maps_center = new LinkedList<Vector2D>();
            ImageCoo = new Vector3D[imagesize, imagesize];

            random = new Random((int)DateTime.Now.ToBinary());

            UpdateTime = DateTime.Now;
        }
        public void MapUpdate(comlink.Position P)
        {
            int x = (int)Math.Round((P.X - Center_x) / Unit);
            int z = (int)Math.Round((P.Z - Center_z) / Unit);
            if (Math.Abs(x) > Size || Math.Abs(z) > Size)
            {
                Maps.AddFirst(Map);
                Maps_center.AddFirst(new Vector2D(Center_x, Center_z));
                LinkedListNode<float[,]> maps_node=Maps.First;
                LinkedListNode<Vector2D> center_node=Maps_center.First;
                while (center_node != null)
                {
                    if (Math.Abs((int)Math.Round((P.X - center_node.Value.x) / Unit)) <= Size && Math.Abs((int)Math.Round((P.Z - Center_z) / Unit)) <= Size)
                    {
                        Center_x = (float)center_node.Value.x;
                        Center_z = (float)center_node.Value.y;
                        Map = maps_node.Value;
                        MapUpdate(P);
                        return;
                    }
                    center_node = center_node.Next;
                    maps_node = maps_node.Next;
                }
                Map = new float[Size * 2 + 1, Size * 2 + 1];
                Reliability = new float[Size * 2 + 1, Size * 2 + 1];
                Center_x = P.X;
                Center_z = P.Z;
                MapUpdate(P);
                return;
            }
            Map[x + Size, z + Size] = P.Y;
            Reliability[x + Size, z + Size] = 1;
        }
        public void MapUpdate(comlink.Position Position,Vector3D Oritation,int[] Edge,CamType Type)
        {
            Matrix tran_M = Matrix.matrix(Position.X, Position.Y, Position.Z, -Oritation.x, -Oritation.y, -Oritation.z);
            int from_x =(int)Math.Round(( Position.X-Center_x)/ Unit), from_z = (int)Math.Round( (Position.Z-Center_z)/Unit);
            if (Math.Abs(from_x) > Size || Math.Abs(from_z) > Size)
                MapUpdate(Position);  
            for (int i = 1; i < Edge.Length-1; ++i)
            {
                if (ImageCoo[i, Edge[i]] == null)
                    ImageCoo[i, Edge[i]] = ImageAnalyseTech.BasicVision(new Vector2D(i, Edge[i]), Type);
                Matrix M_to = Matrix.MatrixMulti(tran_M, new Matrix(-ImageCoo[i, Edge[i]].x, ImageCoo[i, Edge[i]].y, -ImageCoo[i, Edge[i]].z, 1));
                Vector3D V_to=new Vector3D (M_to.data[0,0],M_to.data[1,0],M_to.data[2,0]);
                int to_x = (int)Math.Round((V_to.x - Center_x) / Unit), to_z = (int)Math.Round((V_to.z - Center_z) / Unit);
                if (Math.Abs(to_x - from_x) > Math.Abs(to_z - from_z))
                {
                    float deta_y = (float)(V_to.y - Position.Y) / (to_x - from_x);
                    float deta_z = 1.0f*(to_z - from_z) / (to_x - from_x);
                    for (int j = from_x; j != to_x && j <= Size && j >= -Size; j += Math.Sign(to_x - from_x))
                    {
                        int z = (int)Math.Round(from_z + 1.0 * (j - from_x) * deta_z);
                        if (z > Size || z < -Size)
                            break;
                        if (Reliability[j + Size, z + Size] >= 1)
                            continue;
                        //+Math.Pow(y-Position.Y,2)
                        float y = Position.Y + deta_y * (j - from_x);
                        float relie = (float)(1.0 / (Math.Pow(Math.Pow((j - from_x) * Unit, 2) + Math.Pow((z - from_z) * Unit, 2), 0.25) + 1));
                        Map[j + Size, z + Size] = y * (relie / (relie + Reliability[j + Size, z + Size])) + Map[j + Size, z + Size] * (Reliability[j + Size, z + Size] / (relie + Reliability[j + Size, z + Size]));
                        Reliability[j + Size, z + Size] = 1 - (1 - Reliability[j + Size, z + Size]) * (1 - relie);
                    }
                }
                else
                {
                    float deta_y = (float)(V_to.y - Position.Y) / (to_z - from_z);
                    float deta_x = 1.0f*(to_x - from_x) / (to_z - from_z);
                    for (int j = from_z; j != to_z && j <= Size && j >= -Size; j += Math.Sign(to_z - from_z))
                    {
                        int x = (int)Math.Round(from_x + 1.0 * (j - from_z) * deta_x);
                        if (x > Size || x < -Size)
                            break;
                        if (Reliability[x + Size, j + Size] >= 1)
                            continue;
                        //+Math.Pow(y-Position.Y,2)
                        float y = Position.Y + deta_y * (j - from_z);
                        float relie = (float)(1.0 / (Math.Sqrt(Math.Pow((j - from_z) * Unit, 2) + Math.Pow((x - from_x) * Unit, 2)) + 1));
                        Map[x + Size, j + Size] = y * (relie / (relie + Reliability[x + Size, j + Size])) + Map[x + Size, j + Size] * (Reliability[x + Size, j + Size] / (relie + Reliability[x + Size, j + Size]));
                        Reliability[x + Size, j + Size] = 1 - (1 - Reliability[x + Size, j + Size]) * (1 - relie);
                    }
                }
            }
            if (DateTime.Now.Subtract(UpdateTime).TotalSeconds > 60)
            {
                UpdateTime = DateTime.Now;
                //CommonLib.CommonLib.DrawImage(Map);
                //CommonLib.CommonLib.DrawImage(Reliability);
            }
        }
        static double Front_P = 0.5;
        static double Left_P = 0.7;//0.2+0.5
        static double Right_P = 0.9;//..
        static double Back_P=1;//...
        static double MinLenth = 2;
        static double MaxLenth = 10;
        static int G_times = 20;
        static Random random;
        public static int[] dx8D = { 1, 0, 0, -1, 1, 1, -1, -1 };
        public static int[] dy8D = { 0, 1, -1, 0, 1, -1, 1, -1 };
        public Vector3D PlanMidPoint(Vector3D Position, Vector3D Goal)
        {
            //SortedList<double, Vector3D> sortedlist = new SortedList<double, Vector3D>();
            double max = -1;
            Vector3D res=null;
            for (int i = 0; i < G_times; ++i)
            {
                double dir_P = random.NextDouble();

                double dir_A = random.NextDouble() * Math.PI / 2 - Math.PI / 4;
                if (dir_P < Front_P)
                    dir_A += 0;
                else if (dir_P < Left_P)
                    dir_A += Math.PI / 4;
                else if (dir_P < Right_P)
                    dir_A += Math.PI / 2;
                else
                    dir_A += Math.PI * 3 / 4;
                double sin = Math.Sin(dir_A), cos = Math.Cos(dir_A);

                double len = Vector3D.Distance2D(Position, Goal);
                Vector2D diretion = new Vector2D((Goal.x - Position.x) / len, (Goal.z - Position.z) / len);
                double dir_L = random.NextDouble() * (MaxLenth - MinLenth) + MinLenth;
                Vector3D MidPoint = new Vector3D((cos * diretion.x - sin * diretion.y) * dir_L + Position.x,0, (sin * diretion.x + cos * diretion.y) * dir_L + Position.z);
                int mid_x = (int)Math.Round((MidPoint.x - Center_x) / Unit), mid_z = (int)Math.Round((MidPoint.z - Center_z) / Unit);
                if (Math.Abs(mid_x) > Size || Math.Abs(mid_z) > Size)
                    continue;
                MidPoint.y = Map[mid_x + Size, mid_z + Size];
                int from_x = (int)Math.Round((Position.x - Center_x) / Unit), from_z = (int)Math.Round((Position.z - Center_z) / Unit);
                double grads = 0, reliability = 0;
                if (Math.Abs(mid_x - from_x) > Math.Abs(mid_z - from_z))
                {
                    double deta_z = 1.0*(mid_z - from_z) / (mid_x - from_x);
                    for (int j = from_x; j != mid_x; j += Math.Sign(mid_x - from_x))
                    {
                        int z = (int)Math.Round((j - from_x) * deta_z + from_z);
                        for (int k = 0; k < 8; ++k)
                        {
                            if (Math.Abs(j + dx8D[k]) > Size || Math.Abs(z + dy8D[k]) > Size)
                                continue;
                            grads = Math.Max(grads, Math.Abs(Map[j + Size, z + Size] - Map[j + dx8D[k] + Size, z + dy8D[k] + Size]));
                        }      
                        reliability += Reliability[j + Size, z + Size];
                    }
                    reliability /= Math.Abs(mid_x - from_x);
                }
                else
                {
                    double deta_x = 1.0 * (mid_x - from_x) / (mid_z - from_z);
                    for (int j = from_z; j != mid_z; j += Math.Sign(mid_z - from_z))
                    {
                        int x = (int)Math.Round((j - from_z) * deta_x + from_x);
                        for (int k = 0; k < 8; ++k)
                        {
                            if (Math.Abs(j + dy8D[k]) > Size || Math.Abs(x + dx8D[k]) > Size)
                                continue;
                            grads = Math.Max(grads, Math.Abs(Map[x + Size, j + Size] - Map[x + dx8D[k] + Size, j + dy8D[k] + Size]));
                        }
                        reliability += Reliability[x + Size, j + Size];
                    }
                    reliability /= Math.Abs(mid_z - from_z);
                }
                double score = reliability / (grads/Unit + 0.01) / ((Goal.y - MidPoint.y) / Math.Sqrt(Math.Pow(Goal.x - MidPoint.x, 2) + Math.Pow(Goal.z - MidPoint.z, 2))+0.01); //
                score /= (Vector3D.Distance(MidPoint, Position) + Vector3D.Distance(MidPoint, Goal));
                //sortedlist.Add(score, MidPoint);
                if(max<score)
                {
                    max=score;
                    res=MidPoint;
                }
            }
            return res;
        }

        public int Suggust(Vector3D Position, Vector2D Oritation)
        {
            int from_x = (int)Math.Round((Position.x - Center_x) / Unit), from_z = (int)Math.Round((Position.z - Center_z) / Unit);
            double p_score = 0, u_score = 0;
            if (Math.Abs(Oritation.x) > Math.Abs(Oritation.y))
            {
                int last_x = from_x, last_z = from_z;
                double deta_z = Oritation.y / Math.Abs(Oritation.x);
                for (int j = 0; j < 10; j++)
                {
                    int z1 = (int)Math.Round(from_z + (j + 1) * deta_z);
                    int x1 = from_x + Math.Sign(Oritation.x);
                    if (z1 > Size || z1 < -Size || x1 > Size || x1 < -Size)
                    {
                        p_score = p_score * 10 / (j + 1);
                        break;
                    }
                    p_score += Math.Abs(Map[x1 + Size, z1 + Size] - Map[last_x + Size, last_z + Size]);
                    last_x = x1; last_z = z1;
                    if (j == 9)
                        p_score /= 10;
                }
                last_x = from_x; last_z = from_z;
                for (int j = 0; j < 10; j++)
                {
                    int z2 = (int)Math.Round(from_z - (j + 1) * deta_z);
                    int x2 = from_x - 1 * Math.Sign(Oritation.x);
                    if (z2 > Size || z2 < -Size || x2 > Size || x2 < -Size)
                    {
                        u_score = u_score * 10 / (j + 1);
                        break;
                    }
                    u_score += Math.Abs(Map[x2 + Size, z2 + Size] - Map[last_x + Size, last_z + Size]);
                    last_x = x2; last_z = z2;
                    if (j == 9)
                        u_score /= 10;
                }
                return p_score > u_score ? -1 : 1;
            }
            else
            {
                int last_x = from_x, last_z = from_z;
                double deta_x = Oritation.x / Math.Abs(Oritation.y);
                for (int j = 0; j < 10; j++)
                {
                    int x1 = (int)Math.Round(from_x + (j + 1) * deta_x);
                    int z1 = from_z + 1 * Math.Sign(Oritation.y);
                    if (z1 > Size || z1 < -Size || x1 > Size || x1 < -Size)
                    {
                        p_score = p_score * 10 / (j + 1);
                        break;
                    }
                    p_score += Math.Abs(Map[x1 + Size, z1 + Size] - Map[last_x + Size, last_z + Size]);
                    last_x = x1; last_z = z1;
                    if (j == 9)
                        p_score /= 10;
                }
                last_x = from_x; last_z = from_z;
                for (int j = 0; j < 10; j++)
                {
                    int x2 = (int)Math.Round(from_x - (j + 1) * deta_x);
                    int z2 = from_z - 1 * Math.Sign(Oritation.y);
                    if (z2 > Size || z2 < -Size || x2 > Size || x2 < -Size)
                    {
                        u_score = u_score * 10 / (j + 1);
                        break;
                    }
                    u_score += Math.Abs(Map[x2 + Size, z2 + Size] - Map[last_x + Size, last_z + Size]);
                    last_x = x2; last_z = z2;
                    if (j == 9)
                        u_score /= 10;
                }
                return p_score > u_score ? -1 : 1;
            }
        }
    }

    public class VisionData
    {
        public comlink.Position Position { get; set; }
        public double Score { get; set; }
        public stereo.RockType Type { get; set; }
        public VisionData(comlink.Position position, double score, stereo.RockType type)
        {
            Position = position;
            Score = score;
            Type = type;
        }
    }

    public class VisionMemory
    {
        public LinkedList<VisionData> Database { get; set; }
        public LinkedList<VisionData> Memory { get; set; }
        public comlink.Position Mission_P { get; set; }
        public double Mission_R { get; set; }
        public stereo.RockType Mission_T { get; set; }
        public VisionMemory() { Database = new LinkedList<VisionData>(); Mission_P = new comlink.Position(); }
        public void Newmission(Mission mission)
        {
            Mission_P = mission.place;
            Mission_R = mission.radix;
            if (mission.type == mission_type.ExamineRock)
                Mission_T = stereo.RockType.Rock;
            else if (mission.type == mission_type.ReachShield)
                Mission_T = stereo.RockType.Shield;

            Memory = new LinkedList<VisionData>();
            foreach (VisionData data in Database)
            {
                if (MarsChallengerService.Distance(Mission_P, data.Position) < Mission_R && data.Type == Mission_T)
                    Memory.AddLast(data);
            }
        }
        public void Update(VisionData data)
        {
            Database.AddLast(data);
            if (MarsChallengerService.Distance(Mission_P, data.Position) < Mission_R && data.Type == Mission_T)
                Memory.AddLast(data);
        }
        public comlink.Position Get()
        {
            double min_s = double.MaxValue;
            VisionData best=null;
            foreach (VisionData data in Memory)
            {
                if (data.Score < min_s)
                {
                    min_s = data.Score;
                    best = data;
                }
            }
            if (best == null)
                return null;
            comlink.Position res = best.Position;
            Memory.Remove(best);
            return res;
        }
    }

    [DataContract]
    public class SensorInterval
    {
        public DateTime LatestComLink;
        public int ComLinkInterval;            //ms
        public DateTime LatestFroCam;
        public int FroCamInterval;
        public DateTime LatestRearCam;
        public int RearCamInterval;
        public DateTime LatestVision;
        public int VisionInterval;

        public SensorInterval()
        {
            ComLinkInterval = 1000; LatestComLink=DateTime.Now;
            FroCamInterval = 500; LatestFroCam=DateTime.Now;
            RearCamInterval = 500; LatestRearCam=DateTime.Now;
            VisionInterval = 500; LatestVision = DateTime.Now;
        }
    }

    public enum ControlLevel
    {
        Basic=0x01,
        Eyes=0x02,
        Brain=0x04,
        Immergent=0x08,
        FromEarth=0x10,
    }
    [DataContract]
    public class DriveState
    {
        public double Left { get; set; }
        public double Right { get; set; }
        public DateTime LastTimeChange { get; set; }
        public ControlLevel Level{ get; set; }
        public object Locker { get; set; }
        public DriveState() { Left = 0; Right = 0; Level = ControlLevel.Basic; Locker = new object(); Solution = DefSol; }
        public int Solution;
        static int DefSol = 7;
    }
    
    [DataContract]
    public class HeadState
    {
        public HeadState(){CLevel=ControlLevel.Basic;}
        public DateTime LastChange { get; set; }
        public float Pan { get; set; }
        public float Tilt { get; set; }
        public ControlLevel CLevel { get; set; }
    }
 

    /// <summary>
    /// Mars Rover States
    /// </summary>
    [DataContract]
    public enum RoverState
    {
        Test,
        /// <summary>
        /// Intitialize the robot
        /// </summary>
        Initialize,

        /// <summary>
        /// Basic navigation
        /// </summary>
        Navigate,
        
        /// <summary>
        /// Mission is over
        /// </summary>
        EndMission,
    }

    public enum mission_state
    {
        success,
        undergoing,
        midpoint,
        failure,
    }
    public enum mission_type
    {
        ReachArea,
        ExamineRock,
        ReachShield,
        MidPoint,
    }
    [DataContract]
    public class Mission
    {
        public string name{get;set;}
        public comlink.Position position { get; set; }
        public double radix { get; set; }
        public mission_state state { get; set; }
        public mission_type type { get; set; }
        public double score { get; set; }
        public Mission() { }
        public comlink.Position place { get; set; }
        public Mission(mission_type T, string N, comlink.Position P)
        {
            state = mission_state.undergoing;
            type = T;
            name = N;
            position = P;
            score = double.MaxValue;
            place = position;
        }
        public Mission(mission_type T, string N, comlink.Position P,double R)
        {
            state = mission_state.undergoing;
            type = T;
            name = N;
            position = P;
            radix=R;
            score = 1000;
            place = position;
        }
    }


    #region Service Operations

    [ServicePort]
    public class MarsChallengerOperations : PortSet<DsspDefaultLookup, DsspDefaultDrop, Get, Initialize, HttpGet>
    {
    }

    public class Get : Get<GetRequestType, PortSet<MarsChallengerState, Fault>>
    {
        public Get() : base() { }
        public Get(GetRequestType body) : base(body) { }
        public Get(GetRequestType body, PortSet<MarsChallengerState, Fault> responsePort) : base(body, responsePort) { }
    }

    public class Initialize : Update<InitializeRequest, PortSet<DefaultUpdateResponseType, Fault>>
    {
        public Initialize() : base() { }
        public Initialize(InitializeRequest body) : base(body) { }
        public Initialize(InitializeRequest body, PortSet<DefaultUpdateResponseType, Fault> responsePort) : base(body, responsePort) { }
    }

    [DataContract]
    public class InitializeRequest
    {
    }

    #endregion
}
