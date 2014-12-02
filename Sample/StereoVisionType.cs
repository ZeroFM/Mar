using System;
using Microsoft.Ccr.Core;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;
using W3C.Soap;

using Microsoft.Robotics.RoboChamps.MarsChallenger.CommonLib;
using webcam = Microsoft.Robotics.Services.WebCam.Proxy;


namespace Microsoft.Robotics.RoboChamps.MarsChallenger.StereoVision
{
    public sealed class Contract
    {
        [DataMember]
        public const String Identifier = "http://www.robochamps.com/2009/02/marschallenger/stereovision.html";
    }

    [DataContract]
    public class StereoVisionState
    {
        [DataMember]
        public DateTime LastPanProcessing { get; set; }
        [DataMember]
        public DateTime LastNavProcessing { get; set; }
        [DataMember]
        public Search SearchFor { get; set; }

        public webcam.QueryFrameResponse PanFrame { get; set; }
        public webcam.QueryFrameResponse NavFrame { get; set; }

    }
    public enum Search
    {
        rock,
        edge,
        shield,
    }
    [DataContract]
    public class Mode
    {
        [DataMember]
        public Search mode { get; set; }
        public Mode(Search s) { mode = s; }
        public Mode() { }
    }

    /// <summary>
    /// Class that contains result for image processing
    /// </summary>
    [DataContract]
    public class VisionResult
    {
        //add.......
        [DataMember]
        public Rock[] Rocks { get; set; }
        public Vector2D Center { get; set; }
        [DataMember]
        public int Center_x { get; set; }
        [DataMember]
        public int Center_y { get; set; }
        [DataMember]
        public Blob[] PanBlobs { get; set; }
        [DataMember]
        public Blob[] NavBlobs { get; set; }
        [DataMember]
        public DateTime TimeStamp { get; set; }
    }

    public enum RockType
    {
        Rock,
        Shield,
        Edge,
    }
    [DataContract]
    public class Rock
    {
        public Rock() { }
        public Rock(Vector3D position,double score,int panblobnum,int navblobnum,RockType type){
            X = position.x; Y = position.y; Z = position.z; Score = score; PanBlob_num = panblobnum; NavBlob_num = navblobnum; Type = type;
        }
        [DataMember]
        public double X { get; set; }
        [DataMember]
        public double Y { get; set; }
        [DataMember]
        public double Z { get; set; }
        [DataMember]
        public double Score { get; set; }
        [DataMember]
        public int PanBlob_num { get; set; }
        [DataMember]
        public int NavBlob_num { get; set; }
        [DataMember]
        public RockType Type { get; set; }
    }

    [DataContract]
    public class Blob
    {
        public Blob(int X, int Y) { x = X; y = Y; }
        public Blob(int X, int Y, int Max_x, int Max_y, int Min_x, int Min_y,int Sum_B,int Sum_G,int Sum_R,int Pexel_num)
        {
            x = X; y = Y;
            max_x = Max_x; max_y = Max_y;
            min_x = Min_x; min_y = Min_y;
            sum_B = Sum_B; sum_G = Sum_G; sum_R = Sum_R;
            pexel_num = Pexel_num;
        }
        public Blob() { }
        [DataMember]
        public int x;
        [DataMember]
        public int y;
        [DataMember]
        public int max_x;
        [DataMember]
        public int max_y;
        [DataMember]
        public int min_x;
        [DataMember]
        public int min_y;
        [DataMember]
        public int sum_R;
        [DataMember]
        public int sum_B;
        [DataMember]
        public int sum_G;
        [DataMember]
        public int pexel_num;

        [DataMember]
        public double Rock_Pro;
        [DataMember]
        public double Shield_Pro;
    }

    [ServicePort]
    public class StereoVisionOperations : PortSet<DsspDefaultLookup, DsspDefaultDrop, Subscribe, ProcessVision, ModeChange>
    {
        public virtual PortSet<SubscribeResponseType, Fault> Subscribe(IPort notificationPort, params Type[] types)
        {
            SubscribeRequestType body = new SubscribeRequestType();
            Subscribe operation = new Subscribe(body);
            operation.NotificationPort = notificationPort;
            if ((types != null))
            {
                body.TypeFilter = new string[types.Length];
                for (int index = 0; (index < types.Length); index = (index + 1))
                {
                    body.TypeFilter[index] = DsspServiceBase.GetTypeFilterDescription(types[index]);
                }
            }
            this.Post(operation);
            return operation.ResponsePort;
        }

    }

    public class ModeChange : Update<Mode, PortSet<DefaultUpdateResponseType, Fault>>
    {
        public ModeChange() : base() { }
        public ModeChange(Mode body) : base(body) { }
        public ModeChange(Mode body, PortSet<DefaultUpdateResponseType, Fault> responsePort) : base(body, responsePort) { }

    }

    public class Subscribe : Subscribe<SubscribeRequestType, PortSet<SubscribeResponseType, Fault>>
    {
        public Subscribe() : base() { }
        public Subscribe(SubscribeRequestType body) : base(body) { }
        public Subscribe(SubscribeRequestType body, PortSet<SubscribeResponseType, Fault> responsePort) : base(body, responsePort) { }
    }

    public class ProcessVision : Update<VisionResult, PortSet<DefaultUpdateResponseType, Fault>>
    {
        public ProcessVision() : base() { }
        public ProcessVision(VisionResult body) : base(body) { }
        public ProcessVision(VisionResult body, PortSet<DefaultUpdateResponseType, Fault> responsePort) : base(body, responsePort) { }
    }
}