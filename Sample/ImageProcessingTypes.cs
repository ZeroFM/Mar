using System;
using Microsoft.Ccr.Core;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;
using W3C.Soap;

namespace Microsoft.Robotics.RoboChamps.MarsChallenger.ImageProcessing
{
    public sealed class Contract
    {
        [DataMember]
        public const String Identifier = "http://www.robochamps.com/2009/02/marschallenger/imageprocessing.html";
    }

    [DataContract]
    public class ImageProcessingState
    {
        [DataMember]
        public DateTime LastProcessing { get; set; }
    }

    /// <summary>
    /// Class that contains result for image processing
    /// </summary>
    [DataContract]
    public class ImageResult
    {
        [DataMember]
        public DateTime Timestamp { get; set; }
        [DataMember]
        public target[] Avoid { get; set; }
        [DataMember]
        public int[] Edge { get; set; }
        // You could add what you want in image result
    }
    [DataContract]
    public class target
    {
        public target(int X, int Y) { x = X; y = Y; }
        public target(int X, int Y, int Max_x, int Max_y, int Min_x, int Min_y,double Tan_left,double Tan_right) 
        {
            x = X; y = Y;
            max_x = Max_x; max_y = Max_y;
            min_x = Min_x; min_y = Min_y;
            tan_left = Tan_left; tan_right = Tan_right;
        }
        public target() { }
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
        public double tan_left;
        [DataMember]
        public double tan_right;
    }

    [ServicePort]
    public class ImageProcessingOperations : PortSet<DsspDefaultLookup, DsspDefaultDrop, Subscribe, ProcessImage>
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

    public class Subscribe : Subscribe<SubscribeRequestType, PortSet<SubscribeResponseType, Fault>>
    {
        public Subscribe() : base() { }
        public Subscribe(SubscribeRequestType body) : base(body) { }
        public Subscribe(SubscribeRequestType body, PortSet<SubscribeResponseType, Fault> responsePort) : base(body, responsePort) { }
    }

    public class ProcessImage : Update<ImageResult, PortSet<DefaultUpdateResponseType, Fault>>
    {
        public ProcessImage() : base() { }
        public ProcessImage(ImageResult body) : base(body) { }
        public ProcessImage(ImageResult body, PortSet<DefaultUpdateResponseType, Fault> responsePort) : base(body, responsePort) { }
    }
}