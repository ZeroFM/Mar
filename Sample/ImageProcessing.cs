using System;
using Microsoft.Ccr.Core;
using System.Collections.Generic;
using System.ComponentModel;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;

using submgr = Microsoft.Dss.Services.SubscriptionManager;
using webcam = Microsoft.Robotics.Services.WebCam.Proxy;

namespace Microsoft.Robotics.RoboChamps.MarsChallenger.ImageProcessing
{
    /// <summary>
    /// Processes webcam image and sends notifications with result
    /// </summary>
    [DisplayName("Mars Image Processor")]
    [Description("The Mars Challenger Basic Camara Image Processing Service")]
    [Contract(Contract.Identifier)]
    public class ImageProcessingService : DsspServiceBase
    {

        [ServiceState]
        private ImageProcessingState _state = new ImageProcessingState();

        [ServicePort("/ImageProcessingService", AllowMultipleInstances = true)]
        private ImageProcessingOperations _mainPort = new ImageProcessingOperations();

        #region Partners

        [Partner("Camera", Contract = webcam.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private webcam.WebCamOperations _camPort = new webcam.WebCamOperations();
        private webcam.WebCamOperations _camNotify = new webcam.WebCamOperations();

        [SubscriptionManagerPartner]
        private submgr.SubscriptionManagerPort _submgrPort = new submgr.SubscriptionManagerPort();

        #endregion

        /// <summary>
        /// Dictionary that hold image processor, depending service name
        /// </summary>
        private Dictionary<string, IImageProcessor> _processors =
            new Dictionary<string, IImageProcessor>(new ProcessorDictionaryComparer());

        /// <summary>
        /// Current service processor
        /// </summary>
        private IImageProcessor _processor = null;

        /// <summary>
        /// Initializes the service.
        /// Associates service name and image processor.
        /// </summary>
        /// <param name="creationPort"></param>
        public ImageProcessingService(DsspServiceCreationPort creationPort) :
            base(creationPort)
        {
            _processors.Add("frontcamprocessor", new BasicCamProcessor());
            _processors.Add("rearcamprocessor", new BasicCamProcessor());
        }

        /// <summary>
        /// Service Start
        /// </summary>
        protected override void Start()
        {
            base.Start();

            // Gets the processor associated to this service
            _processors.TryGetValue(ServiceInfo.Service, out _processor);

            // Initialization
            _state.LastProcessing = DateTime.MinValue;

            // Subscribe to webcam notification
            MainPortInterleave.CombineWith(Arbiter.Interleave(
                new TeardownReceiverGroup(),
                new ExclusiveReceiverGroup
                (
                    Arbiter.ReceiveWithIterator<webcam.UpdateFrame>(true, _camNotify, UpdateFrameHandler)
                ),
                new ConcurrentReceiverGroup()));

            _camPort.Subscribe(_camNotify, typeof(webcam.UpdateFrame));
        }

        /// <summary>
        /// Handler for Subscribe message
        /// </summary>
        [ServiceHandler(ServiceHandlerBehavior.Concurrent)]
        public IEnumerator<ITask> SubscribeHandler(Subscribe subscribe)
        {
            yield return Arbiter.Choice(
                SubscribeHelper(_submgrPort, subscribe, subscribe.ResponsePort),
                (success) => { },
                (error) => LogError(error));

        }

        /// <summary>
        /// Handler for ProcessImage message.
        /// It's called internally and just sends notifications.
        /// </summary>
        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public IEnumerator<ITask> ProcessImageHandler(ProcessImage process)
        {
            process.ResponsePort.Post(DefaultUpdateResponseType.Instance);
            SendNotification<ProcessImage>(_submgrPort, process);
            yield break;
        }

        /// <summary>
        /// Handler for webcam notifications
        /// </summary>
        protected IEnumerator<ITask> UpdateFrameHandler(webcam.UpdateFrame update)
        {
            // Discards old frame
            if (update.Body.TimeStamp.CompareTo(_state.LastProcessing) < 0)
            {
                yield break;
            }

            _state.LastProcessing = update.Body.TimeStamp;

            // Queries a new frame
            webcam.QueryFrameResponse queryResponse = null;
            yield return Arbiter.Choice(_camPort.QueryFrame(), (success) => queryResponse = success, LogError);

            if (_processor != null && queryResponse != null)
            {
                // Calls the image processor and post the result on the main port (to send notifications)
                _mainPort.Post(new ProcessImage(_processor.Process(queryResponse)));
            }
        }

    }
    #region Helpers

    /// <summary>
    /// Equality comparer for processor dictionary.
    /// Compares two URI with the final part
    /// </summary>
    internal class ProcessorDictionaryComparer : IEqualityComparer<string>
    {
        #region IEqualityComparer<string> Members

        public bool Equals(string x, string y)
        {
            return Normalize(x) == Normalize(y);
        }

        public int GetHashCode(string obj)
        {
            return Normalize(obj).GetHashCode();
        }

        #endregion

        private string Normalize(string s)
        {
            if (s == null)
            {
                return s;
            }
            int i = s.LastIndexOf('/');
            if (i != -1)
            {
                s = s.Substring(i + 1);
            }
            return s;
        }
    }

    #endregion

    #region Processors

    /// <summary>
    /// Processor interface
    /// </summary>
    internal interface IImageProcessor
    {
        /// <summary>
        /// Returns an image result after analyzing a query frame response
        /// </summary>
        ImageResult Process(webcam.QueryFrameResponse image);
    }

    /// <summary>
    /// Processor for rear and front cameras
    /// </summary>
    internal class BasicCamProcessor : IImageProcessor
    {
        public static int[] dx = { 1, 0, 0, -1 };
        public static int[] dy = { 0, 1, -1, 0 };
        public static int[] dx8D = { 1, 0, 0, -1, 1, 1, -1, -1 };
        public static int[] dy8D = { 0, 1, -1, 0, 1, -1, 1, -1 };
        const double LightThres = 180;
        /// <summary>
        /// process for basic camara.
        /// it use gray diffence mathod.
        /// </summary>
        /// <param name="image"></param>
        /// <returns></returns>
        public ImageResult Process(Microsoft.Robotics.Services.WebCam.Proxy.QueryFrameResponse image)
        {
            ImageResult result = new ImageResult();
            result.Timestamp = DateTime.Now;

            int offset = 0;
            double[,] gray = new double[image.Size.Width, image.Size.Height];
            double[,] DIS = new double[image.Size.Width, image.Size.Height];

            for (int y = 0; y < image.Size.Height; ++y)
            {
                offset = y * image.Size.Width * 3;
                for (int x = 0; x < image.Size.Width; ++x)
                {
                    int r, g, b;
                    b = image.Frame[offset++];
                    g = image.Frame[offset++];
                    r = image.Frame[offset++];
                    gray[x, y] = 0.299 * r + 0.587 * g + 0.114 * b;
                    if ((r + g + b) / 3 > LightThres)
                        DIS[x, y] = -1;
                }
            }

            double sum = 0;
            int count = 0;

            for (int y = 1; y < image.Size.Height - 1; ++y)
                for (int x = 1; x < image.Size.Width - 1; ++x)
                {
                    if (DIS[x, y] == -1 || (y >= 97 && (x <= 17 || x >= 110)))         //dig out sky and wheels
                        continue;
                    bool EdgeOfSky = false;
                    for (int k = 0; k < 8; ++k)
                        if (DIS[x + dx8D[k], y + dy8D[k]] == -1)
                            EdgeOfSky = true;
                    if (EdgeOfSky)
                        continue;
                    DIS[x, y] = Math.Abs(gray[x - 1, y - 1] - gray[x + 1, y - 1]) + Math.Abs(gray[x - 1, y - 1] - gray[x + 1, y]) + Math.Abs(gray[x - 1, y - 1] - gray[x + 1, y + 1]) + Math.Abs(gray[x - 1, y - 1] - gray[x, y + 1]) + Math.Abs(gray[x - 1, y - 1] - gray[x - 1, y + 1]);
                    DIS[x, y] += Math.Abs(gray[x, y - 1] - gray[x - 1, y]) + Math.Abs(gray[x, y - 1] - gray[x + 1, y]) + Math.Abs(gray[x, y - 1] - gray[x + 1, y + 1]) + Math.Abs(gray[x, y - 1] - gray[x, y + 1]) + Math.Abs(gray[x, y - 1] - gray[x - 1, y + 1]);
                    DIS[x, y] += Math.Abs(gray[x + 1, y - 1] - gray[x - 1, y]) + Math.Abs(gray[x + 1, y - 1] - gray[x + 1, y]) + Math.Abs(gray[x + 1, y - 1] - gray[x + 1, y + 1]) + Math.Abs(gray[x + 1, y - 1] - gray[x, y + 1]) + Math.Abs(gray[x + 1, y - 1] - gray[x - 1, y + 1]);
                    DIS[x, y] += Math.Abs(gray[x - 1, y] - gray[x + 1, y]) + Math.Abs(gray[x - 1, y] - gray[x + 1, y + 1]) + Math.Abs(gray[x, y + 1] - gray[x + 1, y]) + Math.Abs(gray[x - 1, y + 1] - gray[x + 1, y + 1]);
                    sum += DIS[x, y];
                    count++;
                }

            double arg = sum / count;

            Queue<target> ts = new Queue<target>();//test

            Queue<target> avoid = new Queue<target>();
            int[] edge = new int[image.Size.Width];
            for (int x = 1; x < image.Size.Width - 1; ++x)
            {
                for (int y = 1; y < image.Size.Width - 1; ++y)//
                {
                    if (DIS[x, y] > arg * 6)//1.8
                    {
                        bool need_avoid = false;
                        int sum_x = 0, sum_y = 0, pixel_count = 0, max_x = 0, max_y = 0, min_x = image.Size.Width - 1, min_y = image.Size.Height - 1;
                        double tan_left = double.MaxValue, tan_right = double.MaxValue;
                        Queue<int> Q_x = new Queue<int>();
                        Queue<int> Q_y = new Queue<int>();
                        Q_x.Enqueue(x);
                        Q_y.Enqueue(y);
                        while (Q_x.Count != 0)
                        {
                            int index_x = Q_x.Dequeue(), index_y = Q_y.Dequeue();
                            sum_x += index_x; sum_y += index_y; pixel_count++;
                            max_x = Math.Max(max_x, index_x); max_y = Math.Max(max_y, index_y);
                            min_x = Math.Min(min_x, index_x); min_y = Math.Min(min_y, index_y);
                            if (index_y < 95)//
                            {
                                tan_left = Math.Min(tan_left, 1.0 * (128 - index_y) / (index_x+1));
                                tan_right = Math.Min(tan_right, 1.0 * (128 - index_y) / (128 - index_x));
                            }
                            if (!need_avoid && index_y >= 68 && index_x - 98 <= index_y - 70 && index_x - 10 >= 95 - index_y)
                                need_avoid = true;
                            for (int i = 0; i < 4; ++i)
                            {
                                if (index_y + dy[i] >= 35 && DIS[index_x + dx[i], index_y + dy[i]] > arg * 6)
                                {
                                    Q_x.Enqueue(index_x + dx[i]); Q_y.Enqueue(index_y + dy[i]);
                                    DIS[index_x + dx[i], index_y + dy[i]] = -1;
                                }
                            }
                        }
                        if (pixel_count > 15)
                        {
                            ts.Enqueue(new target(sum_x / pixel_count, sum_y / pixel_count, max_x, max_y, min_x, min_y, tan_left, tan_right));
                            if (need_avoid)
                                avoid.Enqueue(new target(sum_x / pixel_count, sum_y / pixel_count, max_x, max_y, min_x, min_y, tan_left, tan_right));
                        }
                    }//if

                    if (DIS[x, y] > arg * 4 || DIS[x,y]==-1)
                        edge[x] = y;
                }
                edge[x] = Math.Max(edge[x], 35);
            }

            result.Avoid = avoid.ToArray();
            result.Edge = edge;
            //if (result.Avoid.Length != 0)
               // CommonLib.CommonLib.DrawImage(image, result.Avoid, result.Edge);
            return result;
        }

    }

    #endregion


}