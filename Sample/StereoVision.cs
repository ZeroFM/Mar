using System;
using Microsoft.Ccr.Core;
using System.Collections.Generic;
using System.ComponentModel;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;

using submgr = Microsoft.Dss.Services.SubscriptionManager;
using webcam = Microsoft.Robotics.Services.WebCam.Proxy;
using Microsoft.Robotics.RoboChamps.MarsChallenger.CommonLib;

namespace Microsoft.Robotics.RoboChamps.MarsChallenger.StereoVision
{
    /// <summary>
    /// Processes webcam image and sends notifications with result
    /// </summary>
    [DisplayName("Mars StereoVision Processor")]
    [Description("The Mars Challenger StereoVision Processing Service")]
    [Contract(Contract.Identifier)]
    public class StereoVisionService : DsspServiceBase
    {

        [ServiceState]
        private StereoVisionState _state = new StereoVisionState();

        [ServicePort("/StereoVisionService", AllowMultipleInstances = false)]
        private StereoVisionOperations _mainPort = new StereoVisionOperations();

        #region Partners

        [Partner("PanCamera", Contract = webcam.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private webcam.WebCamOperations _PanCamPort = new webcam.WebCamOperations();
        private webcam.WebCamOperations _PanCamNotify = new webcam.WebCamOperations();

        [Partner("NavCamera", Contract = webcam.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private webcam.WebCamOperations _NavCamPort = new webcam.WebCamOperations();
        private webcam.WebCamOperations _NavCamNotify = new webcam.WebCamOperations();

        [SubscriptionManagerPartner]
        private submgr.SubscriptionManagerPort _submgrPort = new submgr.SubscriptionManagerPort();

        #endregion

        /// <summary>
        /// Initializes the service.
        /// Associates service name and image processor.
        /// </summary>
        /// <param name="creationPort"></param>
        public StereoVisionService(DsspServiceCreationPort creationPort) :
            base(creationPort)
        {
        }

        /// <summary>
        /// Service Start
        /// </summary>
        protected override void Start()
        {
            base.Start();

            // Initialization
            _state.LastPanProcessing = DateTime.MinValue;
            _state.LastNavProcessing = DateTime.MinValue;
            _state.SearchFor = Search.rock;

            // Subscribe to webcam notification
            MainPortInterleave.CombineWith(Arbiter.Interleave(
                new TeardownReceiverGroup(),
                new ExclusiveReceiverGroup
                (
                    Arbiter.ReceiveWithIterator<webcam.UpdateFrame>(true, _PanCamNotify, PanUpdateFrameHandler),
                    Arbiter.ReceiveWithIterator<webcam.UpdateFrame>(true, _NavCamNotify, NavUpdateFrameHandler)
                ),
                new ConcurrentReceiverGroup()));

            _PanCamPort.Subscribe(_PanCamNotify, typeof(webcam.UpdateFrame));
            _NavCamPort.Subscribe(_NavCamNotify, typeof(webcam.UpdateFrame));
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
        /// Handler for ProcessVission message.
        /// It's called internally and just sends notifications.
        /// </summary>
        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public IEnumerator<ITask> ProcessVisionHandler(ProcessVision process)
        {
            process.ResponsePort.Post(DefaultUpdateResponseType.Instance);
            SendNotification<ProcessVision>(_submgrPort, process);
            yield break;
        }

        /// <summary>
        /// Handler for ModeChange message.
        /// It Change the Search Mode.Include Rock and Shield Mode, currently
        /// </summary>
        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public IEnumerator<ITask> ModeChangeHandler(ModeChange mode)
        {
            mode.ResponsePort.Post(DefaultUpdateResponseType.Instance);
            _state.SearchFor = mode.Body.mode;
            yield break;
        }

        /// <summary>
        /// Handler for webcam notifications
        /// </summary>
        protected IEnumerator<ITask> PanUpdateFrameHandler(webcam.UpdateFrame update)
        {
            // Discards old frame
            if (update.Body.TimeStamp.CompareTo(_state.LastPanProcessing) < 0)
                yield break;

            // Queries a new frame
            yield return Arbiter.Choice(_PanCamPort.QueryFrame(), (success) => _state.PanFrame = success, LogError);

            if (_state.PanFrame != null && _state.NavFrame != null)
            {
                //synchronize pan and nav
                if (Math.Abs(_state.NavFrame.TimeStamp.Subtract(_state.PanFrame.TimeStamp).TotalMilliseconds) > TimeThres)
                    yield break;

                // Calls the image processor and post the result on the main port (to send notifications)
                _mainPort.Post(new ProcessVision(StereoVisionProcess()));
            }
        }
        protected IEnumerator<ITask> NavUpdateFrameHandler(webcam.UpdateFrame update)
        {
            // Discards old frame
            if (update.Body.TimeStamp.CompareTo(_state.LastNavProcessing) < 0)
                yield break;

            // Queries a new frame
            yield return Arbiter.Choice(_NavCamPort.QueryFrame(), (success) => _state.NavFrame = success, LogError);
            if (_state.NavFrame != null && _state.PanFrame != null)
            {
                //synchronize pan and nav
                if (Math.Abs(_state.NavFrame.TimeStamp.Subtract(_state.PanFrame.TimeStamp).TotalMilliseconds) > TimeThres)
                    yield break;

                 // Calls the image processor and post the result on the main port (to send notifications)
                _mainPort.Post(new ProcessVision(StereoVisionProcess()));
            }
        }

        const int TimeThres = 40;//ms
        const double ProThres = 0.6;
        const int ImageSize = 128;
        public VisionResult StereoVisionProcess()
        {
            VisionResult res = new VisionResult();
            res.TimeStamp = _state.PanFrame.TimeStamp.AddMilliseconds(_state.NavFrame.TimeStamp.Subtract(_state.PanFrame.TimeStamp).TotalMilliseconds / 2);

            if (_state.SearchFor == Search.rock)
            {
                res.NavBlobs = InterestBlobs(_state.NavFrame);
                res.PanBlobs = InterestBlobs(_state.PanFrame);
            }
            else if (_state.SearchFor == Search.shield)
            {
                res.NavBlobs = Saturation(_state.NavFrame);
                res.PanBlobs = Saturation(_state.PanFrame);
            }
            else
                return null;
            double Score;
            res.Center = null; 
            
            Queue<Rock> res_r = new Queue<Rock>();
            for(int i=0;i<res.PanBlobs.Length;++i)
                if (res.PanBlobs[i].Rock_Pro > ProThres || res.PanBlobs[i].Shield_Pro>ProThres)
                {
                    if (res.Center == null)
                    {
                        //global mosaic
                        res.Center = MatchingBlobs(new Blob(64, 64, 127, 127, 0, 0, 0, 0, 0, 0), new Blob(64, 64, 127, 127, 0, 0, 0, 0, 0, 0), out Score);
                        res.Center_x = (int)res.Center.x;
                        res.Center_y = (int)res.Center.y;
                    }
                    int x_nav = (int)res.Center.x;
                    int y_nav = (int)res.Center.y;
                    if (64 != res.PanBlobs[i].y || 64 != res.PanBlobs[i].x)//64 = image.size/2
                    {
                        double len_pan = Math.Sqrt(Math.Pow(res.PanBlobs[i].y-64, 2) + Math.Pow(res.PanBlobs[i].x-64, 2));
                        double sin = (res.PanBlobs[i].y-64) / len_pan;
                        double cos = (res.PanBlobs[i].x-64) / len_pan;
                        double len_nav = len_pan * ImageAnalyseTech.NavCam.FocusLen / ImageAnalyseTech.PanCam.FocusLen;
                        //...
                        x_nav = (int)Math.Round(len_nav * cos + res.Center.x);
                        y_nav = (int)Math.Round(len_nav * sin + res.Center.y);
                        if (x_nav < 0 || x_nav >= ImageSize || y_nav < 0 || y_nav >= ImageSize)
                            continue;
                    }

                    double min_s = double.MaxValue;
                    Vector2D best_match=null;
                    int navnum=0;
                    for (int j = 0; j < res.NavBlobs.Length; ++j)
                    {
                        if (res.NavBlobs[j].max_x >= x_nav && res.NavBlobs[j].min_x <= x_nav && res.NavBlobs[j].max_y >= y_nav && res.NavBlobs[j].min_y <= y_nav)
                        {
                            double score;
                            Blob navblob = res.NavBlobs[j];
                            navblob.x = x_nav;
                            navblob.y = y_nav;
                            //local mosaic 
                            Vector2D match = MatchingBlobs(res.PanBlobs[i], navblob, out score);
                            if (score < min_s)
                            {
                                min_s=score;
                                best_match=match;
                                navnum=j;
                            }
                        }
                    }
                    if (best_match != null)
                    {
                        RockType type = RockType.Rock;
                        if(res.PanBlobs[i].Shield_Pro > ProThres)
                            type = RockType.Shield;
                        res_r.Enqueue(new Rock(ImageAnalyseTech.StereoVision(new Vector2D(res.PanBlobs[i].x, res.PanBlobs[i].y), best_match),min_s,i,navnum,type));
                    }
                }
            if (res_r.Count!= 0)
            {
                res.Rocks=res_r.ToArray();
                //CommonLib.CommonLib.DrawImage(_state.NavFrame, res.NavBlobs);
                //CommonLib.CommonLib.DrawImage(_state.PanFrame, res.PanBlobs);
                //CommonLib.CommonLib.DrawImage(_state.PanFrame, _state.NavFrame, res.Center_x, res.Center_y);
            }
            return res;
        }

        const double SatThres = 10;
        const double ColThres = 90;
        /// <summary>
        /// function use to recogonize shield.
        /// it seperate image by saturation
        /// </summary>
        /// <param name="image"></param>
        /// <returns></returns>
        public Blob[] Saturation(Microsoft.Robotics.Services.WebCam.Proxy.QueryFrameResponse image)
        {
            int offset = 0;
            double[,] saturation = new double[image.Size.Width, image.Size.Height];
            for (int y = 0; y < image.Size.Height; ++y)
            {
                offset = y * image.Size.Width * 3;
                for (int x = 0; x < image.Size.Width; ++x)
                {
                    int r, g, b;
                    b = image.Frame[offset++];
                    g = image.Frame[offset++];
                    r = image.Frame[offset++];
                    int arg_c = (r + b + g) / 3;
                    saturation[x, y] = Math.Sqrt(Math.Pow(r - arg_c, 2) + Math.Pow(g - arg_c, 2) + Math.Pow(b - arg_c, 2));
                }
            }

            Queue<Blob> Blobs = new Queue<Blob>();
            for (int x = 0; x < image.Size.Width; ++x)
            {
                for (int y = 0; y < image.Size.Width; ++y)//
                {
                    if (saturation[x, y] < SatThres)
                    {
                        int sum_x = 0, sum_y = 0, pixel_count = 0, max_x = 0, max_y = 0, min_x = image.Size.Width - 1, min_y = image.Size.Height - 1;
                        int sum_R = 0, sum_G = 0, sum_B = 0;
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
                            int place = index_y * image.Size.Width * 3 + index_x * 3;
                            sum_B += image.Frame[place++];
                            sum_G += image.Frame[place++];
                            sum_R += image.Frame[place];
                            for (int i = 0; i < 4; ++i)
                            {
                                if (index_x + dx[i] >= 0 && index_x + dx[i] < image.Size.Width && index_y + dy[i] >= 0 && index_y + dy[i] < image.Size.Height && saturation[index_x + dx[i], index_y + dy[i]] < SatThres)//
                                {
                                    Q_x.Enqueue(index_x + dx[i]); Q_y.Enqueue(index_y + dy[i]);
                                    saturation[index_x + dx[i], index_y + dy[i]] = 100;        //mark
                                }
                            }
                        }
                        if (pixel_count > 5)
                            Blobs.Enqueue(new Blob(sum_x / pixel_count, sum_y / pixel_count, max_x, max_y, min_x, min_y, sum_B, sum_G, sum_R, pixel_count));
                    }
                }
            }
            Blob[] res = Blobs.ToArray();

            for (int i = 0; i < res.Length; ++i)
            {
                double Pro = 1;
                double Col=(res[i].sum_B + res[i].sum_G + res[i].sum_R) / res[i].pexel_num / 3;
                if (Col > ColThres)
                    Pro *= (1 - (Col - ColThres) / ColThres);
                res[i].Shield_Pro = Pro;
            }

            return res;
        }

        double Hue(int r, int g, int b)
        {
            int max = Math.Max(Math.Max(r, g), b);
            int min = Math.Min(Math.Min(r, g), b);
            if (max == min)
                return 240;
            if (max == r)
                return 1.0 * ((60 * (g - b) / (max - min) + 360) % 360) / 360 * 240;
            if (max == g)
                return 1.0 * (60 * (b - r) / (max - min) + 120) / 360 * 240;
            return 1.0 * (60 * (r - g) / (max - min) + 240) / 360 * 240;
        }
    
        static int[] dx = { 1, 0, 0, -1 };
        static int[] dy = { 0, 1, -1, 0 };
        const double Mark = -101;
        const double StandardRockSaturation = 130;
        const double StandardRockProportion = 2;
        const double StandardRockAxisP = 3;
        const double RockLightThres = 100;
        const double RockHueThres = 20;
        /// <summary>
        /// function use to find rock
        /// it first seperate image by gray difference, then use three characters i have choosed to recognize rock
        /// </summary>
        /// <param name="image"></param>
        /// <returns></returns>
        public Blob[] InterestBlobs(webcam.QueryFrameResponse image)
        {
            int offset = 0;
            int NavWidth = image.Size.Width, NavHeight = image.Size.Height;
            double[,] gray = new double[NavWidth, NavHeight];
            double[,] DIS = new double[NavWidth, NavHeight];

            for (int y = 0; y < NavHeight; ++y)
            {
                offset = y * NavWidth * 3;
                for (int x = 0; x < NavHeight; ++x)
                {
                    int r, g, b;
                    b = image.Frame[offset++];
                    g = image.Frame[offset++];
                    r = image.Frame[offset++];
                    gray[x, y] = 0.299 * r + 0.587 * g + 0.114 * b;

                    if (1.0 * (r + g + b) / 3 > RockLightThres)
                        DIS[x, y] = -1;      //mark
                    if (Hue(r, g, b) > RockHueThres)
                        DIS[x, y] = -1;      //mark
                }
            }

            double sum = 0;
            int count = 0;

            for (int y = 1; y < NavWidth - 1; ++y)
                for (int x = 1; x < NavHeight - 1; ++x)
                {
                    if(DIS[x,y]==-1)        //mark
                        continue;
                
                    //DIS
                    DIS[x, y] = Math.Abs(gray[x - 1, y - 1] - gray[x + 1, y - 1]) + Math.Abs(gray[x - 1, y - 1] - gray[x + 1, y]) + Math.Abs(gray[x - 1, y - 1] - gray[x + 1, y + 1]) + Math.Abs(gray[x - 1, y - 1] - gray[x, y + 1]) + Math.Abs(gray[x - 1, y - 1] - gray[x - 1, y + 1]);
                    DIS[x, y] += Math.Abs(gray[x, y - 1] - gray[x - 1, y]) + Math.Abs(gray[x, y - 1] - gray[x + 1, y]) + Math.Abs(gray[x, y - 1] - gray[x + 1, y + 1]) + Math.Abs(gray[x, y - 1] - gray[x, y + 1]) + Math.Abs(gray[x, y - 1] - gray[x - 1, y + 1]);
                    DIS[x, y] += Math.Abs(gray[x + 1, y - 1] - gray[x - 1, y]) + Math.Abs(gray[x + 1, y - 1] - gray[x + 1, y]) + Math.Abs(gray[x + 1, y - 1] - gray[x + 1, y + 1]) + Math.Abs(gray[x + 1, y - 1] - gray[x, y + 1]) + Math.Abs(gray[x + 1, y - 1] - gray[x - 1, y + 1]);
                    DIS[x, y] += Math.Abs(gray[x - 1, y] - gray[x + 1, y]) + Math.Abs(gray[x - 1, y] - gray[x + 1, y + 1]) + Math.Abs(gray[x, y + 1] - gray[x + 1, y]) + Math.Abs(gray[x - 1, y + 1] - gray[x + 1, y + 1]);
                    sum += DIS[x, y];
                    count++;
                }
            double arg = sum / count;

            Queue<Blob> Blobs = new Queue<Blob>();
            for (int x = 1; x < image.Size.Width - 1; ++x)
            {
                for (int y = 1; y < image.Size.Width - 1; ++y)//
                {
                    if (DIS[x, y] > arg * 7)//
                    {
                        int sum_x = 0, sum_y = 0, pixel_count = 0, max_x = 0, max_y = 0, min_x = image.Size.Width - 1, min_y = image.Size.Height - 1;
                        int sum_R = 0, sum_G = 0, sum_B = 0;
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
                            int place = index_y * NavWidth * 3 + index_x * 3;
                            sum_B += image.Frame[place++];
                            sum_G += image.Frame[place++];
                            sum_R += image.Frame[place];
                            for (int i = 0; i < 4; ++i)
                            {
                                if (DIS[index_x + dx[i], index_y + dy[i]] > arg * 7)//
                                {
                                    Q_x.Enqueue(index_x + dx[i]); Q_y.Enqueue(index_y + dy[i]);
                                    DIS[index_x + dx[i], index_y + dy[i]] = Mark;
                                }
                            }
                        }
                        if (pixel_count > 15)
                            Blobs.Enqueue(new Blob(sum_x / pixel_count, sum_y / pixel_count, max_x, max_y, min_x, min_y, sum_B, sum_G, sum_R, pixel_count));
                    }
                }
            }
            Blob[] res=Blobs.ToArray();
            for (int i = 0; i < res.Length; ++i)
            {
                //have shedow?
                //color
                double probability = 1;
                int max = Math.Max(Math.Max(res[i].sum_R, res[i].sum_G), res[i].sum_B);
                int min = Math.Min(Math.Min(res[i].sum_R, res[i].sum_G), res[i].sum_B);
                double saturation = 0;
                if (max != min)
                {
                    double light = 1.0 * (max + min) / 2 / res[i].pexel_num / 255;
                    if (light <= 0.5)
                        saturation = 1.0 * (max - min) / (max + min) * 240;
                    else
                        saturation = 1.0 * (max - min) / (2 * 255 * res[i].pexel_num - (max + min)) * 240;
                }
                if (saturation < StandardRockSaturation)
                    probability *= saturation / StandardRockSaturation;

                //shape..
                int length = Math.Max(res[i].max_x - res[i].min_x + 1, res[i].max_y - res[i].min_y + 1);
                int width = Math.Min(res[i].max_x - res[i].min_x + 1, res[i].max_y - res[i].min_y + 1);
                double proportion = 1.0 * length / width;
                if (proportion > StandardRockProportion)
                    probability *= 1.0 / (proportion - StandardRockProportion + 1);

                //axis proportion
                double step = CommonLib.CommonLib.ToRadians(3);//...
                double shortaxis = double.MaxValue;
                double longaxis = 0;
                for (double d = 0; d < Math.PI; d += step)
                {
                    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
                    int y_l = -1;
                    int y_r = 1 << 30;
                    if (d != Math.PI / 2)
                    {
                        y_l = (int)Math.Round(res[i].y - (res[i].x - res[i].min_x) * Math.Tan(d));
                        y_r = (int)Math.Round(res[i].y + (res[i].max_x - res[i].x) * Math.Tan(d));
                    }
                    if (y_l < res[i].min_y)
                    {
                        y1 = res[i].min_y;
                        x1 = (int)Math.Round(res[i].x - (res[i].y - res[i].min_y) / Math.Tan(d));
                    }
                    else if (y_l > res[i].max_y)
                    {
                        y1 = res[i].max_y;
                        x1 = (int)Math.Round(res[i].x + (res[i].max_y - res[i].y) / Math.Tan(d));
                    }
                    else
                    {
                        y1 = y_l;
                        x1 = res[i].min_x;
                    }

                    if (y_r < res[i].min_y)
                    {
                        y2 = res[i].min_y;
                        x2 = (int)Math.Round(res[i].x - (res[i].y - res[i].min_y) / Math.Tan(d));
                    }
                    else if (y_r > res[i].max_y)
                    {
                        y2 = res[i].max_y;
                        x2 = (int)Math.Round(res[i].x + (res[i].max_y - res[i].y) / Math.Tan(d));
                    }
                    else
                    {
                        y2 = y_r;
                        x2 = res[i].max_x;
                    }

                    //(x1,y1)->(x,y)
                    double dis_l = 0;
                    if (Math.Abs(x1 - res[i].x) < Math.Abs(y1 - res[i].y))
                    {
                        double deta = 1.0 * (res[i].x - x1) / (res[i].y - y1);
                        for (int y = y1; y != res[i].y; y += Math.Sign(res[i].y - y1))
                        {
                            int x = (int)Math.Round(x1 + (y - y1) * deta);
                            if (DIS[x, y] == Mark)
                            {
                                dis_l = Math.Sqrt(Math.Pow(x - res[i].x, 2) + Math.Pow(y - res[i].y, 2));
                                break;
                            }
                        }
                    }
                    else
                    {
                        double deta = 1.0 * (res[i].y - y1) / (res[i].x - x1);
                        for (int x = x1; x != res[i].x; x += Math.Sign(res[i].x - x1))
                        {
                            int y = (int)Math.Round(y1 + (x - x1) * deta);
                            if (DIS[x, y] == Mark)
                            {
                                dis_l = Math.Sqrt(Math.Pow(x - res[i].x, 2) + Math.Pow(y - res[i].y, 2));
                                break;
                            }
                        }
                    }

                    //(x2,y2)->(x,y)
                    double dis_r = 0;
                    if (Math.Abs(x2 - res[i].x) < Math.Abs(y2 - res[i].y))
                    {
                        double deta = 1.0 * (res[i].x - x2) / (res[i].y - y2);
                        for (int y = y2; y != res[i].y; y += Math.Sign(res[i].y - y2))
                        {
                            int x = (int)Math.Round(x2 + (y - y2) * deta);
                            if (DIS[x, y] == Mark)
                            {
                                dis_r = Math.Sqrt(Math.Pow(x - res[i].x, 2) + Math.Pow(y - res[i].y, 2));
                                break;
                            }
                        }
                    }
                    else
                    {
                        double deta = 1.0 * (res[i].y - y2) / (res[i].x - x2);
                        for (int x = x2; x != res[i].x; x += Math.Sign(res[i].x - x2))
                        {
                            int y = (int)Math.Round(y2 + (x - x2) * deta);
                            if (DIS[x, y] == Mark)
                            {
                                dis_r = Math.Sqrt(Math.Pow(x - res[i].x, 2) + Math.Pow(y - res[i].y, 2));
                                break;
                            }
                        }
                    }

                    shortaxis = Math.Min(shortaxis, dis_l + dis_r);
                    longaxis = Math.Max(longaxis, dis_l + dis_r);
                }
                double axisproportion = longaxis / shortaxis;
                if (axisproportion > StandardRockAxisP)
                    probability *= 1.0 / (axisproportion - StandardRockAxisP + 1);

                res[i].Rock_Pro = probability;
            }
            return res;
        }

        /// <summary>
        /// quite simple function use to mosaic.
        /// </summary>
        /// <param name="PanBlob"></param>
        /// <param name="NavBlob"></param>
        /// <param name="MinScore"></param>
        /// <returns></returns>
        public Vector2D MatchingBlobs(Blob PanBlob, Blob NavBlob, out double MinScore)
        {
            double min_score = double.MaxValue;
            double[,] score_memory = new double[_state.NavFrame.Size.Width, _state.NavFrame.Size.Height];
            int center_x = NavBlob.x, center_y = NavBlob.y;
            int min_centerx = center_x, min_centery = center_y;
            double max_len = Math.Max(Math.Max(PanBlob.max_x - PanBlob.x, PanBlob.x - PanBlob.min_x), Math.Max(PanBlob.max_y - PanBlob.x, PanBlob.x - PanBlob.min_y));
            for (int step = 3; step > 0; step -= 2)
            {
                bool sign = true;
                while (sign)
                {
                    sign = false;
                    for (int k = 0; k < 4; ++k)
                    {
                        bool smaller = true;
                        center_x = min_centerx;
                        center_y = min_centery;
                        while (smaller)
                        {
                            smaller = false;
                            center_x += dx[k] * step;
                            center_y += dy[k] * step;
                            if (!(center_x < NavBlob.max_x && center_x > NavBlob.min_x && center_y < NavBlob.max_y && center_y > NavBlob.min_y))
                                break;
                            //match search
                            double score = 0;
                            if (score_memory[center_x, center_y] > 0)
                                score = score_memory[center_x, center_y];
                            else
                            {
                                for (int x = PanBlob.min_x; x <= PanBlob.max_x; ++x)
                                    for (int y = PanBlob.min_y; y <= PanBlob.max_y; ++y)
                                    {
                                        //zoom
                                        int x_nav = center_x;
                                        int y_nav = center_y;
                                        if (y != PanBlob.y || x != PanBlob.x)
                                        {
                                            double len_pan = Math.Sqrt(Math.Pow(y - PanBlob.y, 2) + Math.Pow(x - PanBlob.x, 2));
                                            double sin = (y - PanBlob.y) / len_pan;
                                            double cos = (x - PanBlob.x) / len_pan;
                                            double len_nav = len_pan * ImageAnalyseTech.NavCam.FocusLen / ImageAnalyseTech.PanCam.FocusLen;
                                            //...
                                            x_nav = (int)Math.Round(len_nav * cos + center_x);
                                            y_nav = (int)Math.Round(len_nav * sin + center_y);
                                            if (x_nav < 0 || x_nav >= ImageSize || y_nav < 0 || y_nav >= ImageSize)
                                            {
                                                score += 10000;     //amerce
                                                continue;
                                            }
                                        }
                                        //check x_nav,y_nav
                                        int setoff_pan = y * _state.PanFrame.Size.Width * 3 + x * 3;
                                        int setoff_nac = y_nav * _state.NavFrame.Size.Width * 3 + x_nav * 3;
                                        double pro = 1 - (Math.Max(Math.Abs(x - PanBlob.x), Math.Abs(y - PanBlob.y)) / max_len) * 0.5;
                                        score += Math.Pow(_state.PanFrame.Frame[setoff_pan++] - _state.NavFrame.Frame[setoff_nac++], 2) * pro;
                                        score += Math.Pow(_state.PanFrame.Frame[setoff_pan++] - _state.NavFrame.Frame[setoff_nac++], 2) * pro;
                                        score += Math.Pow(_state.PanFrame.Frame[setoff_pan++] - _state.NavFrame.Frame[setoff_nac++], 2) * pro;
                                    }
                                score_memory[center_x, center_y] = score;
                            }
                            if (min_score > score)
                            {
                                smaller = true;
                                sign = true;
                                min_score = score;
                                min_centerx = center_x;
                                min_centery = center_y;
                            }
                        }//while(smaller)
                    }//for k 1:4
                }//while(sign)
            }//for step

            MinScore = min_score / (PanBlob.max_x - PanBlob.min_x) / (PanBlob.max_y - PanBlob.min_y) / 3;    
            
            return new Vector2D(min_centerx, min_centery);
        }
    }

}