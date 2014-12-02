using Microsoft.Ccr.Core;
using Microsoft.Dss.Core.Attributes;
using Microsoft.Dss.ServiceModel.Dssp;
using Microsoft.Dss.ServiceModel.DsspServiceBase;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using Microsoft.Ccr.Core.Arbiters;
using Microsoft.Robotics.PhysicalModel.Proxy;

using Microsoft.Robotics.RoboChamps.MarsChallenger.CommonLib;
using drive = Microsoft.Robotics.Services.Drive.Proxy;
using processor = Microsoft.Robotics.RoboChamps.MarsChallenger.ImageProcessing;
using stereovision = Microsoft.Robotics.RoboChamps.MarsChallenger.StereoVision;
using comlink = Microsoft.Robotics.RoboChamps.MarsComLink.Proxy;
using spectro = Microsoft.Robotics.RoboChamps.MarsSpectrometer.Proxy;
using pantilt = SimplySim.Robotics.PanTilt.Proxy;
using arm = Microsoft.Robotics.Services.ArticulatedArm.Proxy;

namespace Microsoft.Robotics.RoboChamps.MarsChallenger
{
    [DisplayName("MarsChallenger")]
    [Description("zero.lin's Mars Challenger Service")]
    [Contract(Contract.Identifier)]
    public class MarsChallengerService : DsspServiceBase
    {
        /// <summary>
        /// Constant vector for articulated arm operation
        /// </summary>
        private static Vector3 Twist = new Vector3(1, 0, 0);

        /// <summary>
        /// Constants for arm joint names
        /// </summary>
        private const string ShoulderHJointName = "ShoulderH";
        private const string ShoulderVJointName = "ShoulderV";
        private const string ElbowJointName = "Elbow";
        private const string WristHJointName = "WristH";
        private const string WristVJointName = "WristV";

        /// <summary>
        /// Service state.
        /// You can view current state with a browser http://localhost:50000/marschallenger
        /// </summary>
        [ServiceState]
        private MarsChallengerState _state = new MarsChallengerState();

        /// <summary>
        /// Service operation port
        /// </summary>
        [ServicePort("/MarsChallenger", AllowMultipleInstances = false)]
        private MarsChallengerOperations _mainPort = new MarsChallengerOperations();

        #region Partners

        [Partner("ComLink", Contract = comlink.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private comlink.MarsComLinkOperations _comlinkPort = new comlink.MarsComLinkOperations();
        private comlink.MarsComLinkOperations _comlinkNotify = new comlink.MarsComLinkOperations();

        [Partner("Spectro", Contract = spectro.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private spectro.MarsSpectrometerOperations _spectroPort = new spectro.MarsSpectrometerOperations();
        private spectro.MarsSpectrometerOperations _spectroNotify = new spectro.MarsSpectrometerOperations();

        [Partner("StereoVision", Contract = stereovision.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private stereovision.StereoVisionOperations _stereoPort = new stereovision.StereoVisionOperations();
        private stereovision.StereoVisionOperations _stereoNotify = new stereovision.StereoVisionOperations();

        [Partner("FrontCamProcessor", Contract = processor.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private processor.ImageProcessingOperations _frontcamPort = new processor.ImageProcessingOperations();
        private processor.ImageProcessingOperations _frontcamNotify = new processor.ImageProcessingOperations();

        [Partner("RearCamProcessor", Contract = processor.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private processor.ImageProcessingOperations _rearcamPort = new processor.ImageProcessingOperations();
        private processor.ImageProcessingOperations _rearcamNotify = new processor.ImageProcessingOperations();

        [Partner("Drive", Contract = drive.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private drive.DriveOperations _drivePort = new drive.DriveOperations();

        [Partner("Head", Contract = pantilt.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private pantilt.PanTiltOperations _headPort = new pantilt.PanTiltOperations();

        [Partner("Arm", Contract = arm.Contract.Identifier, CreationPolicy = PartnerCreationPolicy.UsePartnerListEntry)]
        private arm.ArticulatedArmOperations _armPort = new arm.ArticulatedArmOperations();


        #endregion

        public MarsChallengerService(DsspServiceCreationPort creationPort) :
            base(creationPort)
        {
        }

        protected override void Start()
        {
            base.Start();

            _mainPort.Post(new Initialize());
        }

        #region Initialization

        [ServiceHandler(ServiceHandlerBehavior.Exclusive)]
        public IEnumerator<ITask> InitializeHandler(Initialize initialize)
        {
            _state.State = RoverState.Initialize;

            _state.ComlinkMemory = new ComlinkMemory();
            while(_state.ComlinkMemory.Data.Count==0)
                yield return Arbiter.Choice(_comlinkPort.Get(), (success) => { success.Timestamp = DateTime.Now; _state.ComlinkMemory.Update(success); }, LogError);
            yield return Arbiter.Choice(_spectroPort.Get(), (success) => _state.SpectrometerData = success, LogError);

            // Adding handlers for sensors and camera processor notifications
            MainPortInterleave.CombineWith(
                Arbiter.Interleave(
                    new TeardownReceiverGroup(),
                    new ExclusiveReceiverGroup
                    (
                        Arbiter.Receive<comlink.Replace>(true, _comlinkNotify, ComlinkReplaceHandler),
                        Arbiter.Receive<spectro.Replace>(true, _spectroNotify, SpectroReplaceHandler),
                        Arbiter.Receive<stereovision.ProcessVision>(true, _stereoNotify, StereoVisionHandler),
                        Arbiter.Receive<processor.ProcessImage>(true, _frontcamNotify, FrontCamProcessImageHandler),
                        Arbiter.Receive<processor.ProcessImage>(true, _rearcamNotify, RearCamProcessImageHandler)
                    ),
                    new ConcurrentReceiverGroup()));

            // Subscribing 
            yield return Arbiter.Choice(
                _comlinkPort.Subscribe(_comlinkNotify, typeof(comlink.Replace)),
                (success) => { },
                (fault) => LogError(fault));

            yield return Arbiter.Choice(
                _spectroPort.Subscribe(_spectroNotify, typeof(spectro.Replace)),
                (success) => { },
                (fault) => LogError(fault));

            yield return Arbiter.Choice(
                _stereoPort.Subscribe(_stereoNotify, typeof(stereovision.ProcessVision)),
                (success) => { },
                (fault) => LogError(fault));

            yield return Arbiter.Choice(
                _frontcamPort.Subscribe(_frontcamNotify, typeof(processor.ProcessImage)),
                (success) => { },
                (fault) => LogError(fault));

            yield return Arbiter.Choice(
                _rearcamPort.Subscribe(_rearcamNotify, typeof(processor.ProcessImage)),
                (success) => { },
                (fault) => LogError(fault));

            RoverInitHandle();
        }

        private void RoverInitHandle()
        {
            //init class members
            _state.HeadState = new HeadState();
            _state.DriveState = new DriveState();
            _state.SI = new SensorInterval();
            _state.VisionMemory = new VisionMemory();
            _state.MissionList = new LinkedList<Mission>();

                
            comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();

            _state.Memory = new Memory(CData.Position.X, CData.Position.Z);

            // Move the arm in a basic position
            ArmReset();

            // Move the head
            HeadHelper(0,-10,ControlLevel.Brain);

            //init mission list
            for (int i = 0; i < CData.Areas.Length; ++i)
            {
                _state.MissionList.AddLast(new Mission(mission_type.ReachArea, CData.Areas[i].Name, CData.Areas[i].Position));
                _state.MissionList.AddLast(new Mission(mission_type.ExamineRock, CData.Areas[i].Name, CData.Areas[i].Position, 3));
            }

            //add last mission: reach shield
            comlink.Position ShieldP=new comlink.Position ();
            ShieldP.X = 60; ShieldP.Y = 5; ShieldP.Z = 111;
            _state.MissionList.AddLast(new Mission(mission_type.ReachShield, "Heatshield", ShieldP, 80));

            // Set a new state
            _state.State = RoverState.Navigate;
        }
       
        #endregion

        #region Sensors Notifications Handlers

        private void CheckMissionState()
        {
            if (_state.current_M == null)
                return;
            comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();
            switch (_state.current_M.type)
            {
                case mission_type.ReachArea:
                    if (CData.CurrentArea != null && CData.CurrentArea.Name == _state.current_M.name)
                        _state.current_M.state = mission_state.success;
                    break;
                case mission_type.ExamineRock:
                    if (_state.SpectrometerData.InContact)
                    {
                        ExamineRockHelper(MessageType.GetData);
                        _state.current_M.state = mission_state.success;
                    }
                    break;
                case mission_type.ReachShield:
                    if (CData.CurrentArea != null && CData.CurrentArea.Name == _state.current_M.name)
                        _state.current_M.state = mission_state.success;
                    break;
            }
        }

        private void DataUpdate()
        {
            CheckMissionState();
            if (_state.current_M == null || _state.current_M.state == mission_state.success)
            {
                if (_state.MissionList.Count == 0)
                {
                    EndMission();
                    return;
                }

                //get new mission
                _state.current_M = _state.MissionList.First.Value;
                _state.MissionList.RemoveFirst();
                
                //update vision memory
                _state.VisionMemory.Newmission(_state.current_M);
                if (_state.current_M.type == mission_type.ReachShield)
                {
                    //change stereo vision service's mode
                    _stereoPort.Post(new StereoVision.ModeChange(new StereoVision.Mode(StereoVision.Search.shield)));
                    Angle_down = -10;
                }

                LogInfo("new mission, name:" + _state.current_M.name + " position:" + _state.current_M.position.X + " " + _state.current_M.position.Y + " " + _state.current_M.position.Z);
            } 
        }

        private void EndMission()
        {
            _state.State = RoverState.EndMission;
            StopRover(ControlLevel.FromEarth);

            /*CommonLib.CommonLib.DrawImage(_state.Memory.Map,_state.VisionMemory.Database,_state.Memory.Center_x,_state.Memory.Center_z);
            CommonLib.CommonLib.DrawImage(_state.Memory.Reliability,_state.VisionMemory.Database,_state.Memory.Center_x,_state.Memory.Center_z);
            System.IO.StreamWriter writer = new System.IO.StreamWriter("C:\\Users\\zerolin\\Microsoft Robotics Dev Studio 2008\\RoboChamps\\Mars\\images\\SI.txt");
            writer.WriteLine("comlink: " + _state.SI.ComLinkInterval);
            writer.WriteLine("frontcam: " + _state.SI.FroCamInterval);
            writer.WriteLine("rearcam: " + _state.SI.RearCamInterval);
            writer.WriteLine("vision: " + _state.SI.VisionInterval);
            writer.Close();
            */
            // Send end mission message to Earth !
            Arbiter.Choice(_comlinkPort.EndMission(), (success) => { }, LogError);
        }

        const double DisThres = 1;
        private void ComlinkReplaceHandler(comlink.Replace replace)
        {
            
            if (replace.Body.Timestamp.CompareTo(_state.SI.LatestComLink) < 0)
                return;
            _state.SI.ComLinkInterval = (int)replace.Body.Timestamp.Subtract(_state.SI.LatestComLink).TotalMilliseconds / 2 + _state.SI.ComLinkInterval / 2;
            _state.SI.LatestComLink = replace.Body.Timestamp;
            replace.Body.Timestamp = DateTime.Now;
            _state.ComlinkMemory.Update(replace.Body);
            if (_state.State == RoverState.Navigate)
            {
                DataUpdate();
                
                DriveHelper(_state.current_M.position, _state.SI.ComLinkInterval, ControlLevel.Basic);
                
                _state.Memory.MapUpdate(replace.Body.Position);
                
                //send message to function subcibe comlink
                ClimbHelper(MessageType.Comlink);
                PositionHelper(MessageType.Comlink);
                if (_state.current_M.type == mission_type.ExamineRock)
                    ExamineRockHelper(MessageType.Comlink);
                if ((Math.Abs(replace.Body.Pitch) > 15 || Math.Abs(replace.Body.Roll)>13 )&& _state.current_M.type!=mission_type.ExamineRock)
                    CareBody(MessageType.Comlink);
                if (Distance2D(_state.current_M.position, replace.Body.Position) < DisThres)
                    PositionHelper(MessageType.ReachArea);
            }
        }

        private void SpectroReplaceHandler(spectro.Replace replace)
        {
            if (_state.State == RoverState.Navigate)
            {
                _state.SpectrometerData = replace.Body;
                DataUpdate();
            }
        }

        int SVH_Sign = 1;
        /// <summary>
        /// take charge in translate vision data and control head 
        /// </summary>
        /// <param name="process"></param>
        private void StereoVisionHandler(stereovision.ProcessVision process)
        {
            if (process.Body.TimeStamp.CompareTo(_state.SI.LatestVision) < 0)
                return;
            _state.SI.VisionInterval = (int)process.Body.TimeStamp.Subtract(_state.SI.LatestVision).TotalMilliseconds / 2 + _state.SI.VisionInterval / 2;
            _state.SI.LatestVision = process.Body.TimeStamp;
            if (_state.State == RoverState.Navigate)
            {
                comlink.MarsComLinkState CData = _state.ComlinkMemory.Get(process.Body.TimeStamp);
                if (process.Body.Rocks != null)
                {
                    double min_scoer = double.MaxValue;
                    int num = -1;
                    for (int i = 0; i < process.Body.Rocks.Length; ++i)
                    {
                        Vector3D p = ImageAnalyseTech.PanToRoverCoo(new Vector3D(process.Body.Rocks[i].X, process.Body.Rocks[i].Y, process.Body.Rocks[i].Z),
                            ToRadians(_state.HeadState.Pan), ToRadians(_state.HeadState.Tilt));
                        p = ImageAnalyseTech.CooTran(new Vector3D(-p.x, p.y, -p.z)
                            , new Vector3D(CData.Position.X, CData.Position.Y, CData.Position.Z),
                            new Vector3D(ToRadians(-CData.Pitch), ToRadians(-CData.Yaw), ToRadians(-CData.Roll)));

                        if (process.Body.Rocks[i].Type == stereovision.RockType.Rock)
                            _state.VisionMemory.Update(new VisionData(p.ToComlink(), process.Body.Rocks[i].Score, stereovision.RockType.Rock));
                        else
                        {
                            if (process.Body.Rocks[i].Score < _state.current_M.score && Distance(p.ToComlink(), _state.current_M.place) < _state.current_M.radix)
                            {
                                _state.current_M.position = p.ToComlink();
                                _state.current_M.score = process.Body.Rocks[i].Score;
                            }
                            _state.VisionMemory.Update(new VisionData(p.ToComlink(), process.Body.Rocks[i].Score, stereovision.RockType.Shield));
                        }
                        if (min_scoer > process.Body.Rocks[i].Score)
                        {
                            min_scoer = process.Body.Rocks[i].Score;
                            num = i;
                        }
                    }
                    if (num != -1)
                    {
                        //look at min_score
                        Vector2D S_coo = ImageAnalyseTech.ToSphericalCoo(ImageAnalyseTech.ToImageCoo(new Vector2D(process.Body.PanBlobs[process.Body.Rocks[num].PanBlob_num].x, process.Body.PanBlobs[process.Body.Rocks[num].PanBlob_num].y)), CamType.Pan);
                        HeadHelper(_state.HeadState.Pan + ToDegree((float)S_coo.x), _state.HeadState.Tilt + ToDegree((float)S_coo.y), ControlLevel.Eyes);
                        return;
                    }
                }
                if (process.Body.NavBlobs.Length != 0)
                {
                    double max_score = -1;
                    int num = -1;
                    for (int i = 0; i < process.Body.NavBlobs.Length; ++i)
                    {
                        double score = process.Body.NavBlobs[i].Rock_Pro + process.Body.NavBlobs[i].Shield_Pro;

                        if (score > max_score)
                        {
                            max_score = score;
                            num = i;
                        }
                    }
                    if (process.Body.NavBlobs[num].Rock_Pro > 0.6 || process.Body.NavBlobs[num].Shield_Pro > 0.6)
                    {
                        Vector2D S_coo = ImageAnalyseTech.ToSphericalCoo(ImageAnalyseTech.ToImageCoo(new Vector2D(process.Body.NavBlobs[num].x, process.Body.NavBlobs[num].y)), CamType.Nav);

                        if (process.Body.Center_x == 0)
                        {
                            process.Body.Center_x = 64;
                            process.Body.Center_y = 64;
                        }
                        Vector2D Center = ImageAnalyseTech.ToSphericalCoo(ImageAnalyseTech.ToImageCoo( new Vector2D(process.Body.Center_x, process.Body.Center_y)), CamType.Nav);

                        HeadHelper(_state.HeadState.Pan + (ToDegree((float)S_coo.x) - ToDegree((float)Center.x)), _state.HeadState.Tilt + (ToDegree((float)S_coo.y) - ToDegree((float)Center.y)), ControlLevel.Eyes);
                        return;
                    }
                }
                if (_state.current_M.type != mission_type.ReachShield)
                {
                    float V = -60;
                    float len = (float)Math.Sqrt(Math.Pow(_state.current_M.position.X - CData.Position.X, 2) + Math.Pow(_state.current_M.position.Z - CData.Position.Z, 2));
                    if (len != 0)
                        V = ToDegree((float)Math.Atan((_state.current_M.position.Y - (ImageAnalyseTech.HeadPosition[1] + CData.Position.Y)) / len));
                    HeadHelper(0, V + CData.Pitch, ControlLevel.Eyes);
                }
                else
                {
                    HeadHelper(_state.HeadState.Pan + 3 * SVH_Sign, -5, ControlLevel.Eyes);
                    if (_state.HeadState.Pan > 60)
                        SVH_Sign = -1;
                    else if (_state.HeadState.Pan < -60)
                        SVH_Sign = 1;
                }
            }
        }

        private void FrontCamProcessImageHandler(processor.ProcessImage process)
        {
            if (process.Body.Timestamp.CompareTo(_state.SI.LatestFroCam) < 0)
                return;
            _state.SI.FroCamInterval = (int)process.Body.Timestamp.Subtract(_state.SI.LatestFroCam).TotalMilliseconds / 2 + _state.SI.FroCamInterval / 2;
            _state.SI.LatestFroCam = process.Body.Timestamp;
            _state.FrontCamData = process.Body;
            if (_state.State == RoverState.Navigate)
            {
                if (process.Body.Avoid.Length != 0)
                {
                    if (_state.current_M.type == mission_type.ExamineRock)
                    {
                        ExamineRockHelper(MessageType.FrontAdvoid);
                    }
                    AdvoidHelper(MessageType.FrontAdvoid);
                }
                else 
                {
                    if (_state.current_M.type == mission_type.ExamineRock)
                        ExamineRockHelper(MessageType.FrontNonAdvoid);
                    AdvoidHelper(MessageType.FrontNonAdvoid);
                }

                comlink.MarsComLinkState CData = _state.ComlinkMemory.Get(process.Body.Timestamp);
                _state.Memory.MapUpdate(CData.Position, new Vector3D(ToRadians(CData.Pitch), ToRadians(CData.Yaw), ToRadians(CData.Roll)), process.Body.Edge, CamType.Front);
            }

        }

        private void RearCamProcessImageHandler(processor.ProcessImage process)
        {
            if (process.Body.Timestamp.CompareTo(_state.SI.LatestRearCam) < 0)
                return;
            _state.SI.RearCamInterval = (int)process.Body.Timestamp.Subtract(_state.SI.LatestRearCam).TotalMilliseconds / 2 + _state.SI.RearCamInterval / 2;
            _state.SI.LatestRearCam = process.Body.Timestamp;
            _state.RearCamData = process.Body;
            if (_state.State == RoverState.Navigate)
            {
                if (process.Body.Avoid.Length != 0)
                {
                    if (_state.current_M.type == mission_type.ExamineRock)
                    {
                        ExamineRockHelper(MessageType.RearAdvoid);
                    }
                    AdvoidHelper(MessageType.RearAdvoid);
                }
                else
                {
                    if (_state.current_M.type == mission_type.ExamineRock)
                        ExamineRockHelper(MessageType.RearNonAdvoid);
                    AdvoidHelper(MessageType.RearNonAdvoid);
                }

                comlink.MarsComLinkState CData = _state.ComlinkMemory.Get(process.Body.Timestamp);
                _state.Memory.MapUpdate(CData.Position, new Vector3D(ToRadians(CData.Pitch), ToRadians(CData.Yaw), ToRadians(CData.Roll)), process.Body.Edge, CamType.Rear);
            }
        }

        #endregion

        #region DriveHelper

        private void StopRover(ControlLevel level)
        {
            if (level < _state.DriveState.Level)
                return;
            _state.DriveState.Level = level;
            Drive(0, 0, level);
        }

        /// <summary>
        /// a basic fuction control the drive. it a part between brain and body
        /// it also take care of power defference and watch weather rover is traped
        /// </summary>
        /// <param name="left"></param>
        /// <param name="right"></param>
        /// <param name="level"></param>
        void Drive(double left, double right, ControlLevel level)
        {
            if (level < _state.DriveState.Level )
                return;
            double increment = 1;
            comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();
            if (Math.Abs(CData.Roll) > 10 )
            {
                double max_change = Math.Max(0.1, 0.4 - (Math.Abs(CData.Roll) - 10) / 5);
                double L = _state.DriveState.Left, R = _state.DriveState.Right;
                if (left == right)
                {
                    L = Math.Min(L, R);
                    R = L;
                }
                if (Math.Abs(left - L) > max_change)
                    left = L + max_change * Math.Sign(left - L);
                if (Math.Abs(right - R) > max_change)
                    right = R + max_change * Math.Sign(right - R);
            }
            left = Math.Max(-1, Math.Min(1, left));
            right = Math.Max(-1, Math.Min(1, right));

            //check weather trapped
            if (left == _state.DriveState.Left && right == _state.DriveState.Right)
            {
                if(DateTime.Now.Subtract(_state.DriveState.LastTimeChange).TotalMilliseconds>2000)
                {
                    comlink.MarsComLinkState from = _state.ComlinkMemory.Get(_state.DriveState.LastTimeChange);
                    comlink.MarsComLinkState to = _state.ComlinkMemory.Get(DateTime.Now);
                    double dis = Distance(from.Position, to.Position);
                    double yaw = Math.Abs(from.Yaw - to.Yaw) % 180;
                    double time=DateTime.Now.Subtract(_state.DriveState.LastTimeChange).TotalMilliseconds;

                    if (left == 0 && right == 0)
                        ;
                    else if (left * right <= 0)
                        if (yaw < time * Drive_Turn_Prm[(int)(Math.Abs(left - right) / 2 * 10 - 1)] / 2)
                            increment *= time * Drive_Turn_Prm[(int)(Math.Abs(left - right) / 2 * 10 - 1)] / 2 / yaw;
                    else
                        if(dis < time* Drive_Forward_Prm[(int)(Math.Abs(left - right) / 2 * 10 - 1)] / 2)
                            increment *= time * Drive_Forward_Prm[(int)(Math.Abs(left - right) / 2 * 10 - 1)] / 2 / dis;
                }
                else
                    return;
            }

            if (increment > 1 && (left==0 || right==0))
            {
                if (left == 0)
                {
                    double value = Math.Abs(right * increment) - 1;
                    if (value > 0)
                        _drivePort.SetDrivePower(-value * Math.Sign(right), right * increment);
                    else
                        _drivePort.SetDrivePower(left * increment, right * increment);
                }
                else
                {
                    double value = Math.Abs(left * increment) - 1;
                    if (value > 0)
                        _drivePort.SetDrivePower(left * increment, -value * Math.Sign(left));
                    else
                        _drivePort.SetDrivePower(left * increment, right * increment);
                }
            }
            else
                _drivePort.SetDrivePower(left * increment, right * increment);
            _state.DriveState.Left = left; _state.DriveState.Right = right;
            if(increment==1)
                _state.DriveState.LastTimeChange = DateTime.Now;
        }

        double GetAngle(comlink.Position goal)
        {
            comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();

            double Tan = 0;
            if (goal.Z - CData.Position.Z != 0)
                Tan = (goal.X - CData.Position.X) / (goal.Z - CData.Position.Z);
            double sata = Math.Atan(Tan) / Math.PI * 180;
            if (goal.Z - CData.Position.Z < 0)
            {
                if (sata > 0)
                    sata -= 180;
                else
                    sata += 180;
            }
            double deta = sata - CData.Yaw;
            if (deta < -180)
                deta += 360;
            else if (deta > 180)
                deta -= 360;

            return deta;
        }

        void DriveHelper(comlink.Position goal, int Interval_ms, ControlLevel level)
        {
            if (level < _state.DriveState.Level)
                return;
            
            comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();

            double deta = GetAngle(goal);

            if (deta > _state.DriveState.Solution)
            {
                double turn_p = Turn(deta, Interval_ms);
                Drive(-turn_p, turn_p, level);
            }
            else if (deta < -_state.DriveState.Solution)
            {
                double turn_p = Turn(-deta, Interval_ms);
                Drive(turn_p, -turn_p, level);
            }
            else
            {
                double forward_p = Forward(Distance(goal, CData.Position), Interval_ms);
                Drive(forward_p, forward_p, level);
            }
        }

        double[] Drive_Turn_Prm ={0.0133720626031732,
                                0.028238447750641,
                                0.0421635890637907,
                                0.0559616132741301,
                                0.0718866944313049,
                                0.0857190346020106,
                                0.0975352885843325,
                                0.111652660369873,
                                0.122752882107218,
                                0.141540785943078};
        double Turn(double Angle, int Interval_ms)
        {
            int left = 0, right = Drive_Turn_Prm.Length - 1;
            while (true)
            {
                if (Drive_Turn_Prm[(left + right) / 2] * Interval_ms < Angle)
                    left = (left + right) / 2 + 1;
                else
                    right = (left + right) / 2 - 1;
                if (right <= left)
                    return 1.0 * (left + 1) / 10;
            }

        }

        double[] Drive_Forward_Prm ={0.000118625503091607,
                                    0.000221727837924846,
                                    0.000363802653737366,
                                    0.000440930540207773,
                                    0.000566910952329636,
                                    0.000714599504135549,
                                    0.000852787808980793,
                                    0.000955054245423526,
                                    0.00110033480450511,
                                    0.00115549028851092};
        double Forward(double Dis, int Interval_ms)
        {
            int left = 0, right = Drive_Turn_Prm.Length - 1;
            while (true)
            {
                if (Drive_Forward_Prm[(left + right) / 2] * Interval_ms < Dis)
                    left = (left + right) / 2 + 1;
                else
                    right = (left + right) / 2 - 1;
                if (right <= left)
                    return 1.0 * (left + 1) / 10;
            }
        }

        #endregion

        #region ArmHelper

        class ArmPara
        {
            public Vector3D Size { get; set; }
            public Vector3D Position { get; set; }
            public double Angle_H { get; set; }
            public double Angle_V { get; set; }
            public ArmPara(Vector3D size, Vector3D position, double angle_H, double angle_V)
            {
                Size = size;
                Position = position;
                Angle_H = angle_H;
                Angle_V = angle_V;
            }
        }
        ArmPara ArmBase = new ArmPara(new Vector3D(0.15, 0.12, 0.12), new Vector3D(-0.3, -0.19, -0.41), 0, 0);
        ArmPara Shoulder = new ArmPara(new Vector3D(0.02, 0.06, 0.35), new Vector3D(0.1, -0.085, -0.091), 160, 70);//0.3 -0.1 change 5.14
        ArmPara Elbow = new ArmPara(new Vector3D(0.03, 0.03, 0.25), new Vector3D(0.03, 0, -0.15), 0, 290);
        ArmPara WristV = new ArmPara(new Vector3D(0.0073, 0.0319, 0.0092), new Vector3D(0.04, -0.001, -0.12), 0, 340);
        ArmPara WristH = new ArmPara(new Vector3D(0.0535, 0.0633, 0.1425), new Vector3D(0.01, 0.01, 0), 350, 0);
        double Spe_len = 0.12;

        enum ArmResult
        {
            Done,
            Left,
            Right,
            Forward,
            Backward,
        }
        /// <summary>
        /// important function use to control the arm. 
        /// </summary>
        /// <param name="position">postion of the rock</param>
        /// <param name="oritation">oritation of the rock surface</param>
        /// <returns></returns>
        ArmResult ArmControl(Vector3D position, Vector3D oritation)
        {
            position.x -= ArmBase.Position.x; position.y -= ArmBase.Position.y; position.z -= ArmBase.Position.z;
            position.x -= Shoulder.Position.x; position.y -= Shoulder.Position.y; position.z -= Shoulder.Position.z;

            double wrist_h = Math.PI / 2 * (-oritation.y <= 0 ? 1 : -1);
            if (oritation.x != 0)
                wrist_h = Math.Atan(oritation.y / oritation.x);
            if (-oritation.x > 0)
            {
                if (wrist_h > 0)
                    wrist_h -= Math.PI;
                if (wrist_h < 0)
                    wrist_h += Math.PI;
            }
            if (Math.Abs(wrist_h) > ToRadians((float)WristH.Angle_H / 2))
            {
                if (wrist_h < 0)
                    return ArmResult.Backward;
                return ArmResult.Forward;
            }

            position.x += oritation.x * Spe_len; position.y += oritation.y * Spe_len; position.z += WristH.Size.y / 2 + WristV.Size.y;
            position.x -= WristV.Position.x; position.y -= WristV.Position.y; position.z -= (WristV.Position.z + Elbow.Size.z / 2);

            double arpha = 0;
            if (position.z != 0)
                arpha = Math.Atan(position.x / position.z);
            if (Math.Abs(arpha) > ToRadians((float)Shoulder.Angle_H / 2))
            {
                if (arpha < 0)
                    return ArmResult.Right;
                return ArmResult.Left;
            }

            double r = Math.Sqrt(Math.Pow(position.x, 2) + Math.Pow(position.y, 2) + Math.Pow(position.z, 2));
            if (r > Shoulder.Size.z + Elbow.Size.z)
            {
                if (arpha > 45)
                    return ArmResult.Left;
                else if (arpha < -45)
                    return ArmResult.Right;
                return ArmResult.Forward;
            }
            if (r < Math.Abs(Shoulder.Size.z - Elbow.Size.z))
                return ArmResult.Backward;

            double elbow = Math.PI - Math.Acos((Math.Pow(Shoulder.Size.z, 2) + Math.Pow(Elbow.Size.z, 2) - Math.Pow(r, 2)) / (2 * Shoulder.Size.z * Elbow.Size.z));
            if (elbow > ToRadians((float)Elbow.Angle_V / 2))
                return ArmResult.Backward;

            double seta = Math.Atan(position.y / Math.Sqrt(Math.Pow(position.x, 2) + Math.Pow(position.z, 2)));
            double deta = Math.Asin(Elbow.Size.z * Math.Sin(elbow) / r);
            double should_v = 0;
            if (seta > 0)
            {
                should_v = seta - deta;
            }
            else
            {
                elbow = -elbow;
                should_v = seta + deta;
            }
            if (Math.Abs(should_v) > ToRadians((float)Shoulder.Angle_V / 2))
                return ArmResult.Backward;

            double wrist_v = Math.PI / 2 + (Math.Abs(elbow) - Math.Abs(should_v)) * (elbow >= 0 ? -1 : 1);
            if (Math.Abs(wrist_v) > ToRadians((float)WristV.Angle_V / 2))
                return ArmResult.Backward;

            _armPort.SetJointTargetPose(ElbowJointName, new Vector3(), new AxisAngle(Twist, (float)elbow));
            _armPort.SetJointTargetPose(WristVJointName, new Vector3(), new AxisAngle(Twist, (float)wrist_v));
            _armPort.SetJointTargetPose(WristHJointName, new Vector3(), new AxisAngle(Twist, (float)wrist_h));
            _armPort.SetJointTargetPose(ShoulderHJointName, new Vector3(), new AxisAngle(Twist, (float)arpha));
            _armPort.SetJointTargetPose(ShoulderVJointName, new Vector3(), new AxisAngle(Twist, (float)should_v));
            return ArmResult.Done;
        }

        void ArmReset()
        {
            // Move the arm in a basic position
            _armPort.SetJointTargetPose(ShoulderVJointName, new Vector3(), new AxisAngle(Twist, ToRadians(35)));
            _armPort.SetJointTargetPose(ShoulderHJointName, new Vector3(), new AxisAngle(Twist, ToRadians(0)));
            _armPort.SetJointTargetPose(WristHJointName, new Vector3(), new AxisAngle(Twist, ToRadians(90)));
            _armPort.SetJointTargetPose(WristVJointName, new Vector3(), new AxisAngle(Twist, ToRadians(0)));
            _armPort.SetJointTargetPose(ElbowJointName, new Vector3(), new AxisAngle(Twist, ToRadians(55)));
        }

        IEnumerator<ITask> ArmReset(DateTime t)
        {
            ArmReset();
            yield break;
        }

        private void RearmHelper(int rearmAfter)
        {
            if (MainPortInterleave.ArbiterState != ArbiterTaskState.Done)
            {
                MainPortInterleave.CombineWith(Arbiter.Interleave(
                    new TeardownReceiverGroup(),
                    new ExclusiveReceiverGroup
                    (
                        Arbiter.ReceiveWithIterator<DateTime>(false, TimeoutPort(rearmAfter), ArmReset)
                    ),
                    new ConcurrentReceiverGroup()));
            }
        }

        #endregion

        #region Helpers

        private float ToDegree(float radians)
        {
            return (float)(radians * 180.0d / Math.PI);
        }
        /// <summary>
        /// Converts degrees to radians
        /// </summary>
        private float ToRadians(float degree)
        {
            return (float)(degree * Math.PI / 180.0d);
        }

        /// <summary>
        /// Calculates the distance between two positions
        /// </summary>
        public static float Distance(comlink.Position from, comlink.Position to)
        {
            return (float)Math.Sqrt(Math.Pow(to.X - from.X, 2) + Math.Pow(to.Y - from.Y, 2) + Math.Pow(to.Z - from.Z, 2));
        }
        public static float Distance2D(comlink.Position from, comlink.Position to)
        {
            return (float)Math.Sqrt(Math.Pow(to.X - from.X, 2) + Math.Pow(to.Z - from.Z, 2));
        }
        #endregion

        #region HeadHelper
       
        const float HeadThres = 5;
        float Angle_down = -60;
        void HeadHelper(float H, float V, ControlLevel level)
        {
            if (level < _state.HeadState.CLevel)
                return;

            if (Math.Abs(_state.HeadState.Pan - H) > HeadThres)
                H = _state.HeadState.Pan + Math.Sign(H - _state.HeadState.Pan) * HeadThres;
            H = Math.Max(-70, Math.Min(70, H));
            _headPort.SetPan(H);
            _state.HeadState.Pan = H;

            if (Math.Abs(_state.HeadState.Tilt - V) > HeadThres)
                V = _state.HeadState.Tilt + Math.Sign(V - _state.HeadState.Tilt) * HeadThres;
            V = Math.Max(Angle_down, Math.Min(10, V));
            _headPort.SetTilt(V);
            _state.HeadState.Tilt = V;
            _state.HeadState.LastChange = DateTime.Now;
        }
       
        #endregion

        #region Brain

        enum MessageType
        {
            FrontAdvoid,
            FrontNonAdvoid,
            RearAdvoid,
            RearNonAdvoid,
            Comlink,
            Vision,
            GetData,
            SearchRock,
            SearchShield,
            FoundShield,
            ReachArea,
            Climb,
        }

        bool ERH_ListenFroAdvoid = true;
        bool ERH_ListenFroNonAdvoid = false;
        bool ERH_ListenRearAdvoid = true;
        bool ERH_ListenComlink = true;
        bool ERH_ListenSpect = true;
        bool ERH_Flag = false;
        int ERH_Count = 0;
        /// <summary>
        /// part of brain. respond to examine rock
        /// </summary>
        /// <param name="message"></param>
        void ExamineRockHelper(MessageType message)
        {
            if (ERH_ListenFroAdvoid && message == MessageType.FrontAdvoid)
            {
                StopRover(ControlLevel.Brain);
                if (!ERH_Flag)
                {
                    ERH_Flag = true;
                    ERH_ListenFroNonAdvoid = true;
                    ERH_ListenRearAdvoid = false;
                    ERH_ListenComlink = false;
                    ERH_Count = 0;
                    return;
                }
                Vector3D position = ImageAnalyseTech.BasicVision(new Vector2D(_state.FrontCamData.Avoid[_state.FrontCamData.Avoid.Length - 1].x, _state.FrontCamData.Avoid[_state.FrontCamData.Avoid.Length - 1].y), CamType.Front);
                double sin = (60 - _state.FrontCamData.Avoid[0].x) / 68;
                Vector3D oritation=new Vector3D (sin ,Math.Sqrt(1-Math.Pow(sin,2)),0);
                ArmResult res=ArmControl(position , oritation );
                comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();
                switch (res)
                {
                    case ArmResult.Done:
                        ERH_Count++;
                        if (ERH_Count > 10)
                        {
                            Drive(-1, -1, ControlLevel.Brain);
                            ERH_Count /= 2;
                        }
                        RearmHelper(400);
                        break;
                    case ArmResult.Forward:
                        double increment = 0;
                        if (CData.Pitch < -10)
                            increment = 1.0 * (-CData.Pitch - 10) / 5 / 10;
                        Drive(0.1+increment, 0.1+increment, ControlLevel.Brain);
                        break;
                    case ArmResult.Backward:
                        double decrement = 0;
                        if (CData.Pitch > 10)
                            increment = 1.0 * (CData.Pitch - 10) / 5 / 10;
                        Drive(-0.1-decrement, -0.1-decrement, ControlLevel.Brain);
                        break;
                    case ArmResult.Left:
                        Drive(0, 0.1, ControlLevel.Brain);
                        break;
                    case ArmResult.Right:
                        Drive(0.1, 0, ControlLevel.Brain);
                        break;
                }
            }
            if (ERH_ListenFroNonAdvoid && message == MessageType.FrontNonAdvoid)
            {
                ERH_Flag = false;
                ERH_ListenComlink = true;
                ERH_ListenRearAdvoid = true;
                ERH_ListenFroNonAdvoid = false;
            }
            if (ERH_ListenComlink && message == MessageType.Comlink)
            {
                DriveHelper(_state.current_M.position, _state.SI.ComLinkInterval, ControlLevel.Brain);
            }
            if (ERH_ListenRearAdvoid && message == MessageType.RearAdvoid)
            {
                Vector3D position = ImageAnalyseTech.BasicVision(new Vector2D(_state.RearCamData.Avoid[0].x, _state.RearCamData.Avoid[0].y), CamType.Rear);
                comlink.MarsComLinkState CData = _state.ComlinkMemory.Get(_state.RearCamData.Timestamp);
                position.x *= -1; position.z *= -1;
                position = ImageAnalyseTech.CooTran(position, Vector3D.Convert(CData.Position), new Vector3D(ToRadians(-CData.Pitch), ToRadians(-CData.Yaw), ToRadians(-CData.Roll)));
                comlink.Position p = position.ToComlink();
                if (Distance(p, _state.current_M.place) < _state.current_M.radix)
                    _state.current_M.position = p;
                ERH_ListenRearAdvoid = false;
            }
            if (ERH_ListenSpect && message == MessageType.GetData)
            {
                _state.DriveState.Level = ControlLevel.Basic;
                ERH_ListenFroAdvoid = true;
                ERH_ListenFroNonAdvoid = false;
                ERH_ListenComlink = true;
                ERH_ListenRearAdvoid = true;
                ERH_ListenSpect = true;
                ERH_Flag = false;
            }
        }

        bool AH_ListenFroAdvoid = true;
        bool AH_LestenFroNonAdvoid = false;
        bool AH_ListenRearAdvoid = false;
        bool AH_FroFlag = false;
        double AH_Choose = 0;
        /// <summary>
        /// part of brain. respond to refuge. but it only have eyes control level 
        /// </summary>
        /// <param name="message"></param>
        void AdvoidHelper(MessageType message)
        {
            if (_state.DriveState.Level > ControlLevel.Eyes)
                return;
            if (AH_ListenFroAdvoid && message == MessageType.FrontAdvoid)
            {
                if (!AH_FroFlag)
                {
                    StopRover(ControlLevel.Eyes);
                    AH_FroFlag = true;
                    double tan_left = double.MaxValue, tan_right = double.MaxValue;
                    double max_y = 0;
                    for (int i = 0; i < _state.FrontCamData.Avoid.Length; ++i)
                    {
                        tan_left = Math.Min(tan_left, _state.FrontCamData.Avoid[i].tan_left);
                        tan_right = Math.Min(tan_right, _state.FrontCamData.Avoid[i].tan_right);
                        max_y = Math.Max(max_y, _state.FrontCamData.Avoid[i].max_y);
                    }

                    //weather need backward?
                    if (max_y > 100)
                    {
                        Drive(-0.2, -0.2, ControlLevel.Eyes);
                        AH_ListenRearAdvoid = true;
                    }

                    double deta = GetAngle(_state.current_M.position);

                    if (deta > 45)
                        AH_Choose = -Turn(deta, _state.SI.FroCamInterval);
                    else if (deta < -45)
                        AH_Choose = Turn(-deta, _state.SI.FroCamInterval);
                    else if (tan_right < tan_left)
                        AH_Choose = Turn(90 - ToDegree((float)Math.Atan(tan_left)), _state.SI.FroCamInterval);
                    else
                        AH_Choose = -Turn(90 - ToDegree((float)Math.Atan(tan_right)), _state.SI.FroCamInterval);
                    return;
                }
            }
            if (AH_LestenFroNonAdvoid && message == MessageType.FrontNonAdvoid)
            {
                Drive(1, 1, ControlLevel.Eyes);
                if (!AH_FroFlag)
                {
                    AH_ListenFroAdvoid = true;
                    AH_LestenFroNonAdvoid = false;
                    AH_Choose = 0;
                    _state.DriveState.Level = ControlLevel.Basic;
                }
                AH_FroFlag = false;
            }
            if (AH_ListenRearAdvoid && message == MessageType.RearAdvoid)
            {
                Drive(0, 0, ControlLevel.Eyes);
            }
            if (AH_FroFlag)//turn
            {
                Drive(AH_Choose, -AH_Choose, ControlLevel.Eyes);
                AH_LestenFroNonAdvoid = true;
                AH_ListenRearAdvoid = false;
            }
        }

        bool CH_ListenComlink = false;
        bool CH_ListenClimb = true;
        const double CH_SafeDegree = 5;
        double CH_Px = 0;
        double CH_pz = 0;
        bool CH_Flag = false;
        /// <summary>
        /// important part of brain help rover to climb up safely
        /// </summary>
        /// <param name="message"></param>
        void ClimbHelper(MessageType message)
        {
            if (CH_ListenComlink && message == MessageType.Comlink)
            {
                comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();
                if (Math.Abs(CData.Pitch) < CH_SafeDegree && Math.Abs(CData.Roll) < CH_SafeDegree)
                {
                    CH_ListenComlink = false;
                    CH_ListenClimb = true;
                    CH_Flag = false;
                    _state.DriveState.Level = ControlLevel.Basic;
                    ArmReset();
                    return;
                }
                ImageAnalyseTech.CooTran(new Vector3D(0,1,0),new Vector3D(0,0,0),new Vector3D(ToRadians(-CData.Pitch),ToRadians(-CData.Yaw),ToRadians(-CData.Roll)));
                double sin_x = Math.Sin(ToRadians(-CData.Pitch)), sin_y = Math.Sin(ToRadians(-CData.Yaw)), sin_z = Math.Sin(ToRadians(-CData.Roll));
                double cos_x = Math.Cos(ToRadians(-CData.Pitch)), cos_y = Math.Cos(ToRadians(-CData.Yaw)), cos_z = Math.Cos(ToRadians(-CData.Roll));
                double x = cos_y * sin_z + sin_x * sin_y * cos_z, y = cos_x * cos_z, z = sin_y * sin_z - sin_x * cos_y * cos_z;//
                double len = Math.Sqrt(Math.Pow(x, 2) + Math.Pow(z, 2));
                double p_x = -x/len, p_z =-z/len;

                double angle = 90-ToDegree((float) Math.Atan(y / len));
                if (CH_Flag)
                {
                    if (!(-(x * CH_Px + z * CH_pz) / y > Math.Tan(ToRadians(30)) || -(x * CH_Px + z * CH_pz) / y < Math.Tan(ToRadians(18))))
                    {
                        p_x = CH_Px;
                        p_z = CH_pz;
                    }
                    else
                        CH_Flag = false;
                }
                if (angle >= 30 && !CH_Flag)
                {
                    int max_y = 0;
                    int num = 0;
                    for (int i = 1; i < _state.FrontCamData.Edge.Length - 1; ++i)
                    {
                        if (_state.FrontCamData.Edge[i] > max_y)
                        {
                            max_y = _state.FrontCamData.Edge[i];
                            num = i;
                        }
                    }
                    double power = Math.Min(0.8, Math.Max(0, Math.Min(_state.DriveState.Left, _state.DriveState.Right)));
                    double cos = Math.Cos(ToRadians(num >= 64 ? -1 : 1));
                    double sin = Math.Sin(ToRadians(num >= 64 ? -1 : 1));

                    do
                    {
                        double mid = p_x;
                        p_x = cos * p_x - sin * p_z;
                        p_z = sin * mid + cos * p_z;
                    } while (-(x * p_x + z * p_z) / y > Math.Tan(ToRadians(30)));
                    CH_Px = p_x;
                    CH_pz = p_z;
                    CH_Flag = true;
                }
               
                double Tan = 0;
                if (p_z != 0)
                    Tan = p_x / p_z;
                double sata = Math.Atan(Tan) / Math.PI * 180;
                if (p_z < 0)
                {
                    if (sata > 0)
                        sata -= 180;
                    else
                        sata += 180;
                }

                double deta = sata - CData.Yaw;
                if (deta < -180)
                    deta += 360;
                else if (deta > 180)
                    deta -= 360;

                if (deta > 4 )
                {
                    double turn_p = Turn(deta, _state.SI.ComLinkInterval);
                    Drive(0, 2 * turn_p, ControlLevel.Brain);
                }
                else if (deta < -4)
                {
                    double turn_p = Turn(-deta, _state.SI.ComLinkInterval);
                    Drive(2 * turn_p, 0, ControlLevel.Brain);
                }
                else
                {
                   // double power = Math.Max(0, Math.Min(_state.DriveState.Left, _state.DriveState.Right));
                   // Drive(power + 0.1, power + 0.1, ControlLevel.Brain);
                    Drive(1, 1, ControlLevel.Brain);
                }
            }
            if (CH_ListenClimb && message == MessageType.Climb)
            {
                StopRover(ControlLevel.Brain);
                CH_ListenComlink = true;
                CH_ListenClimb = false;
                _armPort.SetJointTargetPose(ShoulderVJointName, new Vector3(), new AxisAngle(Twist, ToRadians(35)));
                _armPort.SetJointTargetPose(ShoulderHJointName, new Vector3(), new AxisAngle(Twist, ToRadians(-15)));
                _armPort.SetJointTargetPose(WristHJointName, new Vector3(), new AxisAngle(Twist, ToRadians(90)));
                _armPort.SetJointTargetPose(WristVJointName, new Vector3(), new AxisAngle(Twist, ToRadians(90)));
                _armPort.SetJointTargetPose(ElbowJointName, new Vector3(), new AxisAngle(Twist, ToRadians(-10)));
            }
        }

        bool PH_ListenReach = true;
        bool PH_ListenComlink = false;
        bool PH_Flag = false;
        float PH_Yaw = 0;
        int PH_Count = 0;
        Random random = new Random((int)DateTime.Now.ToBinary());
        void PositionHelper(MessageType message)
        {
            if (PH_ListenReach && message == MessageType.ReachArea)
            {
                comlink.Position p = _state.VisionMemory.Get();
                if (p != null)
                {
                    _state.current_M.position = p;
                }
                else
                {
                    if (!PH_Flag)
                    {
                        StopRover(ControlLevel.Eyes);
                        Drive(0.1, 0, ControlLevel.Eyes);
                        PH_Flag = true;
                        comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();
                        PH_Yaw = CData.Yaw;
                        PH_Count = 0;
                        PH_ListenComlink = true;
                        PH_ListenReach = false;
                        return;
                    }
                    comlink.Position new_p=new comlink.Position();
                    new_p.X = _state.current_M.place.X + (float)((random.NextDouble() - 0.5) * _state.current_M.radix * 2);
                    new_p.Z = _state.current_M.place.Z + (float)((random.NextDouble() - 0.5) * _state.current_M.radix * 2);
                    new_p.Y = _state.current_M.place.Y;
                    _state.current_M.position = new_p;
                    PH_Flag = false;
                    _state.DriveState.Level = ControlLevel.Basic;
                }
            }
            if (PH_ListenComlink && message == MessageType.Comlink)
            {
                comlink.MarsComLinkState CData=_state.ComlinkMemory.Last();
                if (CData.Yaw < PH_Yaw && PH_Count == 0)
                    PH_Count++;
                else if (CData.Yaw > PH_Yaw && PH_Count == 1)
                    PH_Count++;
                if (PH_Count == 2)
                {
                    PH_ListenReach = true;
                    PH_ListenComlink = false;
                }
            }
        }

        bool CB_ListenComlink = true;
        const double CB_RadiansThres = 10 * Math.PI / 180;
        /// <summary>
        /// part of brain. it watch weather rover will climb up or down and take proply respond
        /// </summary>
        /// <param name="message"></param>
        void CareBody(MessageType message)
        {
            if (CB_ListenComlink && message == MessageType.Comlink)
            {
                comlink.MarsComLinkState CData = _state.ComlinkMemory.Last();
                Vector3D goal = Vector3D.Convert(_state.current_M.position);

                double sin_x = Math.Sin(ToRadians(-CData.Pitch)), sin_y = Math.Sin(ToRadians(-CData.Yaw)), sin_z = Math.Sin(ToRadians(-CData.Roll));
                double cos_x = Math.Cos(ToRadians(-CData.Pitch)), cos_y = Math.Cos(ToRadians(-CData.Yaw)), cos_z = Math.Cos(ToRadians(-CData.Roll));
                double x = cos_y * sin_z + sin_x * sin_y * cos_z, y = cos_x * cos_z, z = sin_y * sin_z - sin_x * cos_y * cos_z;//

                double tan = -z / x;
                double value = goal.x - CData.Position.X - tan * (goal.z - CData.Position.Z);
                double value_up = goal.x - CData.Position.X - Math.Tan(Math.Atan(tan) + CB_RadiansThres) * (goal.z - CData.Position.Z);
                double value_down = goal.x - CData.Position.X - Math.Tan(Math.Atan(tan) - CB_RadiansThres) * (goal.z - CData.Position.Z);
                double value2 = x - tan * z;

                if (value_up * value_down < 0)
                {
                    //change pose to advoid unexpected things
                    double deta= GetAngle(_state.current_M.position);
                    if (Math.Abs(deta) > 10)
                        return;
                    double x_x = cos_y * cos_z - sin_x * sin_y * sin_z, x_z = sin_y * cos_z + sin_x * cos_y * sin_z;
                    int suggust = _state.Memory.Suggust(Vector3D.Convert(CData.Position), new Vector2D(x_x, x_z));
                    if (suggust == 1)
                        Drive(0.8, 1, ControlLevel.Eyes);
                    else
                        Drive(1, 0.8, ControlLevel.Eyes);
                }
                else if (value2 * value < 0) //rover will go up
                {
                    ClimbHelper(MessageType.Climb);
                }
                else //value2*value>0  //rover will go down
                {
                    if (!CH_ListenClimb)
                    {
                        CH_ListenComlink = false;
                        CH_ListenClimb = true;
                        CH_Flag = false;
                        _state.DriveState.Level = ControlLevel.Basic;
                        ArmReset();
                        return;
                    }
                    //need to go down?
                    //here was planed to add some path plan
                }
            }
        }

        #endregion

    }
}
