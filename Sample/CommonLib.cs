using System.Collections.Generic;
using System;

using Microsoft.Robotics.RoboChamps.MarsChallenger.ImageProcessing;
using comlink = Microsoft.Robotics.RoboChamps.MarsComLink.Proxy;
using webcam = Microsoft.Robotics.Services.WebCam.Proxy;
using Microsoft.Robotics.RoboChamps.MarsChallenger.StereoVision;

namespace Microsoft.Robotics.RoboChamps.MarsChallenger.CommonLib
{
    public class CommonLib
    {
        comlink.Position RollwithAxis(comlink.Position P, comlink.Position A, double R)//|A|==1
        {
            comlink.Position res = new comlink.Position();
            float PA = P.X * A.X + P.Y * A.Y + P.Z * A.Z;
            res.X = (float)(P.X * Math.Cos(R) + (P.Z * A.Y - A.Z * P.Y) * Math.Sin(R) + A.X * PA * (1 - Math.Cos(R)));
            res.Y = (float)(P.Y * Math.Cos(R) + (A.Z * P.X - A.X * P.Z) * Math.Sin(R) + A.Y * PA * (1 - Math.Cos(R)));
            res.Z = (float)(P.Z * Math.Cos(R) + (A.X * P.Y - A.Y * P.X) * Math.Sin(R) + A.Z * PA * (1 - Math.Cos(R)));

            return res;
        }

        static System.Drawing.Color color(double d)
        {
            double r, g, b;
            if (d < 1.0 / 6)
            {
                b = d * 6; r = 0; g = 0;
            }
            else if (d < 2.0 / 6)
            {
                b = 1; r = 0; g = (d - 1.0 / 6) * 6;
            }
            else if (d < 1.0 / 2)
            {
                b = 1 - (d - 2.0 / 6) * 6; r = 0; g = 1;
            }
            else if (d < 4.0 / 6)
            {
                b = 0; r = (d - 3.0 / 6) * 6; g = 1;
            }
            else if (d < 5.0 / 6)
            {
                b = 0; r = 1; g = 1 - (d - 4.0 / 6) * 6;
            }
            else
            {
                r = 1;
                b = (d - 5.0 / 6) * 6;
                g = (d - 5.0 / 6) * 6;
            }
            return System.Drawing.Color.FromArgb((int)(255.99 * r), (int)(255.99 * g), (int)(255.99 * b));
        }

        public static double ToDegree(float radians)
        {
            return (radians * 180.0d / Math.PI);
        }
        
        public static double ToRadians(float degree)
        {
            return (degree * Math.PI / 180.0d);
        }
   
        static int map_count = 0;
        //use to output images. file path need to change
        public static void DrawImage(float[,] map)
        {
            if (map == null)
                return;
            float max = map[0, 0], min = map[0, 0];
            for (int i = 0; i < map.GetLength(0); ++i)
                for (int j = 0; j < map.GetLength(1); ++j)
                {
                    //if (map[i, j] > -10 && map[i, j] < 10)
                    {
                        max = Math.Max(map[i, j], max);
                        min = Math.Min(map[i, j], min);
                    }
                }
            System.Drawing.Bitmap image = new System.Drawing.Bitmap(map.GetLength(0), map.GetLength(1));
            for (int i = 0; i < map.GetLength(0); ++i)
                for (int j = 0; j < map.GetLength(1); ++j)
                {
                    image.SetPixel(i, j, color((map[i, j] - min) / (max - min)));
                }
            map_count++;
            image.Save("C:\\Users\\zerolin\\Microsoft Robotics Dev Studio 2008\\RoboChamps\\Mars\\images\\map" + map_count + ".png");
        }
        public static void DrawImage(float[,] map, LinkedList<VisionData> VD,float Center_x , float Center_z )
        {
            if (map == null)
                return;
            float max = map[0, 0], min = map[0, 0];
            for (int i = 0; i < map.GetLength(0); ++i)
                for (int j = 0; j < map.GetLength(1); ++j)
                {
                    //if (map[i, j] > -10 && map[i, j] < 10)
                    {
                        max = Math.Max(map[i, j], max);
                        min = Math.Min(map[i, j], min);
                    }
                }
            System.Drawing.Bitmap image = new System.Drawing.Bitmap(map.GetLength(0), map.GetLength(1));
            for (int i = 0; i < map.GetLength(0); ++i)
                for (int j = 0; j < map.GetLength(1); ++j)
                {
                    image.SetPixel(i, j, color((map[i, j] - min) / (max - min)));
                }

            foreach (VisionData vd in VD)
            {
                if (!(vd.Position.X < 1000 && vd.Position.Y < 1000))
                    continue;
                int x = (int)Math.Round((vd.Position.X - Center_x) / Memory.Unit);
                int z = (int)Math.Round((vd.Position.Z - Center_z) / Memory.Unit);
                if (Math.Abs(x) > Memory.Size || Math.Abs(z) > Memory.Size || vd.Score > 400)
                    continue;
                if (vd.Score <= 100)
                    image.SetPixel(x + Memory.Size, z + Memory.Size, System.Drawing.Color.Chocolate);
                else if (vd.Score <= 200)
                    image.SetPixel(x + Memory.Size, z + Memory.Size, System.Drawing.Color.Purple);
                else
                    image.SetPixel(x + Memory.Size, z + Memory.Size, System.Drawing.Color.DarkGray);
            }

            map_count++;
            image.Save("C:\\Users\\zerolin\\Microsoft Robotics Dev Studio 2008\\RoboChamps\\Mars\\images\\map" + map_count + ".png");
        }
        public static void DrawImage(webcam.QueryFrameResponse image, Blob[] blobs)
        {
            int offset = 0;
            System.Drawing.Bitmap map = new System.Drawing.Bitmap(image.Size.Width, image.Size.Height);

            for (int y = 0; y < image.Size.Height; ++y)
            {
                offset = y * image.Size.Width * 3;
                for (int x = 0; x < image.Size.Width; ++x)
                {
                    int r, g, b;
                    b = image.Frame[offset++];
                    g = image.Frame[offset++];
                    r = image.Frame[offset++];
                    map.SetPixel(x, y, System.Drawing.Color.FromArgb(r, g, b));
                }
            }

            for (int i = 0; i < blobs.Length; ++i)
            {

                System.Drawing.Color col = color(blobs[i].Rock_Pro);
                for (int j = blobs[i].min_x; j <= blobs[i].max_x; ++j)
                {
                    map.SetPixel(j, blobs[i].max_y, col);
                    map.SetPixel(j, blobs[i].min_y, col);
                }
                for (int j = blobs[i].min_y; j < blobs[i].max_y; ++j)
                {
                    map.SetPixel(blobs[i].min_x, j, col);
                    map.SetPixel(blobs[i].max_x, j, col);
                }
                map.SetPixel(blobs[i].x, blobs[i].y, color(blobs[i].Shield_Pro));
            }
            map_count++;
            map.Save("C:\\Users\\zerolin\\Microsoft Robotics Dev Studio 2008\\RoboChamps\\Mars\\images\\blobmap" + map_count + ".bmp");
        }
        public static void DrawImage(webcam.QueryFrameResponse pan,webcam.QueryFrameResponse nav,int center_x,int center_y) 
        {
            int offset = 0;
            System.Drawing.Bitmap map = new System.Drawing.Bitmap(nav.Size.Width, nav.Size.Height);

            int w = (int)(nav.Size.Width );
            int h = (int)(nav.Size.Height );
            int[,] R = new int[w, h];
            int[,] G = new int[w, h];
            int[,] B = new int[w, h];
            int[,] count = new int[w, h];

            int pan_x = pan.Size.Width / 2;
            int pan_y = pan.Size.Height / 2;
            for (int y = 0; y < pan.Size.Height; ++y)
            {
                offset = y * pan.Size.Width * 3;
                for (int x = 0; x < pan.Size.Width; ++x)
                {
                    int r, g, b;
                    b = pan.Frame[offset++];
                    g = pan.Frame[offset++];
                    r = pan.Frame[offset++];
                    int x_nav = center_x;
                    int y_nav = center_y;
                    if (y != pan_y || x != pan_x)
                    {
                        double len_pan = Math.Sqrt(Math.Pow(y - pan_y, 2) + Math.Pow(x - pan_x, 2));
                        double sin = (y - pan_y) / len_pan;
                        double cos = (x - pan_x) / len_pan;
                        double len_nav = len_pan * ImageAnalyseTech.NavCam.FocusLen / ImageAnalyseTech.PanCam.FocusLen;
                        //...
                        x_nav = (int)Math.Round(len_nav * cos + center_x);
                        y_nav = (int)Math.Round(len_nav * sin + center_y);
                    }
                    R[x_nav, y_nav] += r;
                    G[x_nav, y_nav] += g;
                    B[x_nav, y_nav] += b;
                    count[x_nav, y_nav] ++;
                }
            }

            for (int y = 0; y < nav.Size.Height; ++y)
            {
                for (int x = 0; x < nav.Size.Width; ++x)
                {
                    offset = y * nav.Size.Width * 3+x*3;
                    if (count[x, y] == 0)
                    {
                        int r, g, b;
                        b = nav.Frame[offset++];
                        g = nav.Frame[offset++];
                        r = nav.Frame[offset++];
                        map.SetPixel(x, y, System.Drawing.Color.FromArgb(r, g, b));
                    }
                    else
                    {
                        map.SetPixel(x, y, System.Drawing.Color.FromArgb(R[x, y] / count[x, y], G[x, y] / count[x, y], B[x, y] / count[x, y]));
                    }
                }
            }


            map_count++;
            map.Save("C:\\Users\\zerolin\\Microsoft Robotics Dev Studio 2008\\RoboChamps\\Mars\\images\\blobmap" + map_count + ".bmp");
        }
        public static void DrawImage(webcam.QueryFrameResponse image, target[] target, int[] edge)
        {
            int offset = 0;
            System.Drawing.Bitmap map = new System.Drawing.Bitmap(image.Size.Width, image.Size.Height);

            for (int y = 0; y < image.Size.Height; ++y)
            {
                offset = y * image.Size.Width * 3;
                for (int x = 0; x < image.Size.Width; ++x)
                {
                    int r, g, b;
                    b = image.Frame[offset++];
                    g = image.Frame[offset++];
                    r = image.Frame[offset++];
                    map.SetPixel(x, y, System.Drawing.Color.FromArgb(r, g, b));
                }
            }

            for (int i = 0; i < target.Length; ++i)
            {

                System.Drawing.Color col = System.Drawing.Color.Gold;
                for (int j = target[i].min_x; j <= target[i].max_x; ++j)
                {
                    map.SetPixel(j, target[i].max_y, col);
                    map.SetPixel(j, target[i].min_y, col);
                }
                for (int j = target[i].min_y; j < target[i].max_y; ++j)
                {
                    map.SetPixel(target[i].min_x, j, col);
                    map.SetPixel(target[i].max_x, j, col);
                }
                map.SetPixel(target[i].x, target[i].y, System.Drawing.Color.Red);
            }

            for (int i = 1; i < edge.Length-1; ++i)
            {
                map.SetPixel(i, edge[i], System.Drawing.Color.Green);
            }
                map_count++;
            map.Save("C:\\Users\\zerolin\\Microsoft Robotics Dev Studio 2008\\RoboChamps\\Mars\\images\\basicmap" + map_count + ".bmp");
        }
    }
    
    public class CameraPrm
    {
        public int Lenth { get; set; }
        public int Width { get; set; }
        public double ViewAngle { get; set; }
        public double FocusLen { get; set; }
        public double P_x { get; set; }
        public double P_y { get; set; }
        public double P_z { get; set; }
        public double O_x { get; set; }
        public double O_y { get; set; }
        public double O_z { get; set; }
        public CameraPrm(int lenth, int width, double viewangle, double p_x, double p_y, double p_z, double o_x, double o_y, double o_z)
        {
            Lenth = lenth;
            Width = width;
            ViewAngle = viewangle;
            FocusLen = Math.Sqrt(Math.Pow(Lenth / 2, 2) + Math.Pow(Width / 2, 2)) / Math.Tan(ViewAngle / 2);
            P_x = p_x;
            P_y = p_y;
            P_z = p_z;
            O_x = o_x;
            O_y = o_y;
            O_z = o_z;
        }
    }
    public enum CamType
    {
        Nav,
        Pan,
        Front,
        Rear,
    }
    
    public class Vector2D
    {
        public Vector2D() { }
        public Vector2D(double X, double Y) { x = X; y = Y; }
        public double x { get; set; }
        public double y { get; set; }
    }
    public class Vector3D
    {
        public Vector3D() { }
        public Vector3D(double X, double Y, double Z) { x = X; y = Y; z = Z; }
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public static double Length(Vector3D v3) { return Math.Sqrt(Math.Pow(v3.x, 2) + Math.Pow(v3.y, 2) + Math.Pow(v3.z, 2)); }
        public static double Distance(Vector3D from, Vector3D to) { return Math.Sqrt(Math.Pow(to.x-from.x, 2) + Math.Pow(to.y-from.y, 2) + Math.Pow(to.z-from.z, 2)); }
        public static double Distance2D(Vector3D from, Vector3D to){return Math.Sqrt(Math.Pow(to.x - from.x, 2) + Math.Pow(to.z - from.z, 2));}
        public static Vector3D Convert(comlink.Position P) { return new Vector3D(P.X, P.Y, P.Z); }
        public comlink.Position ToComlink() { comlink.Position p = new comlink.Position(); p.X = (float)x; p.Y = (float)y; p.Z = (float)z; return p; }
    }

    public class Matrix
    {
        public double[,] data { get; set; }
        public Matrix(double[,] matrix)
        {
            data = (double[,])matrix.Clone();
        }
        public Matrix(int len, int wid)
        {
            data = new double[len, wid];
        }
        public Matrix(double m11, double m12, double m13, double m14, double m21, double m22, double m23, double m24, double m31, double m32, double m33, double m34, double m41, double m42, double m43, double m44)
        {
            data = new double[4, 4];
            data[0, 0] = m11; data[0, 1] = m12; data[0, 2] = m13; data[0, 3] = m14;
            data[1, 0] = m21; data[1, 1] = m22; data[1, 2] = m23; data[1, 3] = m24;
            data[2, 0] = m31; data[2, 1] = m32; data[2, 2] = m33; data[2, 3] = m34;
            data[3, 0] = m41; data[3, 1] = m42; data[3, 2] = m43; data[3, 3] = m44;
        }
        public Matrix(double m11, double m21, double m31, double m41)
        {
            data = new double[4, 1];
            data[0, 0] = m11;
            data[1, 0] = m21;
            data[2, 0] = m31;
            data[3, 0] = m41;
        }
        public static Matrix matrix(double p_x, double p_y, double p_z, double o_x, double o_y, double o_z)
        {
            double sin_x = Math.Sin(o_x), sin_y = Math.Sin(o_y), sin_z = Math.Sin(o_z);
            double cos_x = Math.Cos(o_x), cos_y = Math.Cos(o_y), cos_z = Math.Cos(o_z);
            return new Matrix(cos_y * cos_z - sin_x * sin_y * sin_z, cos_y * sin_z + sin_x * sin_y * cos_z, -sin_y * cos_x,  p_x,
                              -cos_x * sin_z,                        cos_x * cos_z,                         sin_x,           p_y,
                              sin_y * cos_z + sin_x * cos_y * sin_z, sin_y * sin_z - sin_x * cos_y * cos_z, cos_x * cos_y,   p_z,
                              0,                                       0 ,                                  0,               1);
        }
        public static Matrix revmatrix(Matrix m)
        {
            Matrix res = new Matrix(m.data.GetLength(1), m.data.GetLength(0));
            for (int i = 0; i < m.data.GetLength(0); ++i)
                for (int j = 0; j < m.data.GetLength(1); ++j)
                    res.data[j, i] = m.data[i, j];
            return res;
        }
        public static Matrix MatrixMulti(Matrix m1, Matrix m2)
        {
            if (m1.data.GetLength(1) != m2.data.GetLength(0))
                return null;
            Matrix res = new Matrix(m1.data.GetLength(0), m2.data.GetLength(1));
            for (int i = 0; i < res.data.GetLength(0); ++i)
                for (int j = 0; j < res.data.GetLength(1); ++j)
                {
                    res.data[i, j] = 0;
                    for (int k = 0; k < m1.data.GetLength(1); ++k)
                        res.data[i, j] += m1.data[i, k] * m2.data[k, j];
                }
            return res;
        }
    }
    
    /// <summary>
    /// very important class. take charge of all coordinate translate
    /// </summary>
    public class ImageAnalyseTech
    {
        //parameters disassambly from eviroment engine xml
        public static double[] HeadPosition = { 0, 0.625+0.05, -0.42 };
        public static CameraPrm NavCam = new CameraPrm(128, 128, 0.7853982, -0.1, 0, 0, 0, 0, 0);
        public static CameraPrm PanCam = new CameraPrm(128, 128, 0.28, 0.1, 0, 0, 0, 0, 0);
        static CameraPrm FroCam = new CameraPrm(128, 128, 2.09, 0, -0.13, -0.6, -30 * Math.PI / 180, 0, 0);
        static CameraPrm RearCam = new CameraPrm(128, 128, 2.09, 0, -0.13, 0.7, 30 * Math.PI / 180, 180 * Math.PI / 180, 0);
        static double BasicCamHeigh = 0.585 - 0.13;

        static Matrix PanToNav = new Matrix(1, 0, 0, 0.2,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1);
        static Matrix FroToRov = Matrix.matrix(FroCam.P_x, FroCam.P_y, FroCam.P_z, -FroCam.O_x, -FroCam.O_y, -FroCam.O_z);
        static Matrix RearToRov = Matrix.matrix(RearCam.P_x, RearCam.P_y, RearCam.P_z, -RearCam.O_x, -RearCam.O_y, -RearCam.O_z);

        /// <summary>
        /// return the position in Panaramic camera's coordinate
        /// </summary>
        /// <param name="PanPix"></param>
        /// <param name="NavPix"></param>
        /// <returns></returns>
        public static Vector3D StereoVision(Vector2D PanPix, Vector2D NavPix)
        {
            Vector2D panpix = ToSphericalCoo(ToImageCoo(PanPix), CamType.Pan);
            Vector2D navpix = ToSphericalCoo(ToImageCoo(NavPix), CamType.Nav);
            Vector3D res = new Vector3D();
            double[,] a = { { -1 ,0,Math.Tan( panpix.x)},
                          {0,-1,Math.Tan(panpix.y)*Math.Sqrt(Math.Pow(Math.Tan(panpix.x),2)+1)},
                          {-1,0,Math.Tan(navpix.x)},
                          {0,-1,Math.Tan(navpix.y)*Math.Sqrt(Math.Pow(Math.Tan(navpix.x),2)+1)}};
            Matrix A = new Matrix(a);
            A = Matrix.MatrixMulti(Matrix.revmatrix(A),A);
            double value=(-A.data[0, 2] * A.data[0, 2] / 2 - A.data[1, 2] * A.data[1, 2] / 2 + A.data[2, 2]);
            res.z = float.MaxValue;
            if (value != 0)
                res.z = (-0.2 * Math.Tan(navpix.x) - 0.1 * A.data[0, 2]) / value;
            res.x = (0.2 - res.z * A.data[0, 2]) / 2;
            res.y = -A.data[1, 2] * res.z / 2;

            return res;
        }

        /// <summary>
        /// basic vision assumme rover is on a plan, 
        /// then count the positon for specific pixel. 
        /// </summary>
        /// <param name="Pixel"></param>
        /// <param name="Type"></param>
        /// <returns></returns>
        public static Vector3D BasicVision(Vector2D Pixel, CamType Type)
        {
            Vector2D pixel = ToSphericalCoo(ToImageCoo(Pixel), Type);
            Vector3D position_a = new Vector3D(Math.Cos(pixel.y) * Math.Sin(pixel.x), Math.Sin(pixel.y), -Math.Cos(pixel.y) * Math.Cos(pixel.x));//r=1
            Vector3D res = new Vector3D();
            res.z = -2 * BasicCamHeigh / (Math.Sqrt(3) * position_a.y / position_a.z + 1);
            res.y = position_a.y / position_a.z * res.z;
            res.x = position_a.x / position_a.z * res.z;
            res = BasicToRoverCoo(res, Type);
            return res;
        }
        
        public static Vector2D ToImageCoo(Vector2D pixel)
        {
            Vector2D res = new Vector2D();
            res.x = pixel.x - 64;
            res.y = -pixel.y + 64;
            return res;
        }
        public static Vector2D ToSphericalCoo(Vector2D pixel, CamType c_t)
        {
            Vector2D res = new Vector2D();
            double d = 0;
            switch (c_t)
            {
                case CamType.Nav:
                    d = NavCam.FocusLen;
                    break;
                case CamType.Pan:
                    d = PanCam.FocusLen;
                    break;
                case CamType.Front:
                    d = FroCam.FocusLen;
                    break;
                case CamType.Rear:
                    d = RearCam.FocusLen;
                    break;
            }
            res.x = Math.Atan(pixel.x / d);
            res.y = Math.Atan(pixel.y * Math.Cos(res.x) / d);
            return res;
        }
       
        public static Vector3D BasicToRoverCoo(Vector3D pixel, CamType c_t)
        {
            Matrix res_m = null;
            switch (c_t)
            {
                case CamType.Front:
                    res_m = Matrix.MatrixMulti(FroToRov
                                           , new Matrix(pixel.x, pixel.y, pixel.z, 1));
                    break;
                case CamType.Rear:
                    res_m = Matrix.MatrixMulti(RearToRov
                                           , new Matrix(pixel.x, pixel.y, pixel.z, 1));
                    break;
                default:
                    return null;
            }
            Vector3D res = new Vector3D();
            res.x = res_m.data[0, 0];
            res.y = res_m.data[1, 0];
            res.z = res_m.data[2, 0];
            return res;
        }
        public static Vector3D PanToRoverCoo(Vector3D pixel, double Head_H, double Head_V)
        {
            Matrix res_m = null;
            double head_x = pixel.x + 0.1;//to head coodination
            //head to rover
            res_m = Matrix.MatrixMulti(Matrix.matrix(HeadPosition[0], HeadPosition[1], HeadPosition[2], -Head_V, Head_H, 0)
                                            , new Matrix(head_x, pixel.y, pixel.z, 1));
            Vector3D res = new Vector3D();
            res.x = res_m.data[0, 0];
            res.y = res_m.data[1, 0];
            res.z = res_m.data[2, 0];
            return res;
        }
        
        public static Vector3D CooTran(Vector3D pixel, Vector3D Position, Vector3D Oritation)
        {
            Matrix res_m = null;
            res_m = Matrix.MatrixMulti(Matrix.matrix(Position.x, Position.y, Position.z, Oritation.x, Oritation.y, Oritation.z)
                                            , new Matrix(pixel.x, pixel.y, pixel.z, 1));
            Vector3D res = new Vector3D();
            res.x = res_m.data[0, 0];
            res.y = res_m.data[1, 0];
            res.z = res_m.data[2, 0];
            return res;
        }
    }

}
