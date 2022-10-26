using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RoboticsFKIK
{
    public class MovementCalculation
    {
        public static Matrix getTransMatrix(int from, int to, double[] degrees, Robot robot)
        {
            if (degrees.Length == robot.armCount)
            {
                Matrix m = new Matrix.Identity(4);
                for (int i = from; i < to; i++)
                {
                    m *= getT(degToRad(degrees[i] + robot.offsetArray[i]), degToRad(robot.alphaArray[i]), robot.dArray[i], robot.aArray[i]);
                }
                return m;
            }
            else
            {
                throw new Exception("运动数据与关节数量不匹配");
            }
        }
        public static Matrix.Vector getPosDeg(Matrix TransMatrix)
        {
            Matrix.Vector posDeg = new Matrix.Vector(new double[6]);
            if (TransMatrix.Rows == 4 && TransMatrix.Cols == 4)
            {
                if (TransMatrix[3, 0] == 0 && TransMatrix[3, 1] == 0 && TransMatrix[3, 2] == 0 && TransMatrix[3, 3] == 1)
                {
                    posDeg[0] = TransMatrix[0, 3];
                    posDeg[1] = TransMatrix[1, 3];
                    posDeg[2] = TransMatrix[2, 3];

                    Matrix.Vector eu = rotationMatrixToEulerAngles(TransMatrix.getChildMatrix(0, 0, 2, 2));
                    posDeg[3] = radToDeg(eu[0]);
                    posDeg[4] = radToDeg(eu[1]);
                    posDeg[5] = radToDeg(eu[2]);
                    return posDeg;
                }
            }
            throw new Exception("不是合规的转换矩阵");
        }
        public static Matrix posDegToTransM(Matrix.Vector posDeg)
        {
            Matrix.Vector theta = new Matrix.Vector(new double[] { degToRad(posDeg[3]), degToRad(posDeg[4]), degToRad(posDeg[5]) });
            Matrix rot = eulerAnglesToRotationMatrix(theta);
            Matrix trans = new Matrix.Identity(4);
            for (int i = 0; i < rot.Rows; i++)
            {
                for (int j = 0; j < rot.Cols; j++)
                {
                    trans[i, j] = rot[i, j];
                }
            }
            trans[0, 3] = posDeg[0];
            trans[1, 3] = posDeg[1];
            trans[2, 3] = posDeg[2];
            return trans;
        }
        public static bool isRotationMatirx(Matrix R)
        {
            if (R.Cols == 3 && R.Rows == 3)
            {
                double err = 1e-6;
                Matrix shouldIdenity = R * R.Transpose();
                Matrix I = new Matrix.Identity(3);
                return (shouldIdenity - I).norm() < err;
            }
            else
            {
                return false;
            }

        }
        public static Matrix.Vector rotationMatrixToEulerAngles(Matrix R)
        {
            if (isRotationMatirx(R))
            {
                double sy = Math.Sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]);
                bool singular = sy < 1e-6;
                double x, y, z;
                if (!singular)
                {
                    x = Math.Atan2(R[2, 1], R[2, 2]);
                    y = Math.Atan2(-R[2, 0], sy);
                    z = Math.Atan2(R[1, 0], R[0, 0]);
                }
                else
                {
                    x = Math.Atan2(-R[1, 2], R[1, 1]);
                    y = Math.Atan2(-R[2, 0], sy);
                    z = 0;
                }
                return new Matrix.Vector(new double[] { x, y, z });
            }
            else
            {
                throw new NotImplementedException();
            }

        }
        /// <summary>
        /// 欧拉角到旋转矩阵
        /// </summary>
        /// <param name="theta">radian</param>
        /// <returns></returns>
        public static Matrix eulerAnglesToRotationMatrix(Matrix.Vector theta)
        {
            Matrix R_x;    // 计算旋转矩阵的X分量
            R_x = new Matrix(new double[,]
            {
                 {   1,              0,               0 },
            { 0,  Math.Cos(theta[0]),  -Math.Sin(theta[0]) },
           { 0,  Math.Sin(theta[0]),   Math.Cos(theta[0]) },
            });

            Matrix R_y;    // 计算旋转矩阵的Y分量
            R_y = new Matrix(new double[,]
            {
                    { Math.Cos(theta[1]),   0, Math.Sin(theta[1]) },
           { 0,   1,             0 },
            { -Math.Sin(theta[1]),  0, Math.Cos(theta[1]) },
            });

            Matrix R_z;    // 计算旋转矩阵的Z分量
            R_z = new Matrix(new double[,]
            {
              { Math.Cos(theta[2]), -Math.Sin(theta[2]), 0 },
            { Math.Sin(theta[2]),  Math.Cos(theta[2]), 0 },
            { 0,              0,             1 }
            });
            Matrix R = R_z * R_y * R_x;
            return R;
        }
        /// <summary>
        /// standard
        /// </summary>
        /// <param name="theta"></param>
        /// <param name="alpha"></param>
        /// <param name="d"></param>
        /// <param name="a"></param>
        /// <returns></returns>
        public static Matrix getT(double theta, double alpha, double d, double a)
        {
            Matrix rotTheta = new Matrix(new double[,]
           {
                {Math.Cos(theta), -Math.Sin(theta), 0, 0},
                {Math.Sin(theta), Math.Cos(theta), 0,0},
                {0, 0, 1,0},
                {0, 0, 0,1},
            });
            Matrix transDis = new Matrix(new double[,]
           {
                {1, 0, 0,0},
                 {0, 1, 0,0},
                {0, 0, 1,d},
                {0, 0, 0,1},
            });
            Matrix transA = new Matrix(new double[,]
          {
                {1, 0, 0,a},
                 {0, 1, 0,0},
                {0, 0, 1,0},
                {0, 0, 0,1},
           });
            Matrix rotAlpha = new Matrix(new double[,]
           {
                {1, 0, 0, 0},
                {0, Math.Cos(alpha), -Math.Sin(alpha),0},
                {0, Math.Sin(alpha), Math.Cos(alpha),0},
                {0, 0, 0,1},
            });

            Matrix matrix = rotTheta * transDis * transA * rotAlpha;
            return matrix;
        }

        public static List<double[]> cal6ArmSolutions(Matrix transM, Robot robot)
        {
            List<double[]> sovledRads = new List<double[]>();
            double[] solution = new double[robot.armCount];
            sovledRads.Add(solution);
            calRadian1(transM, robot, ref sovledRads);
            calRadian5(transM, ref sovledRads);
            calRadian6(transM, ref sovledRads);
            calRadian234(transM, robot, ref sovledRads);
            return sovledRads;
        }
        private static void calRadian1(Matrix transM, Robot robot, ref List<double[]> sovledRads)
        {
            int count = sovledRads.Count;
            for (int i = 0; i < count; i++)
            {
                double A1 = robot.dArray[5] * transM[1, 2] - transM[1, 3];
                double B1 = transM[0, 3] - robot.dArray[5] * transM[0, 2];
                double C1 = robot.dArray[3];
                double radian_1 = Math.Atan2(C1, Math.Sqrt(A1 * A1 + B1 * B1 - C1 * C1)) - Math.Atan2(A1, B1);
                double radian_2 = Math.Atan2(C1, -Math.Sqrt(A1 * A1 + B1 * B1 - C1 * C1)) - Math.Atan2(A1, B1);

                double[] newSolution = (double[])(sovledRads[i].Clone());
                sovledRads[i][0] = radian_1;
                newSolution[0] = radian_2;
                sovledRads.Add(newSolution);
            }
        }
        private static void calRadian5(Matrix transM, ref List<double[]> sovledRads)
        {
            int count = sovledRads.Count;
            for (int i = 0; i < count; i++)
            {
                double cos5 = transM[0, 2] * Math.Sin(sovledRads[i][0]) - transM[1, 2] * Math.Cos(sovledRads[i][0]);
                double sin5 = Math.Sqrt(1 - cos5 * cos5);
                double rad5_1 = Math.Atan2(sin5, cos5);
                double rad5_2 = Math.Atan2(-sin5, cos5);
                double[] newSolution = (double[])(sovledRads[i].Clone());
                sovledRads[i][4] = rad5_1;
                newSolution[4] = rad5_2;
                sovledRads.Add(newSolution);
            }
        }

        private static void calRadian6(Matrix transM, ref List<double[]> sovledRads)
        {
            int count = sovledRads.Count;
            for (int i = 0; i < count; i++)
            {
                double A6 = (transM[0, 1] * Math.Sin(sovledRads[i][0]) - transM[1, 1] * Math.Cos(sovledRads[i][0])) / Math.Sin(sovledRads[i][4]);
                double B6 = -(transM[0, 0] * Math.Sin(sovledRads[i][0]) - transM[1, 0] * Math.Cos(sovledRads[i][0])) / Math.Sin(sovledRads[i][4]);
                sovledRads[i][5] = Math.Atan2(A6, B6);
            }
        }

        private static void calRadian234(Matrix transM, Robot robot, ref List<double[]> sovledRads)
        {
            int count = sovledRads.Count;
            for (int i = 0; i < count; i++)
            {
                double A234 = transM[2, 2] / Math.Sin(sovledRads[i][4]);
                double B234 = (transM[0, 2] * Math.Cos(sovledRads[i][0]) + transM[1, 2] * Math.Sin(sovledRads[i][0])) / Math.Sin(sovledRads[i][4]);

                //joint2
                double M2 = robot.dArray[4] * A234 - robot.dArray[5] * Math.Sin(sovledRads[i][4]) * B234 + transM[0, 3] * Math.Cos(sovledRads[i][0]) + transM[1, 3] * Math.Sin(sovledRads[i][0]);
                double N2 = -robot.dArray[4] * B234 - robot.dArray[5] * Math.Sin(sovledRads[i][4]) * A234 - robot.dArray[0] + transM[2, 3];
                double L2 = (M2 * M2 + N2 * N2 + robot.aArray[1] * robot.aArray[1] - robot.aArray[2] * robot.aArray[2]) / (2 * robot.aArray[1]);

                double rad2_1 = Math.Atan2(N2, M2) - Math.Atan2(L2, Math.Sqrt(M2 * M2 + N2 * N2 - L2 * L2));
                double rad2_2 = Math.Atan2(N2, M2) - Math.Atan2(L2, -Math.Sqrt(M2 * M2 + N2 * N2 - L2 * L2));
                double[] newSolution = (double[])(sovledRads[i].Clone());
                sovledRads[i][1] = rad2_1;
                newSolution[1] = rad2_2;

                //joint3
                double A23 = (-M2 - robot.aArray[1] * Math.Sin(rad2_1)) / robot.aArray[2];
                double B23 = (N2 - robot.aArray[1] * Math.Cos(rad2_1)) / robot.aArray[2];
                sovledRads[i][2] = rad2_1 - Math.Atan2(A23, B23);

                double A23_2 = (-M2 - robot.aArray[1] * Math.Sin(rad2_2)) / robot.aArray[2];
                double B23_2 = (N2 - robot.aArray[1] * Math.Cos(rad2_2)) / robot.aArray[2];
                newSolution[2] = rad2_2 - Math.Atan2(A23_2, B23_2);

                //joint4
                sovledRads[i][3] = Math.Atan2(A234, B234) - Math.Atan2(A23, B23);
                newSolution[3] = Math.Atan2(A234, B234) - Math.Atan2(A23_2, B23_2);

                sovledRads.Add(newSolution);
            }


        }

        public static double radToDeg(double radian)
        {
            return radian * 180 / Math.PI;
        }
        public static double degToRad(double Degree)
        {
            return Degree * Math.PI / 180.0;
        }
    }
}
