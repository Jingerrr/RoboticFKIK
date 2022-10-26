using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RoboticsFKIK
{
    public class Robot
    {
        public double[] offsetArray { get; }
        public double[] alphaArray { get; }
        public double[] dArray { get; }
        public double[] aArray { get; }
        public int armCount { get; }
        public Robot(Matrix dhTable)
        {
            if (dhTable.Cols == 4)
            {
                aArray = dhTable.getCol(0).element;
                alphaArray = dhTable.getCol(1).element;
                dArray = dhTable.getCol(2).element;
                offsetArray = dhTable.getCol(3).element;
                armCount = dhTable.Rows;
            }
            else
            {
                throw new Exception("错误的DH表格式：" + dhTable.Rows.ToString() + "*" + dhTable.Cols.ToString());
            }
        }
        public RobotMovement forwardKinematics(double[] degrees)
        {
            RobotMovement robotMovement = new RobotMovement(this, degrees);
            return robotMovement;
        }
        public List<RobotMovement> inverseKinematics(double[] posDeg)
        {
            List<RobotMovement> movements = new List<RobotMovement>();

            Matrix transM = MovementCalculation.posDegToTransM(new Matrix.Vector(posDeg));

            List<double[]> sovledRads = MovementCalculation.cal6ArmSolutions(transM, this);
            List<double[]> sovledDegs = new List<double[]>();
            for (int i = 0; i < sovledRads.Count; i++)
            {
                sovledDegs.Add(new double[sovledRads[i].Length]);
                for (int j = 0; j < sovledRads[i].Length; j++)
                {
                    if (sovledRads[i][j] > Math.PI)
                    {
                        sovledRads[i][j] = sovledRads[i][j] - 2 * Math.PI;
                    }
                    else if (sovledRads[i][j] < -Math.PI)
                    {
                        sovledRads[i][j] = sovledRads[i][j] + 2 * Math.PI;
                    }

                    sovledDegs[i][j] = MovementCalculation.radToDeg(sovledRads[i][j]);
                }
            }
            for (int i = 0; i < sovledDegs.Count; i++)
            {
                movements.Add(new RobotMovement(sovledDegs[i], transM, new Matrix.Vector(posDeg)));
            }
            return movements;
        }
    }
}
