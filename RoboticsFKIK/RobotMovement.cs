using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RoboticsFKIK
{
   
    public class RobotMovement
    {
        public double[] degrees { get;}
        public Matrix transMatrix { get; }
        public Matrix.Vector posDeg { get;  }
        public RobotMovement(Robot robot,double[] _degrees) 
        {
            degrees = _degrees;
            transMatrix= MovementCalculation.getTransMatrix(0, degrees.Length, degrees, robot);
            posDeg =MovementCalculation.getPosDeg(transMatrix);
        }
        public RobotMovement(double[] _degrees, Matrix _transMatrix, Matrix.Vector _posDeg)
        {
            degrees = _degrees;
            transMatrix = _transMatrix;
            posDeg = _posDeg;
        }
     }

   
}
