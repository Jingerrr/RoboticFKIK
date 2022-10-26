
using RoboticsFKIK;
using System;
using static System.Console;

namespace ConsoleApp
{
    class Program
    {
        static void Main(string[] args)
        {
            Matrix dhTable=new Matrix(new double[,]
           {
                {0, 90, 0.163,0},
                {0.632, 180, 0,90},
                {0.6005, 180, 0,0},
                {0, -90, 0.2013,-90},
                {0, 90, 0.1025,0},
                {0, 0, 0.094,0},
            });
            Show.showDHTable(dhTable);
            WriteLine("\n****************************\n");

            Robot robot = new Robot(dhTable);


            //FK
            Matrix degrees = new Matrix(new double[,]
           {
                 {0,0,90,0,0,0},
                 {10, 10, 90,10,10,10},
                 {20, 20, 90,20,20,20},
                 {30, 30, 90,30,30,30},
                 {40, 40, 90,40,40,40},
            }); 
            for (int i = 0; i < degrees.Rows; i++)
            {
                showFKResult(degrees.getRow(i).element, robot);
            }


            //IK
            Matrix posDegs = new Matrix(new double[,]
           {
                 {0.28127,0,1.13182,90,0,180},
                 {0.41323, 0, 1.00785,90,0,180},
                 {0.54519, 0, 0.88387,90,0,180},
                 {0.67715, 0, 0.7599,90,0,180},
                 {0.8091, 0, 0.63592,90,0,180},
            });

            for (int i = 0; i < posDegs.Rows; i++)
            {
                showIKResult(posDegs.getRow(i).element, robot);
            }
            
        }
        public static void showFKResult(double[] degrees,Robot robot)
        {
            WriteLine("FK:\n");
            Show.showDegs(new Matrix.Vector(degrees));

            RobotMovement robotMovement = robot.forwardKinematics(degrees);
            Show.showPosDeg(robotMovement.posDeg);
            WriteLine("\n****************************\n");
        }
        public static void showIKResult(double[] posDegs, Robot robot)
        {
            WriteLine("IK:\n");
            Show.showPosDeg(new Matrix.Vector(posDegs));

            List<RobotMovement> robotMovements = robot.inverseKinematics(posDegs);
            List<double[]> degs=new List<double[]>();
            
            for (int i = 0; i < robotMovements.Count; i++)
            {
                degs.Add(robotMovements[i].degrees);
            }
            Matrix degM = new Matrix(degs);
            WriteLine("Degrees:");
            Show.showM(degM);
            WriteLine("\n");
           WriteLine("\n****************************\n");
        }
      
    }
}