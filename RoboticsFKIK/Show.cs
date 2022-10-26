using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RoboticsFKIK
{
    public class Show
    {
        public static void showM(Matrix M)//传入数组
        {
            Console.Write(M.ToString());
        }
        public static void showDHTable(Matrix dhTable)
        {
            Console.WriteLine("DH Table:\n");
            Console.WriteLine("a  alpha  d  theta");
            showM(dhTable);
            Console.WriteLine();
        }

        public static void showPosDeg(Matrix posDeg)
        {
            Console.WriteLine("Position&Degree:");
            Console.WriteLine("x  y  z  alpha  beta  gamma");
            showM(posDeg.Transpose());
            Console.WriteLine("\n");
        }
        public static void showDegs(Matrix degs)
        {
            Console.WriteLine("Degrees:");
            showM(degs.Transpose());
            Console.WriteLine("\n");
        }
    }
}
