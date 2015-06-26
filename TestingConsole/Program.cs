using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.Globalization;
using AtmosphereAutopilot;

namespace TestingConsole
{
    class Program
    {
        static void Main(string[] args)
        {
            Thread.CurrentThread.CurrentCulture = CultureInfo.CreateSpecificCulture("en-GB");
            
            Console.ReadKey(true);
        }

        static void simple_ann_test()
        {
            double[] ann_inputs = { -0.3, 0.5, 1.6 };
            SimpleAnn ann = new SimpleAnn(3);
            foreach (var i in ann_inputs)
            {
                double output = ann.eval(i);
                Console.Write(output.ToString("G8") + " ");
            }
        }

        static void ListViewTest()
        {
            double[] arr1 = { -3, -2, -1, 0 };
            double[] arr2 = { 1 };
            double[] arr3 = { 2, 3 };
            ListView<double> view = new ListView<double>(arr1, arr2, arr3);
            Console.WriteLine(view.ToString());
            for (int i = 0; i < view.Count; i++)
                view[i] += 1;
            Console.WriteLine(view.ToString());
        }

        static void LMTest()
        {
            List<double[]> inputs = new List<double[]>();
            inputs.Add(new double[2] { -1, -1 });
            inputs.Add(new double[2] { 0, 0 });
            inputs.Add(new double[2] { 1, 1 });
            double[] outputs = { -0.25, 0, 0.5 };
            SimpleAnn ann = new SimpleAnn(3, 2);
            SimpleAnn.GaussKoeff koeff = new SimpleAnn.GaussKoeff(1e-3, 1e-7, 1e7, 2, 100);
            double err;
            bool succ = ann.lm_iterate_batched(inputs, outputs, new double[3] { 1, 1, 1 }, 3, 0.25, koeff, 3, out err);
            Console.Write(succ.ToString());
        }

        static void GridSpaceTest()
        {
            GridSpace<double> space = new GridSpace<double>(2, new int[2] { 5, 5 }, new double[2] { -10, -10 }, new double[2] { 10, 10 });

        }
    }
}
