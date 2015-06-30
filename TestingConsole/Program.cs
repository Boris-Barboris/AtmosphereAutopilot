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
	using Vector = VectorArray.Vector;

    class Program
    {
        static void Main(string[] args)
        {
            Thread.CurrentThread.CurrentCulture = CultureInfo.CreateSpecificCulture("en-GB");
            simple_ann_test();
            //Console.ReadKey(true);
        }

        static void simple_ann_test()
        {
            double[] ann_inputs = { -0.3, 0.5, 1.6 };
            SimpleAnn ann = new SimpleAnn(3);
            foreach (var i in ann_inputs)
            {
                double output = 0.0;
                for (int j = 0; j < 1000; j++)
                    output += ann.eval(i);
                output /= 1000.0;
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
			List<Vector> inputs = new List<Vector>();
            inputs.Add((Vector)new double[2] { -1, -1 });
			inputs.Add((Vector)new double[2] { 0, 0 });
			inputs.Add((Vector)new double[2] { 1, 1 });
            double[] outputs = { -0.25, 0, 0.5 };
            SimpleAnn ann = new SimpleAnn(3, 2);
            SimpleAnn.GaussKoeff koeff = new SimpleAnn.GaussKoeff(1e-3, 1e-7, 1e7, 2, 100);
            double err;
            bool succ = ann.lm_iterate_batched(inputs, outputs, new double[3] { 1, 1, 1 }, 3, 0.25, koeff, 3, out err);
            Console.Write(succ.ToString());
        }

        static void GridSpaceTest()
        {
            GridSpace<double> space = new GridSpace<double>(2, new int[2] { 4, 9 }, new double[2] { -10, -10 }, new double[2] { 10, 10 });
			space.Put(42.0, (Vector)new double[] {-11.3, 1.1});
			var read = space.Get(new double[] { -11.5, 1.21});
			if (read != null)
				Console.Write(read.data.ToString("G8"));
			else
				Console.Write("Can't read");
			var linear = space.Linearized;
        }

        static void trainingTest()
        {
            List<double[]> inputs = new List<double[]>();
            List<double> outputs = new List<double>();
            int set_size = 50;
            for (int j = 0; j < set_size; j++)
            {
                inputs.Add(new double[] { j * 1.0 / (double)set_size });
                outputs.Add( j * 0.5 / (double)set_size);
            }
            int i = 0;
            SimpleAnn ann = new SimpleAnn(4, 1);
            OnlineAnnTrainer trainer = new OnlineAnnTrainer(ann, 10, new int[] { 5 },
                new double[] { -10.0 }, new double[] { 10.0 },
                (arr) => { arr[0] = inputs[i][0]; },
                () => { return outputs[i]; });
            BackgroundThread thread = new BackgroundThread();
            thread.add_func(() => { trainer.Train(); Console.WriteLine(trainer.ann_performance.ToString("G8")); return false; });
            thread.Start();
            while (i < set_size)
            {
                Thread.Sleep(20);
                trainer.UpdateState(0);
                Console.WriteLine("updated");
                i++;
            }
            thread.Stop();
        }
    }
}
