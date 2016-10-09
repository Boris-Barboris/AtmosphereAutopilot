/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
Atmosphere Autopilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Atmosphere Autopilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.Globalization;
using AtmosphereAutopilot;
using System.IO;

namespace TestingConsole
{
	using Vector = VectorArray.Vector;

    class Program
    {
        static void Main(string[] args)
        {
            Thread.CurrentThread.CurrentCulture = CultureInfo.CreateSpecificCulture("en-GB");
            linprog_testing_grad();     
            Console.ReadKey(true);
        }

        //static void simple_ann_test()
        //{
        //    double[] ann_inputs = { -0.3, 0.5, 1.6 };
        //    SimpleAnn ann = new SimpleAnn(3);
        //    foreach (var i in ann_inputs)
        //    {
        //        double output = 0.0;
        //        for (int j = 0; j < 1000; j++)
        //            output += ann.eval(i);
        //        output /= 1000.0;
        //        Console.Write(output.ToString("G8") + " ");
        //    }
        //}

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

   //     static void LMTest()
   //     {
			//List<Vector> inputs = new List<Vector>();
   //         inputs.Add((Vector)new double[2] { -1, -1 });
			//inputs.Add((Vector)new double[2] { 0, 0 });
			//inputs.Add((Vector)new double[2] { 1, 1 });
   //         double[] outputs = { -0.25, 0, 0.5 };
   //         SimpleAnn ann = new SimpleAnn(3, 2);
   //         SimpleAnn.GaussKoeff koeff = new SimpleAnn.GaussKoeff(1e-3, 1e-7, 1e7, 2, 100);
   //         double err;
   //         bool succ = ann.lm_iterate_batched(inputs, outputs, new double[3] { 1, 1, 1 }, 3, 0.25, koeff, 3, out err);
   //         Console.Write(succ.ToString());
   //     }

   //     static void GridSpaceTest()
   //     {
   //         GridSpace<double> space = new GridSpace<double>(2, new int[2] { 4, 9 }, new double[2] { -10, -10 }, new double[2] { 10, 10 });
			//space.Put(42.0, (Vector)new double[] {-11.3, 1.1});
			//var read = space.Get(new double[] { -11.5, 1.21});
			//if (read != null)
			//	Console.Write(read.data.ToString("G8"));
			//else
			//	Console.Write("Can't read");
			//var linear = space.Linearized;
   //     }

        //static void trainingTest()
        //{
        //    List<double[]> inputs = new List<double[]>();
        //    List<double> outputs = new List<double>();
        //    int set_size = 10;
        //    for (int j = 0; j < set_size; j++)
        //    {
        //        inputs.Add(new double[] { j * 1.0 / (double)set_size });
        //        outputs.Add( j * 0.5 / (double)set_size);
        //    }
        //    int i = 0;
        //    SimpleAnn ann = new SimpleAnn(4, 1);
        //    OnlineAnnTrainer trainer = new OnlineAnnTrainer(ann, 10, new int[] { 5 },
        //        new double[] { -10.0 }, new double[] { 10.0 },
        //        (arr) => { arr[0] = inputs[i][0]; },
        //        () => { return outputs[i]; });
        //    BackgroundThread thread = new BackgroundThread("");
        //    thread.add_func(() => { trainer.Train(); Console.WriteLine(trainer.ann_performance.ToString("G8")); return false; });
        //    thread.Start();
        //    while (i < set_size)
        //    {
        //        Thread.Sleep(20);
        //        trainer.UpdateState(0);
        //        Console.WriteLine("updated");
        //        i++;
        //    }
        //    thread.Stop();
        //}

        //static void trainingPerformanceTest()
        //{
        //    List<double[]> inputs = new List<double[]>();
        //    List<double> outputs = new List<double>();
        //    int set_size = 100;
        //    for (int j = 0; j < set_size; j++)
        //    {
        //        inputs.Add(new double[2] {j * 1.0 / (double)set_size, -j * 0.7 / (double)set_size});
        //        outputs.Add(j * 0.5 / (double)set_size);
        //    }
        //    SimpleAnn ann = new SimpleAnn(6, 2);
        //    int i = 0;
        //    OnlineAnnTrainer trainer = new OnlineAnnTrainer(ann, 20, new int[2] {9, 9},
        //        new double[] {-1.0, -1.0}, new double[] {1.0, 1.0},
        //        (arr) => { arr[0] = inputs[i][0]; arr[1] = inputs[i][1]; },
        //        () => { return outputs[i]; });            
        //    while (i < set_size)
        //    {
        //        for (int j = 0; j < 5 && i < set_size; j++)
        //        {
        //            trainer.UpdateState(0);
        //            i++;
        //        }
        //        trainer.Train();
        //    }
        //    Console.WriteLine("finished");
        //}

        //static void trainingPerformanceTest_lin()
        //{
        //    List<double> inputs = new List<double>();
        //    List<double> outputs = new List<double>();
        //    int set_size = 100;
        //    for (int j = 0; j < set_size; j++)
        //    {
        //        inputs.Add(j);
        //        outputs.Add(j * 0.5);
        //    }
        //    for (int j = 0; j < set_size; j++)
        //    {
        //        inputs.Add(j);
        //        outputs.Add(j * 0.5);
        //    }
        //    LinApprox ann = new LinApprox(1);
        //    int i = 0;
        //    OnlineLinTrainer trainer = new OnlineLinTrainer(ann, null, 10, new int[2] { 9, 9 },
        //        new double[] { -0.1 }, new double[] { 0.1 },
        //        (arr) => { arr[0] = inputs[i]; },
        //        () => { return outputs[i]; });
        //    while (i < set_size)
        //    {
        //        for (int j = 0; j < 5 && i < set_size; j++)
        //        {
        //            trainer.UpdateState(0);
        //            i++;
        //        }
        //        trainer.Train();
        //    }
        //    Console.WriteLine("finished");
        //}

        //static void linear_real_data_training()
        //{
        //    string game_path = @"D:\Games\Kerbal Space Program 0.90\Resources\";

        //    StreamReader aoa_reader = new StreamReader(game_path + "aoa.csv");
        //    StreamReader control_reader = new StreamReader(game_path + "control.csv");
        //    StreamReader acc_reader = new StreamReader(game_path + "acc.csv");
        //    StreamReader density_reader = new StreamReader(game_path + "density.csv");
        //    StreamReader airspd_reader = new StreamReader(game_path + "airspd.csv");

        //    var acc = read_and_split(acc_reader);
        //    var aoa = read_and_split(aoa_reader);
        //    var control = read_and_split(control_reader);
        //    var density = read_and_split(density_reader);
        //    var airspd = read_and_split(airspd_reader);

        //    cut(acc, 2, 1);
        //    cut(aoa, 1, 2);
        //    cut(control, 2, 1);
        //    cut(density, 1, 2);
        //    cut(airspd, 1, 2);

        //    StreamWriter model_output = new StreamWriter(game_path + "debug_predict.csv");

        //    int frame = 0;
        //    int frames_count = acc.Count;
        //    double cpu_time = 0.0;
        //    double cpu_add = 0.5;

        //    LinApprox model = new LinApprox(2);
        //    OnlineLinTrainer trainer = new OnlineLinTrainer(model, null, 10, new double[] { 0.05, 0.05 }, new int[2] { 20, 20 },
        //        (arr) => { arr[0] = aoa[frame]; arr[1] = control[frame]; },
        //        () => { return acc[frame] / density[frame] / airspd[frame] / airspd[frame] * 2e4; });

        //    Vector input_vector = new Vector(2);

        //    for (frame = 0; frame < frames_count; frame++)
        //    {
        //        trainer.UpdateState(3);
        //        if (cpu_time >= 1.0)
        //        {
        //            trainer.Train();
        //            cpu_time -= 1.0;
        //        }
        //        cpu_time += cpu_add;
        //        input_vector[0] = aoa[frame];
        //        input_vector[1] = control[frame];
        //        double model_value = model.eval_training(input_vector);
        //        model_output.Write(model_value.ToString("G8") + ',');
        //    }

        //    model_output.Close();
        //    Console.WriteLine("finished");
        //}

        static List<double> read_and_split(StreamReader reader)
        {
            List<double> res = reader.ReadToEnd().Split(',').Where(s => s.Length > 0).Select(s => double.Parse(s)).ToList();
            reader.Close();
            return res;
        }

        static void cut(List<double> l, int from_start, int from_end)
        {
            l.RemoveRange(0, from_start);
            l.RemoveRange(l.Count - 1 - from_end, from_end);
        }

        static void line_translation_test()
        {
            VectorArray inputs_va = new VectorArray(1, 4);
            inputs_va.data[0] = 0.0;
            inputs_va.data[1] = 1.0;
            inputs_va.data[2] = 0.25;
            inputs_va.data[3] = 0.5;
            List<Vector> inputs = new List<Vector>();
            for (int i = 0; i < 4; i++)
                inputs.Add(inputs_va[i]);
            List<Vector> inputs_short = new List<Vector>();
            inputs_short.Add(inputs_va[0]);
            inputs_short.Add(inputs_va[1]);
            double[] outputs = new double[] { 0.0, 1.0, 0.5, 0.75 };
            double[] outputs_short = new double[] { 0.0, 1.0 };
            LinApprox approx = new LinApprox(1);
            approx.weighted_lsqr(inputs, outputs, new double[4] { 1.0, 1.0, 1.0, 1.0 }, new bool[1] { true });
            approx.weighted_lsqr(inputs_short, outputs_short, new double[2] { 1.0, 1.0 }, new bool[1] { false });            
        }

        static void linprog_testing()
        {
            AffineScaling linsolver = new AffineScaling(4, 3);
            //GradientLP linsolver = new GradientLP(2, 1);
            linsolver.A[2, 0] = 6.66;
            linsolver.A[2, 1] = -5.97;
            linsolver.b[2, 0] = 0.0;

            linsolver.A[0, 0] = 1.0;
            linsolver.A[0, 2] = 1.0;
            linsolver.b[0, 0] = 1.0;

            linsolver.A[1, 1] = 1.0;
            linsolver.A[1, 3] = 1.0;
            linsolver.b[1, 0] = 1.0;

            linsolver.x[0, 0] = 0.5;
            linsolver.x[1, 0] = 0.5;
            linsolver.x[2, 0] = 0.5;
            linsolver.x[3, 0] = 0.5;

            linsolver.c[0, 0] = -154.0;
            linsolver.c[1, 0] = -154.0;

            linsolver.solve(1e-3, 0.66, 10);
            linsolver.solve(1e-3, 0.66, 10);
            linsolver.solve(1e-3, 0.66, 10);
            linsolver.solve(1e-3, 0.66, 10);
            //linsolver.solve(1e-4, 0.5, 1000);
        }

        static void linprog_testing_grad()
        {
            GradientLP linsolver = new GradientLP(2, 1);
            linsolver.A[0, 0] = 6.66;
            linsolver.A[0, 1] = -5.97;
            linsolver.b[0, 0] = 0.0;

            linsolver.x[0, 0] = 1.0;
            linsolver.x[1, 0] = 1.0;

            linsolver.c[0, 0] = 1.0;
            linsolver.c[1, 0] = 1.0;

            //linsolver.solve(0.01, 10);
            //linsolver.solve(100.0, 10);
            //linsolver.solve(100.0, 10);
            //linsolver.solve(100.0, 10);
            //linsolver.solve(1e-4, 0.5, 1000);
        }
    }
}
