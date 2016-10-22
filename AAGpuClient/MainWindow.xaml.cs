using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

using OxyPlot;
using OxyPlot.Series;
using AAGpuWrapper;

namespace AAGpuClient
{

    public class CommandHandler : ICommand
    {
        internal Action _action;
        internal Func<bool> _canExecute;

        public CommandHandler() { }

        public CommandHandler(Action action, Func<bool> canExecute)
        {
            _action = action;
            _canExecute = canExecute;
        }

        public bool CanExecute(object parameter)
        {
            return _canExecute();
        }

        public event EventHandler CanExecuteChanged;

        public void Execute(object parameter)
        {
            _action();
        }

        public void raiseCanExecuteChanged()
        {
            CanExecuteChanged(this, null);
        }
    }


    public class DynamicsPlotModel
    {
        public DynamicsPlotModel()
        {
            Model = new PlotModel();
        }

        public PlotModel Model { get; private set; }

        public void addListAsSeries(List<float> x, List<float> y,
            string name, OxyColor col, LineStyle style = LineStyle.Solid)
        {
            var ser = new LineSeries();
            ser.Title = name;
            ser.Color = col;
            ser.StrokeThickness = 1.0;
            ser.LineStyle = style;
            for (int i = 0; i < x.Count; i++)
                ser.Points.Add(new DataPoint(x[i], y[i]));
            Model.Series.Add(ser);
        }
    }


    public class OptHistoryPlotModel
    {
        public OptHistoryPlotModel()
        {
            Model = new PlotModel();
            ser = new LineSeries();
            ser.Title = "objective";
            ser.Color = OxyColors.Blue;
            ser.StrokeThickness = 1.0;
            ser.LineStyle = LineStyle.Solid;
            Model.Series.Add(ser);
        }

        LineSeries ser;

        public PlotModel Model { get; private set; }

        public void update(int epoch, float value)
        {
            if (ser.Points.Count > 500)
            {
                var lpoint = ser.Points[ser.Points.Count - 1];
                ser.Points.Clear();
                ser.Points.Add(lpoint);
            }
            ser.Points.Add(new DataPoint(epoch, value));
            Model.InvalidatePlot(true);
        }

        public void clean()
        {
            ser.Points.Clear();
            Model.ResetAllAxes();
            Model.InvalidatePlot(true);
        }
    }


    // Main application class
    public class AppLogic
    {
        public AppLogic(MainWindow form)
        {
            owner_form = form;

            rawExperimentPlot = new DynamicsPlotModel();
            AoAExperimentPlot = new DynamicsPlotModel();
            AoAPsoPlot = new OptHistoryPlotModel();

            rawExperiment = new RawModelExperiment();
            aoaEvalExperiment = new AoAEvalExperiment();
            aoaPSOContext = new AoAPsoOptimization(reportAoAPso);

            startRawCommand = new CommandHandler();
            startRawCommand._action =
                () => AsyncStartAction(ref rawTask, doSimulRaw, startRawCommand);
            startRawCommand._canExecute =
                () => canStartByTask(ref rawTask);

            startAoAEvalCommand = new CommandHandler();
            startAoAEvalCommand._action =
                () => AsyncStartAction(ref aoaEvalTask, doSimulAoA, startAoAEvalCommand);
            startAoAEvalCommand._canExecute =
                () => canStartByTask(ref aoaEvalTask);

            startAoAPSOCommand = new CommandHandler();
            startAoAPSOCommand._action =
                () =>
                {
                    if (startOptimizeAoAPso())
                        AoAPsoPlot.clean();
                };
            startAoAPSOCommand._canExecute = () => { return true; };

            stopAoAPSOCommand = new CommandHandler();
            stopAoAPSOCommand._action =
                () => AsyncStartAction(ref stopPsoTask, stopOptimizeAoAPso, stopAoAPSOCommand);
            stopAoAPSOCommand._canExecute = () => canStartByTask(ref stopPsoTask);

            exportAoAParamsCommand = new CommandHandler();
            exportAoAParamsCommand._action =
                () =>
                {
                    if (best_aoa_params != null)
                        aoaEvalExperiment.AoA_params = new List<float>(best_aoa_params);
                    aoaEvalExperiment.InputLowerBounds = new List<float>(aoaPSOContext.InputLowerBounds);
                    aoaEvalExperiment.InputUpperBounds = new List<float>(aoaPSOContext.InputUpperBounds);
                    aoaEvalExperiment.OutputLowerBounds = new List<float>(aoaPSOContext.OutputLowerBounds);
                    aoaEvalExperiment.OutputUpperBounds = new List<float>(aoaPSOContext.OutputUpperBounds);
                };
            exportAoAParamsCommand._canExecute = () => { return true; };
        }

        private MainWindow owner_form;

        // plots

        public DynamicsPlotModel rawExperimentPlot { get; private set; }

        public DynamicsPlotModel AoAExperimentPlot { get; private set; }

        public OptHistoryPlotModel AoAPsoPlot { get; private set; }

        // experiments

        public RawModelExperiment rawExperiment { get; private set; }

        public AoAEvalExperiment aoaEvalExperiment { get; private set; }

        public AoAPsoOptimization aoaPSOContext { get; private set; }

        // commands

        public CommandHandler startRawCommand { get; private set; }

        public CommandHandler startAoAEvalCommand { get; private set; }

        public CommandHandler startAoAPSOCommand { get; private set; }

        public CommandHandler stopAoAPSOCommand { get; private set; }

        public CommandHandler exportAoAParamsCommand { get; private set; }

        // tasks

        Task rawTask, aoaEvalTask, stopPsoTask;

        private void AsyncStartAction(ref Task task, Action action, CommandHandler cmd)
        {
            if (task != null)
                if (!(task.IsCompleted || task.IsFaulted || task.IsCanceled))
                    return;
            task = new Task(action);
            task.ContinueWith((t) => threaded_buttonupdate(cmd));
            task.Start();
            cmd.raiseCanExecuteChanged();
        }

        private void doSimulRaw()
        {
            rawExperiment.execute();
            owner_form.Dispatcher.Invoke(publishRawSimulResult);
        }

        private void doSimulAoA()
        {
            aoaEvalExperiment.execute();
            owner_form.Dispatcher.Invoke(publishAoASimulResult);
        }

        private bool startOptimizeAoAPso()
        {
            return aoaPSOContext.start();
        }

        private void stopOptimizeAoAPso()
        {
            aoaPSOContext.stop();
        }

        private void threaded_buttonupdate(CommandHandler cmd)
        {
            owner_form.Dispatcher.Invoke(() => cmd.raiseCanExecuteChanged());
        }

        private bool canStartByTask(ref Task t)
        {
            if (t == null || t.IsCompleted || t.IsFaulted || t.IsCanceled)
                return true;
            return false;
        }

        private void publishRawSimulResult()
        {
            rawExperimentPlot.Model.Series.Clear();

            rawExperimentPlot.addListAsSeries(rawExperiment.timePoints, rawExperiment.angVelHistory,
                "ang vel", OxyColors.Blue);
            rawExperimentPlot.addListAsSeries(rawExperiment.timePoints, rawExperiment.AOAHistory,
                "AoA", OxyColors.Red);
            rawExperimentPlot.addListAsSeries(rawExperiment.timePoints, rawExperiment.angAccHistory,
                "ang acc", OxyColors.Green);
            rawExperimentPlot.addListAsSeries(rawExperiment.timePoints, rawExperiment.csurfHistory,
                "csurf", OxyColors.Black, LineStyle.Dot);
            rawExperimentPlot.addListAsSeries(rawExperiment.timePoints, rawExperiment.inputHistory,
                "input", OxyColors.Black);

            rawExperimentPlot.Model.ResetAllAxes();
            rawExperimentPlot.Model.InvalidatePlot(true);
        }

        private void publishAoASimulResult()
        {
            AoAExperimentPlot.Model.Series.Clear();

            AoAExperimentPlot.addListAsSeries(aoaEvalExperiment.timePoints, aoaEvalExperiment.angVelHistory,
                "ang vel", OxyColors.Blue);
            AoAExperimentPlot.addListAsSeries(aoaEvalExperiment.timePoints, aoaEvalExperiment.AOAHistory,
                "AoA", OxyColors.Red);
            AoAExperimentPlot.addListAsSeries(aoaEvalExperiment.timePoints, aoaEvalExperiment.angAccHistory,
                "ang acc", OxyColors.Green);
            AoAExperimentPlot.addListAsSeries(aoaEvalExperiment.timePoints, aoaEvalExperiment.csurfHistory,
                "csurf", OxyColors.Black, LineStyle.Dot);
            AoAExperimentPlot.addListAsSeries(aoaEvalExperiment.timePoints, aoaEvalExperiment.inputHistory,
                "input", OxyColors.Black);
            AoAExperimentPlot.addListAsSeries(aoaEvalExperiment.timePoints, aoaEvalExperiment.outputVelHistory,
                "vel output", OxyColors.DarkMagenta);

            AoAExperimentPlot.Model.ResetAllAxes();
            AoAExperimentPlot.Model.InvalidatePlot(true);
        }

        private List<float> best_aoa_params = null;

        private void reportAoAPso(int epoch, float val, List<float> pars)
        {
            owner_form.Dispatcher.Invoke(
                () =>
                {
                    best_aoa_params = pars;
                    AoAPsoPlot.update(epoch, val);
                });
        }
    }

    
    /// <summary>
    /// Main Window class
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            DataContext = new AppLogic(this);
        }
    }

}
