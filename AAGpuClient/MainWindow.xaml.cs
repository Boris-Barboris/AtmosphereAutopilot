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
    }

    public class AppLogic
    {
        public AppLogic(MainWindow form)
        {
            owner_form = form;

            rawExperimentPlot = new DynamicsPlotModel();
            AoAExperimentPlot = new DynamicsPlotModel();

            rawExperiment = new RawModelExperiment();
            aoaEvalEeperiment = new AoAEvalExperiment();

            startRawCommand = new CommandHandler();
            startRawCommand._action = 
                () => startSimulation(ref rawTask, doSimulRaw, startRawCommand);
            startRawCommand._canExecute =
                () => canStart(ref rawTask);

            startAoAEvalCommand = new CommandHandler();
            startAoAEvalCommand._action =
                () => startSimulation(ref aoaEvalTask, doSimulAoA, startAoAEvalCommand);
            startAoAEvalCommand._canExecute =
                () => canStart(ref aoaEvalTask);
        }

        private MainWindow owner_form;

        public DynamicsPlotModel rawExperimentPlot { get; private set; }

        public DynamicsPlotModel AoAExperimentPlot { get; private set; }

        public RawModelExperiment rawExperiment { get; private set; }

        public AoAEvalExperiment aoaEvalEeperiment { get; private set; }

        public CommandHandler startRawCommand { get; private set; }

        public CommandHandler startAoAEvalCommand { get; private set; }

        Task rawTask, aoaEvalTask;

        private void startSimulation(ref Task task, Action action, CommandHandler cmd)
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
            aoaEvalEeperiment.execute();
            owner_form.Dispatcher.Invoke(publishAoASimulResult);
        }

        private void threaded_buttonupdate(CommandHandler cmd)
        {
            owner_form.Dispatcher.Invoke(() => cmd.raiseCanExecuteChanged());
        }

        private bool canStart(ref Task t)
        {
            if (t == null || t.IsCompleted || t.IsFaulted || t.IsCanceled)
                return true;
            return false;
        }

        private void publishRawSimulResult()
        {
            rawExperimentPlot.Model.Series.Clear();

            addListAsSeries(rawExperimentPlot.Model, rawExperiment.timePoints, rawExperiment.angVelHistory,
                "ang vel", OxyColors.Blue);
            addListAsSeries(rawExperimentPlot.Model, rawExperiment.timePoints, rawExperiment.AOAHistory,
                "AoA", OxyColors.Red);
            addListAsSeries(rawExperimentPlot.Model, rawExperiment.timePoints, rawExperiment.angAccHistory,
                "ang acc", OxyColors.Green);
            addListAsSeries(rawExperimentPlot.Model, rawExperiment.timePoints, rawExperiment.csurfHistory,
                "csurf", OxyColors.Black, LineStyle.Dot);
            addListAsSeries(rawExperimentPlot.Model, rawExperiment.timePoints, rawExperiment.inputHistory,
                "input", OxyColors.Black);

            rawExperimentPlot.Model.ResetAllAxes();
            rawExperimentPlot.Model.InvalidatePlot(true);
        }

        private void publishAoASimulResult()
        {
            AoAExperimentPlot.Model.Series.Clear();

            addListAsSeries(AoAExperimentPlot.Model, aoaEvalEeperiment.timePoints, aoaEvalEeperiment.angVelHistory,
                "ang vel", OxyColors.Blue);
            addListAsSeries(AoAExperimentPlot.Model, aoaEvalEeperiment.timePoints, aoaEvalEeperiment.AOAHistory,
                "AoA", OxyColors.Red);
            addListAsSeries(AoAExperimentPlot.Model, aoaEvalEeperiment.timePoints, aoaEvalEeperiment.angAccHistory,
                "ang acc", OxyColors.Green);
            addListAsSeries(AoAExperimentPlot.Model, aoaEvalEeperiment.timePoints, aoaEvalEeperiment.csurfHistory,
                "csurf", OxyColors.Black, LineStyle.Dot);
            addListAsSeries(AoAExperimentPlot.Model, aoaEvalEeperiment.timePoints, aoaEvalEeperiment.inputHistory,
                "input", OxyColors.Black);
            addListAsSeries(AoAExperimentPlot.Model, aoaEvalEeperiment.timePoints, aoaEvalEeperiment.outputVelHistory,
                "vel output", OxyColors.DarkMagenta);

            AoAExperimentPlot.Model.ResetAllAxes();
            AoAExperimentPlot.Model.InvalidatePlot(true);
        }

        private void addListAsSeries(PlotModel plot, List<float> x, List<float> y, 
            string name, OxyColor col, LineStyle style = LineStyle.Solid)
        {
            var ser = new LineSeries();
            ser.Title = name;
            ser.Color = col;
            ser.StrokeThickness = 1.0;
            ser.LineStyle = style;
            for (int i = 0; i < x.Count; i++)
                ser.Points.Add(new DataPoint(x[i], y[i]));
            plot.Series.Add(ser);
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
