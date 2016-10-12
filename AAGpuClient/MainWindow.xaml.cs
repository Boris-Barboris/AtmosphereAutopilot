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
        private Action _action;
        private Func<bool> _canExecute;
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
            rawExperiment = new RawModelExperiment();

            startRawCommand = new CommandHandler(startRawSimulation, canStartRaw);
        }

        private MainWindow owner_form;

        public DynamicsPlotModel rawExperimentPlot { get; private set; }

        public RawModelExperiment rawExperiment { get; private set; }

        public CommandHandler startRawCommand { get; private set; }

        Task rawTask;

        private void startRawSimulation()
        {
            if (rawTask != null)
                if ( !(rawTask.IsCompleted || rawTask.IsFaulted || rawTask.IsCanceled))
                    return;
            rawTask = new Task(doSimulRaw);
            rawTask.ContinueWith((t) => threaded_rawbuttonupdate());
            rawTask.Start();
            startRawCommand.raiseCanExecuteChanged();
        }

        private void doSimulRaw()
        {
            rawExperiment.execute();
            // publish results
            owner_form.Dispatcher.Invoke(publishRawSimulResult);
        }

        private void threaded_rawbuttonupdate()
        {
            owner_form.Dispatcher.Invoke(() => startRawCommand.raiseCanExecuteChanged());
        }

        private bool canStartRaw()
        {
            if (rawTask == null || rawTask.IsCompleted || rawTask.IsFaulted || rawTask.IsCanceled)
                return true;
            return false;
        }

        private void publishRawSimulResult()
        {
            rawExperimentPlot.Model.Series.Clear();

            addListAsSeries(rawExperiment.timePoints, rawExperiment.angVelHistory,
                "ang vel", OxyColors.Blue);
            addListAsSeries(rawExperiment.timePoints, rawExperiment.AOAHistory,
                "AoA", OxyColors.Red);
            addListAsSeries(rawExperiment.timePoints, rawExperiment.angAccHistory,
                "ang acc", OxyColors.Green);
            addListAsSeries(rawExperiment.timePoints, rawExperiment.csurfHistory,
                "csurf", OxyColors.Black, LineStyle.Dot);
            addListAsSeries(rawExperiment.timePoints, rawExperiment.inputHistory,
                "input", OxyColors.Black);

            rawExperimentPlot.Model.ResetAllAxes();
            rawExperimentPlot.Model.InvalidatePlot(true);
        }

        private void addListAsSeries(List<float> x, List<float> y, string name, 
            OxyColor col, LineStyle style = LineStyle.Solid)
        {
            var ser = new LineSeries();
            ser.Title = name;
            ser.Color = col;
            ser.StrokeThickness = 1.0;
            ser.LineStyle = style;
            for (int i = 0; i < x.Count; i++)
                ser.Points.Add(new DataPoint(x[i], y[i]));
            rawExperimentPlot.Model.Series.Add(ser);
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
