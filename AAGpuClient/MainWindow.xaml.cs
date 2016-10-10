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

    public class DynamicsModel
    {
        public DynamicsModel(string name)
        {
            Model = new PlotModel { Title = name, TitleFontSize = 18.0 };
            Model.Series.Add(new FunctionSeries(Math.Cos, 0, 10, 0.1, "cos(x)"));
        }

        public PlotModel Model { get; private set; }
    }

    public class AppContext
    {
        public AppContext()
        {
            Rawmodel = new DynamicsModel("Raw model dynamics");
            RawExperiment = new RawModelExperiment();
        }        

        public DynamicsModel Rawmodel { get; private set; }

        public RawModelExperiment RawExperiment { get; private set; }
    }

    
    /// <summary>
    /// Main Window class
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            DataContext = new AppContext();
        }
    }

}
