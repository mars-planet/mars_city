using MSKinectBodyTracker;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace MSKinectMultipeTrackerGUI
{
    public partial class Form1 : Form
    {
        private KinectManager km = new KinectManager();
        private Control[][] controls;

        public const int LABEL = 0;
        public const int BUTTON = 1;
        public const int TRACKBAR = 2;
        public const int COMBOBOX = 3;
        public const int BOTTOMLABEL = 4;
        public const int SELECTEDMACHINELABEL = 5;

        public Control[] GetControls(int i)
        {
            return controls[i];
        }

        public Form1()
        {
            InitializeComponent();

            controls = new Control[4][] { new Control[] { label1, identifyButton1, orientationTrackBar1, comboBox1, bottomLabel1, selectedMachineLabel1 }, new Control[] { label2, identifyButton2, orientationTrackBar2, comboBox2, bottomLabel2, selectedMachineLabel2 }, new Control[] { label3, identifyButton3, orientationTrackBar3, comboBox3, bottomLabel3, selectedMachineLabel3 }, new Control[] { label4, identifyButton4, orientationTrackBar4, comboBox4, bottomLabel4, selectedMachineLabel4 } };

            km.Start(this);
        }
    }
}
