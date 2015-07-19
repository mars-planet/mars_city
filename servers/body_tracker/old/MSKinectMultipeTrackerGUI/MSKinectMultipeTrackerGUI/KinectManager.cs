using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Kinect;
using System.Globalization;
using System.IO;
using System.Windows.Forms;
using System.Threading;

namespace MSKinectBodyTracker
{
    public class KinectManager
    {
        private KinectSensor [] kinectSensors = new KinectSensor[4];
        private MSKinectMultipeTrackerGUI.Form1 form;

        private Button[] buttons = new Button[4];
        private Label[] labels = new Label[4];
        private TrackBar[] trackbars = new TrackBar[4];
        private ComboBox[] comboboxs = new ComboBox[4];
        private Label[] bottomLabels = new Label[4];
        private Label[] selectedMachineLabels = new Label[4];

        private bool[] canStartTracking = new bool[4] { false, false, false, false };

        /**
         * assignedMachine[i] is the ERAS machine assigned to the kinectSensors[i]
         * Whene no machine is assigned to kinectSensors[i], then assignedMachine[i] = -1
         * Otherwise, if assignedMachine[i] = y, then:
         * (*) y = 1 <=> the assigned machine is ERAS-1
         * (*) y = 2 <=> the assigned machine is ERAS-2
         * (*) y = 3 <=> the assigned machine is ERAS-3
         * (*) y = 4 <=> the assigned machine is ERAS-4
         */
        private int[] assignedMachine = new int[4] { 0, 0, 0, 0 };

        private String MachineNumberToMachineString(int machineNumber) {
            return "eras-" + machineNumber.ToString();
        }

        public void Start(MSKinectMultipeTrackerGUI.Form1 f)
        {
            form = f;
            Control[] ctrls;
            for (int i = 0; i < kinectSensors.Length; ++i)
            {
                ctrls = form.GetControls(i);
                labels[i] = (Label)ctrls[MSKinectMultipeTrackerGUI.Form1.LABEL];
                buttons[i] = (Button)ctrls[MSKinectMultipeTrackerGUI.Form1.BUTTON];
                trackbars[i] = (TrackBar)ctrls[MSKinectMultipeTrackerGUI.Form1.TRACKBAR];
                comboboxs[i] = (ComboBox)ctrls[MSKinectMultipeTrackerGUI.Form1.COMBOBOX];
                bottomLabels[i] = (Label)ctrls[MSKinectMultipeTrackerGUI.Form1.BOTTOMLABEL];
                selectedMachineLabels[i] = (Label)ctrls[MSKinectMultipeTrackerGUI.Form1.SELECTEDMACHINELABEL];
            }

            KinectSensor.KinectSensors.StatusChanged += new EventHandler<StatusChangedEventArgs>(KinectSensors_StatusChanged);
            DiscoverKinectSensors();

            for (int j = 0; j < kinectSensors.Length; ++j)
            {
                if (kinectSensors[j] != null)
                {
                    int sensorNumber = j;
                    kinectSensors[j].SkeletonFrameReady += (snd, eventArgs) => {
                        kinectSensor_SkeletonFrameReady(snd, eventArgs, sensorNumber);
                    };

                    StartKinect(sensorNumber);
                }
            }
        }

        private void DiscoverKinectSensors()
        {
            int i = 0;
            foreach (KinectSensor sensor in KinectSensor.KinectSensors)
            {
                if (sensor.Status == KinectStatus.Connected)
                {
                    // Found one, set our sensor to this
                    kinectSensors[i] = sensor;

                    // Init the found and connected device
                    if (kinectSensors[i].Status == KinectStatus.Connected)
                    {
                        InitializeKinect(i);
                    }

                    ++i;
                    if (i == 4) break;
                }
            }
        }

        private void DiscoverKinectSensor(int i)
        {
            foreach (KinectSensor sensor in KinectSensor.KinectSensors)
            {
                if (sensor.Status == KinectStatus.Connected)
                {
                    // Found one, set our sensor to this
                    kinectSensors[i] = sensor;
                    break;
                }
            }

            // Init the found and connected device
            if (kinectSensors[i].Status == KinectStatus.Connected)
            {
                InitializeKinect(i);
            }
        }

        private void InitializeKinect(int i)
        {
            InitializeKinect(kinectSensors[i], i);
        }

        private void InitializeKinect(KinectSensor sensor, int i)
        {
            //SkeletonStream
            //sensor.SkeletonStream.Enable();

            sensor.SkeletonStream.Enable(new TransformSmoothParameters()
            {
                Smoothing = 0.7f,
                Correction = 0.4f,
                Prediction = 0.7f,
                JitterRadius = 0.1f,
                MaxDeviationRadius = 0.1f
            });
        }

        private bool StartKinect(int i)
        {
            try
            {
                kinectSensors[i].Start();

                labels[i].ForeColor = System.Drawing.Color.Green;
                buttons[i].Enabled = trackbars[i].Enabled = comboboxs[i].Enabled = true;

                kinectSensors[i].ElevationAngle = 0;
                trackbars[i].Value = kinectSensors[i].ElevationAngle;

                buttons[i].Click += (snd, eventArgs) => { IdentifyButton_Click(buttons[i], i); };

                InitTrackbar(trackbars[i], i);

                comboboxs[i].SelectedValueChanged += (snd, eventArgs) => { SelectedErasMachine_Changed(comboboxs[i], i); };
            }
            catch
            {
                return false;
            }
            return true;
        }

        private void SelectedErasMachine_Changed(ComboBox cb, int sensorNumber)
        {
            String selectedMachine = (String) cb.SelectedItem;
            int selectedIndex = Int32.Parse(selectedMachine[selectedMachine.Length-1].ToString());
            assignedMachine[sensorNumber] = selectedIndex;

            cb.Enabled = false;
            cb.Visible = false;

            bottomLabels[sensorNumber].Visible = true;
            selectedMachineLabels[sensorNumber].Text = selectedMachine;
            selectedMachineLabels[sensorNumber].Visible = true;

            for (int i = 0; i < comboboxs.Length; ++i)
            {
                comboboxs[i].Items.Remove(selectedMachine);
            }

            canStartTracking[sensorNumber] = true;
        }

        private void InitTrackbar(TrackBar tb, int sensorNumber)
        {
            tb.MouseUp += (s, e) =>
            {
                TrackBar_Changed(tb, sensorNumber);
            };
            tb.MouseWheel += (s, e) =>
            {
                HandledMouseEventArgs ee = (HandledMouseEventArgs)e;
                ee.Handled = true;
            };
        }

        private void TrackBar_Changed(TrackBar tb, int sensorNumber)
        {
            tb.Enabled = false;
            buttons[sensorNumber].Enabled = false;
            kinectSensors[sensorNumber].ElevationAngle = tb.Value;

            System.Windows.Forms.Timer tmr = new System.Windows.Forms.Timer();
            tmr.Interval = 1350;
            tmr.Tick += (snd, eventArgs) =>
            {
                tb.Enabled = true;
                buttons[sensorNumber].Enabled = true;
                tmr.Stop();
            };
            tmr.Start();
        }

        private void IdentifyButton_Click(Button btn, int sensorNumber)
        {
            btn.Enabled = false;
            trackbars[sensorNumber].Enabled = false;

            if (kinectSensors[sensorNumber].ElevationAngle >= 0)
            {
                kinectSensors[sensorNumber].ElevationAngle = -27;
            }
            else
            {
                kinectSensors[sensorNumber].ElevationAngle = 27;
            }

            System.Windows.Forms.Timer tmr = new System.Windows.Forms.Timer();
            tmr.Interval = 1350;
            tmr.Tick += (snd, eventArgs) =>
            {
                btn.Enabled = true;
                if (kinectSensors[sensorNumber].ElevationAngle > 27)
                {
                    kinectSensors[sensorNumber].ElevationAngle = trackbars[sensorNumber].Value = 27;
                }
                else if (kinectSensors[sensorNumber].ElevationAngle < -27)
                {
                    kinectSensors[sensorNumber].ElevationAngle = trackbars[sensorNumber].Value = -27;
                }
                else
                {
                    trackbars[sensorNumber].Value = kinectSensors[sensorNumber].ElevationAngle;
                }
                trackbars[sensorNumber].Enabled = true;
                tmr.Stop(); 
            };
            tmr.Start();
        }

        private void KinectSensors_StatusChanged(object sender, StatusChangedEventArgs e)
        {
            for (int i = 0; i < kinectSensors.Length; ++i)
            {
                if (kinectSensors[i] != null && kinectSensors[i] == e.Sensor)
                {
                    if (e.Status == KinectStatus.Disconnected ||
                        e.Status == KinectStatus.NotPowered)
                    {
                        kinectSensors[i] = null;
                        InitializeKinect(kinectSensors[i], i);
                    }
                }
            }
        }      

        private void kinectSensor_SkeletonFrameReady(object sender, SkeletonFrameReadyEventArgs e, int sensorNumber)
        {
            if (!this.canStartTracking[sensorNumber] || sensorNumber < 0 || sensorNumber > 3)
            {
                return;
            }

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    Skeleton[] skeletonData = new Skeleton[skeletonFrame.SkeletonArrayLength];

                    skeletonFrame.CopySkeletonDataTo(skeletonData);

                    Skeleton playerSkeleton = null;
                    foreach (Skeleton skeleton in skeletonData)
                    {
                        if (skeleton.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            playerSkeleton = skeleton;
                            break;
                        }
                    }

                    if (playerSkeleton != null)
                    {
                        WriteJointsToJSONFile(playerSkeleton.Joints, "C:\\xampp\\htdocs\\Joints\\" + MachineNumberToMachineString(assignedMachine[sensorNumber]) + "\\joints.json");
                    }
                    else
                    {
                        //No Skeleton Tracked
                    }
                }
                else
                {
                    //No Skeletons detected
                }
            }
        }

        //private const float SCALE_CORRECTION_FOR_BLENDER = 778;
        private const float SCALE_CORRECTION_FOR_BLENDER = 1;
        private void WriteJointsToJSONFile(JointCollection jc, String filepath)
        {
            String jsonJoints = "{";

            foreach (Joint j in jc)
            {
                //PrintJoint(j);
                
                jsonJoints += "\"" + Enum.GetName(typeof(JointType), j.JointType) + "\"";
                jsonJoints += ":{\"x\":" + (j.Position.X * SCALE_CORRECTION_FOR_BLENDER).ToString("F6", CultureInfo.CreateSpecificCulture("en-US")) + ",\"y\":" + (j.Position.Y * SCALE_CORRECTION_FOR_BLENDER).ToString("F6", CultureInfo.CreateSpecificCulture("en-US")) + ",\"z\":" + (j.Position.Z * SCALE_CORRECTION_FOR_BLENDER).ToString("F6", CultureInfo.CreateSpecificCulture("en-US")) + "},";
            }

            StringBuilder sb = new StringBuilder(jsonJoints);
            sb[jsonJoints.Length - 1] = '}';
            jsonJoints = sb.ToString();

            if (File.Exists(filepath))
            {
                System.IO.StreamWriter file = new System.IO.StreamWriter(filepath + ".tmp");
                file.WriteLine(jsonJoints);
                file.Close();

                try
                {
                    File.Replace(filepath + ".tmp", filepath, filepath + ".tmp_backup");
                }
                catch (Exception e)
                {
                }
            }
            else
            {
                System.IO.StreamWriter file = new System.IO.StreamWriter(filepath);
                file.WriteLine(jsonJoints);
                file.Close();
            }

            Console.WriteLine("---------------");
        }
        
        //Just for debug
        private String PrintJoint(Joint j)
        {
            String jointName = Enum.GetName(typeof(JointType), j.JointType);
            String res = jointName + ":\n";
            res += "\t(" + j.Position.X.ToString("F6", CultureInfo.CreateSpecificCulture("en-US"));
            res += ", " + j.Position.Y.ToString("F6", CultureInfo.CreateSpecificCulture("en-US"));
            res += ", " +j.Position.Z.ToString("F6", CultureInfo.CreateSpecificCulture("en-US"));
            res += ")\n";
            return res;
        }

    }
}
