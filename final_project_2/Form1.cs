using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using CSML;

using System.IO.Ports;
using System.Globalization;
using System.Xml.Schema;

namespace final_project2
{
    public partial class txtRoll : Form
    {
        //
        //var cau hinh IMU
        private int i_m = 0, i_a = 0, i_g = 0; //index
        int accel_config = 0, gyro_config = 0;
        string dataIn = string.Empty;
        private float rollf1 = 0, pitchf1 = 0, yawf1 = 0;
        private bool mode = true;
        //string rollStr = string.Empty, pitchStr = string.Empty, yawStr = string.Empty;
        string[] gyroStr = new string[3];
        string[] accelStr = new string[3];
        string[] magStr = new string[3];
        string dataAnalyze;
        char[] letters = new char[10];
        int indexData = 0;
        int indexLetter = 0;
        //Kalman
        private bool trigKalman = false;
        private bool fistScan = true; //use for getting initial roll/pitch/yaw values
        double[] xKKalman = new double[13]; //13 state vecto kalman [q1 q2 q3 q4 f1 f2 f3 m1 m2 m3 w1 w2 w3]
        //                                                         0  1  2  3  4  5  6  7  8  9  10 11 12 
        double[] zKalman = new double[9];
        private bool syncAccel = false;
        private bool syncGyro = false;
        private bool syncMag = false;
        private float tSample = 0.5f; //chua hieu chinh!!!
        private bool otherScan = false; // chua xai
        double[] xKp1Kalman = new double[13]; //13 state vector kalman k + 1 - chi xai 4 bien dau tien
        double[,] F_k = new double[13, 13];
        double[,] p_k = new double[13, 13];
        double[,] h_k = new double[9, 13];
        double[,] r_k = new double[9, 9];
        private float cF = 0.95f;
        Matrix AKalman = new Matrix(13, 13);
        Matrix PKalman = new Matrix(13, 13);
        Matrix kKKalman = new Matrix(13, 9);
        Matrix hKalman = new Matrix(9, 13);
        Matrix rKalman = new Matrix(9, 9);
        Matrix xKalmanMatrix = new Matrix(1, 13);
        Matrix xKp1KalmanMatrix = new Matrix(1, 13);
        Matrix zKalmanMatrix = new Matrix(1, 9);
        private double iMag = (9 / 180) * Math.PI; //so nay co the thay doi
        private double[,] deviationStandard = new double[9, 9];
        //var calib Mag
        private float[] valueCalibMagX = new float[100];
        private float[] valueCalibMagY = new float[100];
        private float[] valueCalibMagZ = new float[100];
        private bool stopCalibMag = false;
        //
        Matrix betaMag = new Matrix(1, 6);
        Matrix oMag = new Matrix(3, 1);
        Matrix KMagDiag = new Matrix(3, 3);
        Matrix cMag = new Matrix(3, 1);
        Matrix rMag = new Matrix(3, 1);
        private string[] hsBeta = new string[6];// [A1 B1 A2 B2 A3 B3] Mag
        bool trigExecMatrixMag = false; //var enable compute matrix Mag
        private float[] offsetAccel = new float[3]; // offset x y z
        //var calib Gyro
        private float[] valueGyroX = new float[100]; //calib
        private float[] valueGyroY = new float[100];
        private float[] valueGyroZ = new float[100];
        private float[] offsetGyro = new float[3]; // offset x y z
        private bool triGyro = false;
        private bool stopCalibGyro = false;
        //var calib Accel
        private float[] valueAccelX = new float[100];
        private float[] valueAccelY = new float[100];
        private float[] valueAccelZ = new float[100];
        Matrix betaAccel = new Matrix(1, 6);
        private string[] hsBetaAccel = new string[6]; //Beta Accel
        private bool trigAccel = false;
        Matrix oAccel = new Matrix(3, 1);
        Matrix KAccelDiag = new Matrix(3, 3);
        Matrix cAccel = new Matrix(3, 1);
        Matrix rAccel = new Matrix(3, 1);
        private bool stopCalibAccel = false;
        private bool calibAccelMode = true; //mode 1: calib giong gyro 
        //
        Form2 f = new Form2();
        public txtRoll()
        {
            InitializeComponent();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            string[] ComList = SerialPort.GetPortNames();
            Array.Sort(ComList);
            cbxPortName.Items.AddRange(ComList);
            txtReceived.ScrollBars = ScrollBars.Vertical;
            cbxPortName.Text = Properties.Settings.Default.comPort;
            cbxBaudrate.Text = Properties.Settings.Default.baudRate;
            cbxDataBits.Text = Properties.Settings.Default.dataBits;
            cbxStopBits.Text = Properties.Settings.Default.stopBits;
            cbxParity.Text = Properties.Settings.Default.parity;
            for(int i = 0; i < 4; i++)
            {
                F_k[i, i] = 1; // 00 11 22 33 

            }
            for(int i = 0; i < 6; i++)
            {
                F_k[i + 4, i + 4] = cF; // 44 55 66 77 88 99
            }
            for(int i = 0; i < 4; i++) // 04 --> 09 - 14 --> 19 - 24 --> 29 - 34 --> 39
            {
                for(int j = 0; j < 6; j++)
                {
                    F_k[i, j + 4] = 0;
                }
            }
            for(int i = 0; i < 6; i++)
            {
                for(int j = 0; j < 13; j++)
                {
                    F_k[i + 4, j] = 0;
                }
            }
            for(int i = 0; i < 3; i++)
            {
                for(int j = 0; j < 12; j++)
                {
                    F_k[i + 10, j] = 0;
                }
            }
            for(int i = 0; i < 13; i++)
            {
                for(int j =0; j < 13; j++)
                {
                    p_k[i, j] = 10; //set initial PKalman value
                }
            }
            PKalman = new Matrix(p_k);
            for(int i = 0; i < 3; i++) //h
            {
                for(int j = 0; j < 10; j++)
                {
                    h_k[i, j] = 0;
                }
            }
            for (int i = 0; i < 6; i++) //h
            {
                for (int j = 0; j < 3; j++)
                {
                    h_k[i + 3, j + 10] = 0;
                }
            }
            for(int i = 0; i < 6; i++)
            {
                h_k[i + 3, i + 4] = 1;
            }
            for (int i = 0; i < 3; i++)
            {
                h_k[i, i + 10] = 1;
                    }
        }

        private void btn1_Click(object sender, EventArgs e)
        {
            if (cbxPortName.Text == "")
            {
                MessageBox.Show("Please connect to port! (Error 0x03)", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            if (serialPort1.IsOpen)
            {
                serialPort1.Close();
                btn1.Text = "Connect";
                cbxPortName.Enabled = true;
                cbxBaudrate.Enabled = true;
                cbxDataBits.Enabled = true;
                cbxParity.Enabled = true;
                cbxStopBits.Enabled = true;
            }
            else
            {
                if (cbxPortName.Text != "")
                {

                    try
                    {
                        serialPort1.PortName = cbxPortName.Text;
                        serialPort1.BaudRate = Convert.ToInt32(cbxBaudrate.Text);
                        serialPort1.DataBits = Convert.ToInt32(cbxDataBits.Text);
                        switch (cbxParity.Text)
                        {
                            case ("None"):
                                {
                                    serialPort1.Parity = Parity.None;
                                    break;
                                }
                            case ("Odd"):
                                {
                                    serialPort1.Parity = Parity.Odd;
                                    break;
                                }
                            case ("Even"):
                                {
                                    serialPort1.Parity = Parity.Even;
                                    break;
                                }
                        }
                        switch (cbxStopBits.Text)
                        {
                            case ("1"):
                                {
                                    serialPort1.StopBits = StopBits.One;
                                    break;
                                }
                            case ("1.5"):
                                {
                                    serialPort1.StopBits = StopBits.OnePointFive;
                                    break;
                                }
                            case ("2"):
                                {
                                    serialPort1.StopBits = StopBits.Two;
                                    break;
                                }
                        }

                        serialPort1.Open();
                        btn1.Text = "Disconnect";
                        cbxPortName.Enabled = false;
                        cbxBaudrate.Enabled = false;
                        cbxDataBits.Enabled = false;
                        cbxParity.Enabled = false;
                        cbxStopBits.Enabled = false;

                    }
                    catch
                    {
                        MessageBox.Show("Can not connect to (Error 0x02)" + serialPort1.PortName, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    }
                }
            }
        }
  
        private void btn3_Click_1(object sender, EventArgs e)
        {
            if (serialPort1.IsOpen)
            {
                progBar.PerformStep();
                f.ShowDialog();
                //if (f.IsDisposed)
                //{
                 //  f.ShowDialog();
                //}
                //else f.Show();
                //btn3.Visible = false;
                f.enable = true;

            }
            else MessageBox.Show("Please connect to the port", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
        }

        private void cbx6_SelectedIndexChanged_1(object sender, EventArgs e)
        {
            ComboBox cbx6 = sender as ComboBox;

            switch (Convert.ToInt32(cbx6.SelectedIndex))
            {
                case 0:
                    accel_config = 10;
                    break;
                case 1:
                    accel_config = 11;
                    break;
                case 2:
                    accel_config = 12;
                    break;
                case 3:
                    accel_config = 13;
                    break;
                default:
                    break;
            }
            lbstatus.Text = "Status: Value choosen is " + Convert.ToString(accel_config);
        }

        private void btn2_Click_1(object sender, EventArgs e)
        {
            if (serialPort1.IsOpen)
            {
                if (accel_config != 0)
                {
                    serialPort1.Write(accel_config.ToString());
                    // show trang thai
                }
                else lbstatus.Text = "Status: No value (Error 0x01)"; //error value accel
                if (gyro_config != 0)
                {
                    serialPort1.Write(gyro_config.ToString());
                    //show trang thai
                }
                else lbstatus.Text = "Status: No value (Error 0x01)"; //error value gyro
            }
            else MessageBox.Show("Please connect to the port", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Error);
        }
            

        private void Form1_FormClosing_1(object sender, FormClosingEventArgs e)
        {
            if (serialPort1.IsOpen)
            {
                serialPort1.Close();
            }
            Properties.Settings.Default.comPort = cbxPortName.Text;
            Properties.Settings.Default.baudRate = cbxBaudrate.Text;
            Properties.Settings.Default.dataBits = cbxDataBits.Text;
            Properties.Settings.Default.parity = cbxParity.Text;
            Properties.Settings.Default.stopBits = cbxStopBits.Text;
            Properties.Settings.Default.Save();
        }


        private void timer2_Tick(object sender, EventArgs e)
        {
            //
            
        }


        private void btnSend_Click(object sender, EventArgs e)
        {
            serialPort1.Write(txtSendData.Text);
        }

        private void button1_Click(object sender, EventArgs e)
        {
            stopCalibGyro = false;
            stopCalibMag = false;
            stopCalibAccel = false;
            i_a = i_m = i_g = 0;
        }


        private void btn4_Click(object sender, EventArgs e)
        {
           
        }

        private void serialPort1_DataReceived_1(object sender, SerialDataReceivedEventArgs e)
        {
            try
           {

                //dataIn = serialPort1.ReadExisting();
                
                this.Invoke(new EventHandler(showData));
                AnalyzeString();
           }
           catch
            {
              //  MessageBox.Show("Received fail (Error 0x04)", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
           }
        }
        private void showData(object sender, EventArgs e)
        {
            dataAnalyze = serialPort1.ReadLine();
            txtReceived.Text += dataAnalyze + "\r\n";
        }
        
        
        private void AnalyzeString()
        {
            dataIn = dataAnalyze;

            //r**p**y**e
            for (int i = 0; i < dataAnalyze.Length; i++)
            {
                if (dataIn[i] == 'r') //gyro frame: rXXpXXyXXe
                {
                    indexData = i;
                    break;
                }
                if (dataIn[i] == 'a') //accel frame aXXxXXyXXz
                {
                    indexData = i;
                    break;
                }
                if (dataIn[i] == 'm') //mag frame mXXxXXyXXz
                {
                    indexData = i;
                    break;
                }
                if (dataIn[i] == 'M') //calib Mag frame M
                {
                    MessageBox.Show("Start calibrating!!", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    break;
                }
                if (dataIn[i] == 'g') //receive value Magnetic calib
                {
                    indexData = i;
                    break;
                }
                if (dataIn[i] == 'A') //accel array
                {
                    MessageBox.Show("Start getting value Accel!!", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    break;
                }
                if (dataIn[i] == 'D') //accel array
                {
                    MessageBox.Show("Start getting value Gyro!!", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    break;
                }
                if (dataIn[i] == 's') //receive value accel calib
                {
                    indexData = i;
                    break;
                }
                if (dataIn[i] == 'd') //receive value gyro calib
                {
                    indexData = i;
                    break;
                }
                if(dataIn[i] == 'S')
                {
                    MessageBox.Show("Start IMU!!", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    break;
                }
                if (dataIn[i] == 'T')
                {
                    MessageBox.Show("Stop IMU!!", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);
                    break;
                }
            }
            switch (dataIn[indexData])
            {
                case 'r': //value gyro
                    {
                        indexData++;

                        gyroStr[0] = clearLetters('p');

                        gyroStr[1] = clearLetters('y');

                        gyroStr[2] = clearLetters('e');


                        indexData = 0;

                        this.Invoke(new EventHandler(showTextRawGyro)); //
                        //additon
                        dataIn = string.Empty;

                        //w1 w2 w3 xk[10] xk[11] xk[12]
                        if (triGyro) //da co offset
                        {
                            for (int i = 0; i < 3; i++)
                            {
                                if (double.TryParse(gyroStr[i], out xKKalman[i + 10]))
                                {
                                    if (fistScan)
                                    {
                                        xKKalman[i + 10] -= offsetGyro[i];
                                    }
                                    else
                                    {
                                        zKalman[i] -= offsetGyro[i];
                                    }
                                    
                                }
                                //else???
                            }
                            //
                            syncGyro = true; //condition for sync kalmanfilter
                            //
                            this.Invoke(new EventHandler(showTextCalibGyro)); //test thread
                        } //xong if nay la co gia tri hieu chinh gyro
                    }
                    break;
                case 'a': //value accel
                    {
                        indexData++;

                        accelStr[0] = clearLetters('x');

                        accelStr[1] = clearLetters('y');

                        accelStr[2] = clearLetters('z');


                        indexData = 0;

                        this.Invoke(new EventHandler(showTextRawAccel));
                        //additon
                        dataIn = string.Empty;
                        //xuat gia tri sau khi calib
                        if (trigAccel)
                        {
                            double[] accel = new double[3];
                            int j = 0; //var accel[index]
                            foreach(var i in accelStr)
                            {
                                if (double.TryParse(i, out accel[j]))
                                {
                                    //
                                }
                            }
                            if (calibAccelMode)
                            {
                                for(int i = 0; i < 3; i++)
                                {
                                    zKalman[i + 3] = accel[i] - offsetAccel[i]; //gtri sau hieu chinh
                                }     
                            }
                            else
                            {
                                rAccel = new Matrix(accel);
                                Matrix roAccel = new Matrix();
                                roAccel = rAccel + oAccel;
                                cAccel = KAccelDiag * roAccel;
                                convertCAccel(cAccel.ToString()); //xong ham nay co dc he so f1 f2 f3 xk kalman
                            }
                            syncAccel = true; //condition for sync kalmanfilter
                            this.Invoke(new EventHandler(showTextCalibAccel));//
                        }
                    }
                    break;
                case 'm': //value mag
                    {
                        indexData++;

                        magStr[0] = clearLetters('x');

                        magStr[1] = clearLetters('y');

                        magStr[2] = clearLetters('z');

                        indexData = 0;

                        this.Invoke(new EventHandler(showTextRawMag));
                        dataIn = string.Empty;
                        //gia tri sau khi calib
                        if (trigExecMatrixMag)
                        {
                            double[] mag = new double[3];
                            mag[0] = double.Parse(magStr[0]); //chua apply ham foreach nhu accel
                            mag[1] = double.Parse(magStr[1]); //co the bi error
                            mag[2] = double.Parse(magStr[2]);
                            rMag = new Matrix(mag); //global var rMag
                            Matrix roMag = new Matrix();
                            roMag = rMag + oMag;
                            cMag = KMagDiag * roMag;
                            convertCMag(cMag.ToString()); //xong ham nay co dc he so m1 m2 m3 xk kalman

                            syncMag = true; //condition for sync kalmanfilter
                            this.Invoke(new EventHandler(showTextCalibMag));
                        }
                    }
                    break;
                case 'g': //calib mag
                    {
                        if (!stopCalibMag)
                        {

                            indexData++;

                            valueCalibMagX[i_m] = float.Parse(clearLetters('x'));

                            valueCalibMagY[i_m] = float.Parse(clearLetters('y'));

                            valueCalibMagZ[i_m] = float.Parse(clearLetters('z'));

                            i_m++;

                            if (i_m == 100)
                            {
                                i_m = 0;
                                MessageBox.Show("Received data to calib Mag successful!!!", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);

                                deviationStandard[6, 6] = findDeviation(valueCalibMagX); //tim do lech chuan
                                deviationStandard[7, 7] = findDeviation(valueCalibMagY);
                                deviationStandard[8, 8] = findDeviation(valueCalibMagZ);
                                execMatrixMag(); // chay xong ham nay la co he so Beta mag
                                //stop calib
                                stopCalibMag = true;
                            }
                        }
                        
                    }
                    break;
                case 's': //calib accel
                    {
                        if (!stopCalibAccel)
                        {
                            indexData++;

                            valueAccelX[i_a] = float.Parse(clearLetters('x'));
                            valueAccelY[i_a] = float.Parse(clearLetters('y'));
                            valueAccelZ[i_a] = float.Parse(clearLetters('z'));
                            i_a++;
                            if (i_a == 100)
                            {
                                MessageBox.Show("Received data to calib Accel successful!!!", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);
                                //
                                deviationStandard[3, 3] = findDeviation(valueAccelX);//tim do lech chuan
                                deviationStandard[4, 4] = findDeviation(valueAccelY);
                                deviationStandard[5, 5] = findDeviation(valueAccelZ);
                                //
                                if (calibAccelMode)
                                {
                                    findOffsetAccel();
                                }
                                else
                                {
                                    i_a = 0;
                                    execMatrixAccel(); //chay xong ham nay la co he so Beta - o - KDiag Accel
                                    
                                }
                                //stop calib
                                stopCalibAccel = true;
                            }
                        }
                        
                    }
                    break;
                case 'd': //calib gyro
                    {
                        if (!stopCalibGyro)
                        {
                            indexData++;

                            valueGyroX[i_g] = float.Parse(clearLetters('x'));
                            valueGyroY[i_g] = float.Parse(clearLetters('y'));
                            valueGyroZ[i_g] = float.Parse(clearLetters('z'));
                            i_g++;
                            if (i_g == 100)
                            {
                                //i_g = 0;
                                MessageBox.Show("Received data to calib Gyro successful!!!", "Notice", MessageBoxButtons.OK, MessageBoxIcon.Information);
                                deviationStandard[0, 0] = findDeviation(valueGyroX);//tim do lech chuan
                                deviationStandard[1, 1] = findDeviation(valueGyroY);
                                deviationStandard[2, 2] = findDeviation(valueGyroZ);
                                findOffsetGyro();
                                //stop calib gyro
                                stopCalibGyro = true;
                            }
                        }
                        
                    }
                    break;
                default:
                    break;
            }
            if(mode == true)
            {
                //lay value kalman (chua chuyen)(x)
                if (syncAccel & syncGyro & syncMag) //accel gyro va mag da duoc hieu chinh
                {
                    trigKalman = true;
                }
                //
                kalmanFilter(); //tinh bo loc kalman: da co lenh xu ly trigKalman trong method kalmanFilter
                                //
                if (trigKalman)
                {
                    this.Invoke(new EventHandler(showTextRPY));
                }
            }
            else
            {
                float[] accel = new float[3];
                for(int i = 0; i < 3; i++)
                {
                    accel[i] = float.Parse(accelStr[i]);
                }
                rollf1 = (float)Math.Atan(accel[0] / Math.Sqrt(accel[0] * accel[0] + accel[2] * accel[2]));//rad //xem lai ham atan va atan2
                pitchf1 = (float)Math.Atan(Math.Sqrt(accel[0] * accel[0] + accel[1] * accel[1]) / accel[2]); //radian
                yawf1 = (float)Math.Atan(accel[0] / Math.Sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
                this.Invoke(new EventHandler(showTextRPY));
            }
            
        }
        private double findDeviation(float[] value)
        {
            double outValue;
            double average = 0;
            double hs = 1.0000f/(value.Length -1.00000f);
            for (int i = 0; i < value.Length; i++)
            {
                average += value[i];
            }
            average /= value.Length; // x ngang
            double temp = 0;
            for(int i = 0; i < value.Length; i++)
            {
                temp += (value[i] - average) * (value[i] - average);
            }
            outValue = temp* hs;
            return outValue;
        }
        private void showTextRPY(object sender, EventArgs e)
        {
            rollf1 = convertRadToDegree(rollf1); //rad to degree
            pitchf1 = convertRadToDegree(pitchf1);
            yawf1 = convertRadToDegree(yawf1);
            f.GetValueRollPitchYaw(rollf1, pitchf1, yawf1); //send to form2 thi phai la degree
            txtRollf1.Text = rollf1.ToString();
            txtPitchf1.Text = pitchf1.ToString();
            txtYawf1.Text = yawf1.ToString();
        }
        private float convertRadToDegree(float buffIn)
        {
            float buffOut;
            buffOut = (buffIn /(float) Math.PI) * 180;
            return buffOut;
        }
        private void showTextCalibMag(object sender, EventArgs e)
        {
            if (fistScan)
            {
                txtCalibMagX.Text = xKKalman[7].ToString();
                txtCalibMagY.Text = xKKalman[8].ToString();
                txtCalibMagZ.Text = xKKalman[9].ToString();
            }
            else
            {
                txtCalibMagX.Text = zKalman[6].ToString();
                txtCalibMagY.Text = zKalman[7].ToString();
                txtCalibMagZ.Text = zKalman[8].ToString();
            }
        }
        private void showTextRawMag(object sender, EventArgs e)
        {
            txtRawMagX.Text = magStr[0];
            txtRawMagY.Text = magStr[1];
            txtRawMagZ.Text = magStr[2];
        }
        private void showTextCalibAccel(object sender, EventArgs e)
        {
            if(fistScan)
            {
                txtCalibAccelX.Text = xKKalman[4].ToString();
                txtCalibAccelY.Text = xKKalman[5].ToString();
                txtCalibAccelZ.Text = xKKalman[6].ToString();
            }
            else
            {
                txtCalibAccelX.Text = zKalman[3].ToString();
                txtCalibAccelY.Text = zKalman[4].ToString();
                txtCalibAccelZ.Text = zKalman[5].ToString();
            }
        }
        private void showTextRawAccel(object sender, EventArgs e)
        {
            txtRawAccelX.Text = accelStr[0];
            txtRawAccelY.Text = accelStr[1];
            txtRawAccelZ.Text = accelStr[2];
        }
        private void showTextCalibGyro(object sender, EventArgs e)
        {
            if (fistScan)
            {
                txtCalibGyroX.Text = xKKalman[10].ToString();
                txtCalibGyroY.Text = xKKalman[11].ToString();
                txtCalibGyroZ.Text = xKKalman[12].ToString();
            }
            else
            {
                txtCalibGyroX.Text = zKalman[0].ToString();
                txtCalibGyroY.Text = zKalman[1].ToString();
                txtCalibGyroZ.Text = zKalman[2].ToString();
            }
        }
        private void showTextRawGyro(object sender, EventArgs e)
        {
            txtRawGyroX.Text = gyroStr[0];
            txtRawGyroY.Text = gyroStr[1];
            txtRawGyroZ.Text = gyroStr[2];
        }
        private void findOffsetGyro()
        {
            if(i_g == 100)
            {
                foreach (var i in valueGyroX)
                {
                    offsetGyro[0] += i;
                }
                foreach(var i in valueGyroY)
                {
                    offsetGyro[1] += i;
                }
                foreach (var i in valueGyroZ)
                {
                    offsetGyro[2] += i;
                }
                for (int j = 0; j < 3; j++)
                {
                    offsetGyro[j] /= i_g;
                }
                //
                this.Invoke(new EventHandler(showTextOffset));
                //
                i_g = 0; //reset
                triGyro = true;
            }
        }
        private void findOffsetAccel()
        {
            if (i_a == 100)
            {
                foreach (var i in valueAccelX)
                {
                    offsetAccel[0] += i;
                }
                foreach (var i in valueAccelY)
                {
                    offsetAccel[1] += i;
                }
                foreach (var i in valueAccelZ)//z = g
                {
                    offsetAccel[2] += i;
                }
                for (int j = 0; j < 2; j++)
                {
                    offsetAccel[j] /= i_a;
                }
                //offset z
                offsetAccel[2] /= i_a;
                offsetAccel[2] -= 1;
                //
                this.Invoke(new EventHandler(showTextOffset2));
                //
                i_a = 0; //reset
                trigAccel = true;
            }
        }
        private void showTextOffset2(object sender, EventArgs e)
        {
            txtOffsetAccelX.Text = offsetAccel[0].ToString(); //co the ko thuc thi
            txtOffsetAccelY.Text = offsetAccel[1].ToString();
            txtOffsetAccelZ.Text = offsetAccel[2].ToString();
        }
        private void showTextOffset(object sender, EventArgs e)
        {
            txtOffsetX.Text = offsetGyro[0].ToString(); //co the ko thuc thi
            txtOffsetY.Text = offsetGyro[1].ToString();
            txtOffsetZ.Text = offsetGyro[2].ToString();
        }
        private void convertCMag(string buff)
        {
            bool fistArr = true;
            bool trigArr = false;
            int countBuff = 0; 
            string[] xKalmanStr = new string[3];
            foreach (var i in buff)
            {
                if (!fistArr)
                {
                    if (trigArr) //xu ly gtri tiep theo
                    {   //m1 m2 m3 xk[7] xk[8] xk[9]
                        if (i != ';') xKalmanStr[countBuff] += i; //
                        else
                        {
                            countBuff++;
                            trigArr = false;
                        }
                    }
                    if (i == '\n') //bat dau xu ly
                    {
                        trigArr = true;
                    }
                }
                else
                {
                    if (i != ';') xKalmanStr[countBuff] += i;
                    else
                    {
                        countBuff++;
                        fistArr = false;
                    }
                } //xu ly gtri dau tien
            }
            if (fistScan)
            {
                for (int i = 0; i < 3; i++)
                {
                    if (double.TryParse(xKalmanStr[i], out xKKalman[i + 7]))
                    {
                        //
                    }
                }
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    if (double.TryParse(xKalmanStr[i], out zKalman[i + 6]))
                    {
                        //
                    }
                }
            }
        }
        private void convertCAccel(string buff)
        {
            bool fistArr = true;
            bool trigArr = false;
            int countBuff = 0;
            string[] xKKalmanStr = new string[3];
            foreach (var i in buff)
            {
                if (!fistArr)
                {
                    if (trigArr) //xu ly gtri tiep theo
                    {   //f1 f2 f3 xk[4] xk[5] xk[6]
                        if (i != ';') 
                        {
                            xKKalmanStr[countBuff] += i;
                        }
                        else
                        {
                            countBuff++;
                            trigArr = false;
                        }
                    }
                    if (i == '\n') //bat dau xu ly
                    {
                        trigArr = true;
                    }
                }
                else
                {
                    if (i != ';')
                    {
                        xKKalmanStr[countBuff] += i;
                    }
                    else
                    {
                        countBuff++;
                        fistArr = false;
                    }
                } //xu ly gtri dau tien
            }
            if (fistScan)
            {
                for (int i = 0; i < 3; i++)
                {
                    if (double.TryParse(xKKalmanStr[i], out xKKalman[i + 4]))
                    {
                        //
                    }
                }
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    if (double.TryParse(xKKalmanStr[i], out zKalman[i + 3]))
                    {
                        //
                    }
                }
            }
        }
        private void execMatrixAccel() //operate the matrix of the accel values
        {
            double[,] arrXAccel = new double[100, 6];
            for (int j = 0; j < 100; j++)
            {
                arrXAccel[j, 0] = (double)(valueAccelX[j] * valueAccelX[j]);
                arrXAccel[j, 1] = (double)(valueAccelY[j] * valueAccelY[j]);
                arrXAccel[j, 2] = (double)(valueAccelZ[j] * valueAccelZ[j]);
                arrXAccel[j, 3] = (double)(2 * valueAccelX[j]);
                arrXAccel[j, 4] = (double)(2 * valueAccelY[j]);
                arrXAccel[j, 5] = (double)(2 * valueAccelZ[j]);
            }
            //
            Matrix xAccel = new Matrix(arrXAccel);
            Matrix onesAccel = Matrix.Ones(100, 1);
            Matrix xTAccel = xAccel.Transpose();
            Matrix xTemp1Accel = xTAccel * xAccel;
            xTemp1Accel = xTemp1Accel.Inverse();
            Matrix xTemp2Accel = xTAccel * onesAccel;
            betaAccel = xTemp1Accel * xTemp2Accel;
            //
            convertBetaAccelToString(betaAccel.ToString());
            //program compute KDiag and o
            float[] tempHSBetaAccel = new float[6];
            trigAccel = true; //khoi tao trig
            for (int i = 0; i < tempHSBetaAccel.Length; i++)
            {
                if (!float.TryParse(hsBetaAccel[i], out tempHSBetaAccel[i]))
                {
                    MessageBox.Show("Cannot compute Accel matrix!!", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    trigAccel = false;
                }
            }
            if (trigAccel)
            {
                double[] o = new double[3];
                o[0] = (double)(-tempHSBetaAccel[0] / tempHSBetaAccel[3]);
                o[1] = (double)(-tempHSBetaAccel[1] / tempHSBetaAccel[4]);
                o[2] = (double)(-tempHSBetaAccel[2] / tempHSBetaAccel[5]);
                oAccel = new Matrix(o); //global var oAccel
                float G = 1 + (tempHSBetaAccel[3] * tempHSBetaAccel[3]) / tempHSBetaAccel[0] + (tempHSBetaAccel[4] * tempHSBetaAccel[4]) / tempHSBetaAccel[1]
                    + (tempHSBetaAccel[5] * tempHSBetaAccel[5]) / tempHSBetaAccel[2];
                double[,] KDiagArr = new double[3, 3];
                KDiagArr[0, 0] = 1 / (Math.Sqrt((double)(tempHSBetaAccel[0] / G)));
                KDiagArr[1, 1] = 1 / (Math.Sqrt((double)(tempHSBetaAccel[1] / G)));
                KDiagArr[2, 2] = 1 / (Math.Sqrt((double)(tempHSBetaAccel[2] / G)));
                //
                KAccelDiag = new Matrix(KDiagArr); //global var KAccel
                MessageBox.Show("Calibration complete(0x02)!!", "Successful", MessageBoxButtons.OK, MessageBoxIcon.Information);
            }
        }
        private void execMatrixMag()
        {
            double[,] arrXMag = new double[100, 6]; //0->5:0->9
            for (int j = 0; j < 100; j++)
            {
                arrXMag[j, 0] = (double)(valueCalibMagX[j] * valueCalibMagX[j]);
                arrXMag[j, 1] = (double)(valueCalibMagY[j] * valueCalibMagY[j]);
                arrXMag[j, 2] = (double)(valueCalibMagZ[j] * valueCalibMagZ[j]);
                arrXMag[j, 3] = (double)(2 * valueCalibMagX[j]);
                arrXMag[j, 4] = (double)(2 * valueCalibMagY[j]);
                arrXMag[j, 5] = (double)(2 * valueCalibMagZ[j]);
            }
            //
            Matrix xMag = new Matrix(arrXMag);
            Matrix onesMag = Matrix.Ones(100, 1);
            Matrix xTMag = xMag.Transpose();
            Matrix xTemp1Mag = xTMag * xMag;
            xTemp1Mag = xTemp1Mag.Inverse(); //  matrix (X'X)*
            Matrix xTemp2Mag = xTMag * onesMag; // matrix (X'ones)

            betaMag = xTemp1Mag * xTemp2Mag;
            //buf = betaMag.ToString();
            convertFromMatrixToString(betaMag.ToString());
            //
            //prog matrix calib
            float[] tempHSBeta = new float[6];
            trigExecMatrixMag = true; //su dung for o duoi
            for (int i = 0; i < tempHSBeta.Length; i++)
            {
                if (!float.TryParse(hsBeta[i], out tempHSBeta[i]))
                {
                    MessageBox.Show("Cannot compute magnetic matrix!!", "Warning", MessageBoxButtons.OK, MessageBoxIcon.Warning);
                    trigExecMatrixMag = false;
                }
            }
            if (trigExecMatrixMag) //neu k fault chuyen doi du lieu
            {
                double[] o = new double[3];
                o[0] = (double)(-tempHSBeta[0] / tempHSBeta[3]); // -a/g
                o[1] = (double)(-tempHSBeta[1] / tempHSBeta[4]); // -b/h
                o[2] = (double)(-tempHSBeta[2] / tempHSBeta[5]); // -b/h
                oMag = new Matrix(o); //oMag is global var
                float G = 1 + (tempHSBeta[3] * tempHSBeta[3]) / tempHSBeta[0] + (tempHSBeta[4] * tempHSBeta[4]) / tempHSBeta[1]
                    + (tempHSBeta[5] * tempHSBeta[5]) / tempHSBeta[2]; // G = 1 + g^2/a + h^2/b + i^2/c
                double[,] KDiagArr = new double[3, 3];
                KDiagArr[0, 0] = 1/(Math.Sqrt((double)(tempHSBeta[0] / G))); //sqrt(a/G) 
                KDiagArr[1, 1] = 1/(Math.Sqrt((double)(tempHSBeta[1] / G))); //sqrt(b/G)
                KDiagArr[2, 2] = 1/(Math.Sqrt((double)(tempHSBeta[2] / G))); //sqrt(a/G)
                KMagDiag = new Matrix(KDiagArr); //K Diag - global var
                MessageBox.Show("Calibration complete(0x01)!!", "Successful", MessageBoxButtons.OK, MessageBoxIcon.Information);
                //chon trigExecMatrixMag = true de tinh toan calib trong frame m
            }

        }
        private void convertFromMatrixToString(string buff)
        {
            bool fistArr = true;
            bool trigArr = false;
            int countBuff = 0;
            foreach (var i in buff)
            {
                if (!fistArr)
                {
                    if (trigArr) //xu ly gtri tiep theo
                    {
                        if (i != ';') hsBeta[countBuff] += i;
                        else
                        {
                            countBuff++;
                            trigArr = false;
                        }
                    }
                    if (i == '\n') //bat dau xu ly
                    {
                        trigArr = true;
                    }
                }
                else
                {
                    if (i != ';') hsBeta[0] += i;
                    else
                    {
                        countBuff++;
                        fistArr = false;
                    }
                } //xu ly gtri dau tien
            }

        }

        private void btn4Mode(object sender, EventArgs e)
        {
            mode = !mode;
            if (mode == true)
            {
                btn4.Text = "Mode calib";
            }
            else btn4.Text = "Mode Raw";
        }

        private void btnCalibGyro_Click(object sender, EventArgs e)
        {
            serialPort1.Write("D");
        }

        private void btnCalibAccel_Click(object sender, EventArgs e)
        {
            try
            {
                serialPort1.Write("A");
                calibAccelMode = false;
            }
            catch
            {
                MessageBox.Show("Cannot receive data", "Warning");
            }
            
        }

        private void btnCalibMag_Click(object sender, EventArgs e)
        {
            serialPort1.Write("M");
        }

        private void btnStartIMU_Click(object sender, EventArgs e)
        {
            serialPort1.Write("S");
        }

        private void btnStopIMU_Click(object sender, EventArgs e)
        {
            serialPort1.Write("T");
        }

        private void button2_Click(object sender, EventArgs e)
        {
            txtReceived.Text = "";
        }

        private void convertBetaAccelToString(string buff)
        {
            bool fistArr = true;
            bool trigArr = false;
            int countBuff = 0;
            foreach (var i in buff)
            {
                if (!fistArr)
                {
                    if (trigArr) //xu ly gtri tiep theo
                    {
                        if (i != ';') hsBetaAccel[countBuff] += i;
                        else
                        {
                            countBuff++;
                            trigArr = false;
                        }
                    }
                    if (i == '\n') //bat dau xu ly
                    {
                        trigArr = true;
                    }
                }
                else
                {
                    if (i != ';') hsBetaAccel[0] += i;
                    else
                    {
                        countBuff++;
                        fistArr = false;
                    }
                } //xu ly gtri dau tien
            }

        }

        private void button1_Click_1(object sender, EventArgs e)
        {
            serialPort1.Write("A");
            calibAccelMode = true;
        }

        private string clearLetters(char c)
        {
            while (dataIn[indexData] != c)
            {
                letters[indexLetter] = dataIn[indexData];
                indexLetter++;
                indexData++;
            }
            indexLetter = 0;
            indexData++;
            string k = new string(letters);
            for (int i = 0; i < letters.Length; i++)
            {
                letters[i] = Convert.ToChar(0);
            }
            return k;
        }
        private void kalmanFilter()
        { 
            //khoi dong de nhay vo ham if
            //ban dau trigKalman = false
            //
            //khi dua gia tri gyro vao ham se chuyen qua rad
            //
            if (trigKalman)
            { //q1 q2 q3 q4 xk[0] xk[1] xk[2] xk[3]
                if (fistScan)
                {
                    float[] accel = new float[3];
                    for (int i = 0; i < 3; i++)
                    {
                        accel[i] = float.Parse(accelStr[i]); //x y z
                    }
                    //gia su ban dau lay Ax = 0; Ay = 0; Az = g
                    float roll0 = (float)Math.Atan(accel[0] / Math.Sqrt(accel[0] * accel[0] + accel[2] * accel[2]));//rad //xem lai ham atan va atan2
                    float pitch0 = (float)Math.Atan(Math.Sqrt(accel[0] * accel[0] + accel[1] * accel[1]) / accel[2]); //radian
                    float yaw0 = (float)Math.Atan(accel[0] / Math.Sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
                    rollf1 = roll0;
                    pitchf1 = pitch0;
                    yawf1 = yaw0;
                    fistScan = false; //hien tai chi su dung duy nhat 1 lan trong chuong trinh
                                      //
                    for (int i = 0; i < 3; i++) //chuyen w sang rad
                    {
                        xKKalman[i + 10] = convertDegreeToRad((float)xKKalman[i + 10]);
                    }

                    double[] rpy = new double[3]; //chuyen sang type double
                    rpy[0] = convertDegreeToRad(rollf1); //roll
                    rpy[1] = convertDegreeToRad(pitchf1); //pitch
                    rpy[2] = convertDegreeToRad(yawf1); //yaw
                                                        //quanterion
                    xKKalman[0] = (Math.Cos(rpy[0] / 2) * Math.Cos(rpy[1] / 2) * Math.Cos(rpy[2] / 2) + Math.Sin(rpy[0] / 2) * Math.Sin(rpy[1] / 2) * Math.Sin(rpy[2] / 2));
                    xKKalman[1] = (Math.Sin(rpy[0] / 2) * Math.Cos(rpy[1] / 2) * Math.Cos(rpy[2] / 2) - Math.Cos(rpy[0] / 2) * Math.Sin(rpy[1] / 2) * Math.Sin(rpy[2] / 2));
                    xKKalman[2] = (Math.Cos(rpy[0] / 2) * Math.Sin(rpy[1] / 2) * Math.Cos(rpy[2] / 2) + Math.Sin(rpy[0] / 2) * Math.Cos(rpy[1] / 2) * Math.Sin(rpy[2] / 2));
                    xKKalman[3] = (Math.Cos(rpy[0] / 2) * Math.Cos(rpy[1] / 2) * Math.Sin(rpy[2] / 2) - Math.Sin(rpy[0] / 2) * Math.Sin(rpy[1] / 2) * Math.Cos(rpy[2] / 2));
                    //x(k + 1) round tiep theo
                    xKp1Kalman[0] = xKKalman[0] - (tSample / 2) * (xKKalman[1] * xKKalman[10] + xKKalman[2] * xKKalman[11] + xKKalman[3] * xKKalman[12]);
                    xKp1Kalman[1] = xKKalman[1] - (tSample / 2) * (xKKalman[0] * xKKalman[10] + xKKalman[3] * xKKalman[11] + xKKalman[2] * xKKalman[12]);
                    xKp1Kalman[2] = xKKalman[2] - (tSample / 2) * (xKKalman[3] * xKKalman[10] + xKKalman[0] * xKKalman[11] + xKKalman[1] * xKKalman[12]);
                    xKp1Kalman[3] = xKKalman[3] - (tSample / 2) * (xKKalman[2] * xKKalman[10] + xKKalman[1] * xKKalman[11] + xKKalman[0] * xKKalman[12]);
                    xKp1Kalman[4] = 0.98f * xKKalman[4];
                    xKp1Kalman[5] = 0.98f * xKKalman[5];
                    xKp1Kalman[6] = 0.98f * xKKalman[6];
                    xKp1Kalman[7] = 0.98f * xKKalman[7];
                    xKp1Kalman[8] = 0.98f * xKKalman[8];
                    xKp1Kalman[9] = 0.98f * xKKalman[9];
                    xKp1Kalman[10] = 0.0f;
                    xKp1Kalman[11] = 0.0f;
                    xKp1Kalman[12] = 0.0f;
                    //
                    //tim F_k
                    F_k[0, 1] = F_k[3, 2] = tSample / 2 * xKKalman[10];
                    F_k[0, 2] = F_k[1, 3] = tSample / 2 * xKKalman[11];
                    F_k[0, 3] = F_k[2, 1] = tSample / 2 * xKKalman[12];
                    F_k[0, 10] = F_k[2, 12] = -(tSample / 2) * xKKalman[1];
                    F_k[0, 11] = F_k[3, 10] = -(tSample / 2) * xKKalman[2];
                    F_k[0, 12] = F_k[1, 11] = -(tSample / 2) * xKKalman[3];
                    F_k[1, 0] = F_k[2, 3] = tSample / 2 * xKKalman[10];
                    F_k[1, 10] = F_k[3, 12] = F_k[2, 11] = tSample / 2 * xKKalman[0];
                    F_k[1, 12] = tSample / 2 * xKKalman[2];
                    F_k[2, 10] = tSample / 2 * xKKalman[3];
                    F_k[2, 0] = F_k[3, 1] = tSample / 2 * xKKalman[11];
                    F_k[3, 0] = F_k[1, 2] = tSample / 2 * xKKalman[12];
                    //
                    AKalman = new Matrix(F_k);
                    PKalman = AKalman * PKalman * (AKalman.Transpose()); //Pk = A * Pk- * A'
                }
                else
                {
                    //da tim dc z(k)
                    //tim h
                    h_k[3, 0] = -2 * xKKalman[2];
                    h_k[3, 1] = 2 * xKKalman[3];
                    h_k[3, 2] = -2 * xKKalman[0];
                    h_k[3, 3] = 2 * xKKalman[1];
                    //
                    h_k[4, 0] = 2 * xKKalman[1];
                    h_k[4, 1] = 2 * xKKalman[0];
                    h_k[4, 2] = 2 * xKKalman[3];
                    h_k[4, 3] = 2 * xKKalman[2];
                    //
                    h_k[5, 0] = 2 * xKKalman[0];
                    h_k[5, 1] = -2 * xKKalman[1];
                    h_k[5, 2] = -2 * xKKalman[2];
                    h_k[5, 3] = 2 * xKKalman[3];
                    //
                    h_k[6, 0] = 2 * Math.Cos(iMag) * xKKalman[0] - 2 * Math.Sin(iMag) * xKKalman[2];
                    h_k[6, 1] = 2 * Math.Cos(iMag) * xKKalman[1] + 2 * Math.Sin(iMag) * xKKalman[3];
                    h_k[6, 2] = -2 * Math.Cos(iMag) * xKKalman[2] - 2 * Math.Sin(iMag) * xKKalman[0];
                    h_k[6, 3] = -2 * Math.Cos(iMag) * xKKalman[3] + 2 * Math.Sin(iMag) * xKKalman[1];
                    //
                    h_k[7, 0] = -2 * Math.Cos(iMag) * xKKalman[3] + 2 * Math.Sin(iMag) * xKKalman[1];
                    h_k[7, 1] = 2 * Math.Cos(iMag) * xKKalman[2] + 2 * Math.Sin(iMag) * xKKalman[1];
                    h_k[7, 2] = 2 * Math.Cos(iMag) * xKKalman[1] + 2 * Math.Sin(iMag) * xKKalman[3];
                    h_k[7, 3] = -2 * Math.Cos(iMag) * xKKalman[0] + 2 * Math.Sin(iMag) * xKKalman[2];
                    //
                    h_k[8, 0] = 2 * Math.Cos(iMag) * xKKalman[2] + 2 * Math.Sin(iMag) * xKKalman[0];
                    h_k[8, 1] = 2 * Math.Cos(iMag) * xKKalman[3] - 2 * Math.Sin(iMag) * xKKalman[1];
                    h_k[8, 2] = 2 * Math.Cos(iMag) * xKKalman[0] - 2 * Math.Sin(iMag) * xKKalman[2];
                    h_k[8, 3] = 2 * Math.Cos(iMag) * xKKalman[1] + 2 * Math.Sin(iMag) * xKKalman[3];
                    hKalman = new Matrix(h_k);
                    //
                    kKKalman = PKalman * hKalman.Transpose() * (hKalman * PKalman * hKalman.Transpose() + rKalman); // hs k
                    xKalmanMatrix = new Matrix(xKp1Kalman); //lay predict x(k+1) trc do
                    zKalmanMatrix = new Matrix(zKalman);
                    xKp1KalmanMatrix = xKalmanMatrix + kKKalman * (zKalmanMatrix - hKalman * xKalmanMatrix); //x predict
                    Matrix iKalman = Matrix.Identity(13);
                    PKalman = (iKalman - kKKalman * hKalman) * PKalman;
                    //
                    convertXKp1(xKp1KalmanMatrix.ToString());

                }
                rollf1 = (float)Math.Atan2(2 * (xKp1Kalman[0] * xKp1Kalman[1] + xKp1Kalman[2] * xKp1Kalman[3]), xKp1Kalman[0] * xKp1Kalman[0] - xKp1Kalman[1] * xKp1Kalman[1] - xKp1Kalman[2] * xKp1Kalman[2] + xKp1Kalman[3] * xKp1Kalman[3]);
                pitchf1 = (float)Math.Asin(-2 * (xKp1Kalman[0] * xKp1Kalman[2] - xKp1Kalman[1] * xKp1Kalman[3]));
                yawf1 = (float)Math.Atan2(2 * (xKp1Kalman[1] * xKp1Kalman[2] + xKp1Kalman[0] * xKp1Kalman[3]), xKp1Kalman[0] * xKp1Kalman[0] + xKp1Kalman[1] * xKp1Kalman[1] - xKp1Kalman[2] * xKp1Kalman[2] - xKp1Kalman[3] * xKp1Kalman[3]);
                //sync
                syncAccel = false;
                syncGyro = false;
                syncMag = false;
            }
        }
        private void convertXKp1(string buff)
        {
            bool fistArr = true;
            bool trigArr = false;
            int countBuff = 0;
            string[] xKKalmanStr = new string[13];
            foreach (var i in buff)
            {
                if (!fistArr)
                {
                    if (trigArr) //xu ly gtri tiep theo
                    {   //f1 f2 f3 xk[4] xk[5] xk[6]
                        if (i != ';')
                        {
                            xKKalmanStr[countBuff] += i;
                        }
                        else
                        {
                            countBuff++;
                            trigArr = false;
                        }
                    }
                    if (i == '\n') //bat dau xu ly
                    {
                        trigArr = true;
                    }
                }
                else
                {
                    if (i != ';')
                    {
                        xKKalmanStr[countBuff] += i;
                    }
                    else
                    {
                        countBuff++;
                        fistArr = false;
                    }
                } //xu ly gtri dau tien
            }
            for (int i = 0; i < 13; i++)
            {
                if (double.TryParse(xKKalmanStr[i], out xKp1Kalman[i]))
                {
                    //
                }
            }
        }
        private double convertDegreeToRad(float bufferIn)
        {
            double bufOut;
            bufOut = ((double)bufferIn * Math.PI) / 180;
            return bufOut;
        }
        private void cbx7_SelectedIndexChanged_1(object sender, EventArgs e)
        {
            ComboBox cbx7 = sender as ComboBox;
            switch (Convert.ToInt32(cbx7.SelectedIndex))
            {
                case 0:
                    gyro_config = 20;
                    break;
                case 1:
                    gyro_config = 21;
                    break;
                case 2:
                    gyro_config = 22;
                    break;
                case 3:
                    gyro_config = 23;
                    break;
                default:
                    break;
            }
            lbstatus.Text = "Status: Value choosen is " + Convert.ToString(gyro_config);
        }
    }
}
