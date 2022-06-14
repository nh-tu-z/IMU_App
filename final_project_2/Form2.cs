using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

using Microsoft.DirectX;
using Microsoft.DirectX.Direct3D;

namespace final_project2
{
    public partial class Form2 : Form
    {
        private float vectorX = 2.0f, vectorY = 1.0f, vectorZ = 2.0f;
        //
        private Device device = null;

        private VertexBuffer vb = null;
        private IndexBuffer ib = null;

        private VertexBuffer vb2 = null;
        private IndexBuffer ib2 = null;

        private static int terWith = 10;
        private static int terLength = 10;
        private static int terHeight = 10;

        private static int verCount = terWith * terLength + terWith * terHeight + terLength * terHeight;
        private static int indCount = (terWith - 1) * (terLength - 1) * 6 + (terWith - 1) * (terHeight - 1) * 6 + (terLength - 1) * (terHeight - 1) * 6;

        private Vector3 camPosition, camLookAt, camUp;

        private float moveSpeed = 0.2f;

        private float turnSpeed = 0.05f;
        private float rotY = 0;

        private float tempY = 0;
        bool isMiddleMouseDown = false;

        private static int[] indices = null;
        private static int[] indices2 = null;

        CustomVertex.PositionColored[] verts = null;
        CustomVertex.PositionColored[] verts2 = null;

        private float angle = 0.0f;

        private float roll = 0, pitch = 0, yaw = 0;
        //private bool invalidating = true;
        public bool enable = true;
        public Form2()
        {
            this.SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.Opaque, true);
            InitializeComponent();

            InitializeGraphics();

            InitializeEventHandler();

            GetValueRollPitchYaw(0, 0, 0);
        }
        public void GetValueRollPitchYaw(float r, float p, float y)
        {
            roll = r;
            pitch = p;
            yaw = y;
            //rollTxt.Text = r.ToString();
            //pitchTxt.Text = p.ToString();
            //yawTxt.Text = y.ToString();
            //roll = r;
            //pitch = p;
            //yaw = y;
        }
        private void InitializeGraphics()
        {
            PresentParameters pp = new PresentParameters();
            pp.Windowed = true;
            pp.SwapEffect = SwapEffect.Discard;

            pp.EnableAutoDepthStencil = true;
            pp.AutoDepthStencilFormat = DepthFormat.D16;

            device = new Device(0, DeviceType.Hardware, this, CreateFlags.HardwareVertexProcessing, pp);

            GenerateVertex();
            GenerateIndex();
            GenerateNewVertex();
            GenerateNewIndex();

            vb = new VertexBuffer(typeof(CustomVertex.PositionColored), verCount, device, Usage.Dynamic | Usage.WriteOnly, CustomVertex.PositionColored.Format, Pool.Default);
            OnVertexBufferCreate(vb, null);

            vb2 = new VertexBuffer(typeof(CustomVertex.PositionColored), 8, device, Usage.Dynamic | Usage.WriteOnly, CustomVertex.PositionColored.Format, Pool.Default);
            OnVertexBufferCreate2(vb2, null);

            ib = new IndexBuffer(typeof(int), indCount, device, Usage.WriteOnly, Pool.Default);
            OnIndexBufferCreate(ib, null);

            ib2 = new IndexBuffer(typeof(int), indices2.Length, device, Usage.WriteOnly, Pool.Default);
            OnIndexBufferCreate2(ib2, null);

            camPosition = new Vector3(2, 4.5f, -3.5f); // co giai thich he so trong series 4
            camLookAt = new Vector3(2, 3.5f, -2.5f);
            camUp = new Vector3(0, 1, 0);

            
        }

        private void InitializeEventHandler()
        {
            vb.Created += new EventHandler(OnVertexBufferCreate);
            ib.Created += new EventHandler(OnIndexBufferCreate);

            vb2.Created += new EventHandler(OnVertexBufferCreate2);
            ib2.Created += new EventHandler(OnIndexBufferCreate2);

            this.KeyDown += new KeyEventHandler(OnKeyDown);
            this.MouseWheel += new MouseEventHandler(OnMouseScroll);

            //7
            this.MouseMove += new MouseEventHandler(OnMouseMove);
            this.MouseDown += new MouseEventHandler(OnMouseDown);
            this.MouseUp += new MouseEventHandler(OnMouseUp);
        }

        private void OnIndexBufferCreate(object sender, EventArgs e)
        {
            IndexBuffer buffer = (IndexBuffer)sender;



            buffer.SetData(indices, 0, LockFlags.None);
        }

        private void OnVertexBufferCreate(object sender, EventArgs e)
        {
            VertexBuffer buffer = (VertexBuffer)sender;



            buffer.SetData(verts, 0, LockFlags.None);

        }

        private void OnIndexBufferCreate2(object sender, EventArgs e)
        {
            IndexBuffer buffer = (IndexBuffer)sender;



            buffer.SetData(indices2, 0, LockFlags.None);
            
        }

        private void OnVertexBufferCreate2(object sender, EventArgs e)
        {
            VertexBuffer buffer = (VertexBuffer)sender;



            buffer.SetData(verts2, 0, LockFlags.None);
            

        }

        private void SetUpCamera()
        {
            //5
            camLookAt.X = (float)Math.Sin(rotY) + camPosition.X; //chinh theo ti le camPosition va camLookAt //(float)Math.sin(rotY) duoc them vao tu series 6
            camLookAt.Y = camPosition.Y - 1;
            camLookAt.Z = (float)Math.Cos(rotY) + camPosition.Z; //(float)Math.sin(rotY) duoc them vao tu series 6 va chuyen Z - 1 --> Z

            //
            device.Transform.Projection = Matrix.PerspectiveFovLH((float)Math.PI / 4, this.Width / this.Height, 1.0f, 100.0f); //also test for perspectivefovRH
            device.Transform.View = Matrix.LookAtLH(camPosition, camLookAt, camUp); //also test on LookAtRH //series 4 doi  new Vector3(0, 0, 10), new Vector3(), new Vector3(0, 1, 0) --> camPosition, camLookAt, camUp

            device.Transform.World = Matrix.RotationYawPitchRoll(angle / (float)Math.PI, angle / (float)Math.PI / 2, angle / (float)Math.PI * 4);
            //device.Transform.World = Matrix.RotationX(angle);
            //1
            //angle += 0.05f;
            device.RenderState.Lighting = false;

            //3
            device.RenderState.FillMode = FillMode.WireFrame; // co the doi property cua fillmode

            //1
            device.RenderState.CullMode = Cull.None;
        }

        private void GenerateVertex()
        {
            verts = new CustomVertex.PositionColored[verCount];

            int k = 0;


            for (int z = 0; z < terWith; z++)
            {
                for (int x = 0; x < terLength; x++)
                {
                    verts[k].Position = new Vector3(x, 0, z);
                    verts[k].Color = Color.White.ToArgb();

                    k++;
                }
            }
            for (int y = 0; y < terHeight; y++)
            {
                for (int z = 0; z < terWith; z++)
                {
                    verts[k].Position = new Vector3(0, y, z);
                    verts[k].Color = Color.Aqua.ToArgb();
                    k++;
                }
            }
            for (int y = 0; y < terHeight; y++)
            {
                for (int x = 0; x < terLength; x++)
                {
                    verts[k].Position = new Vector3(x, y, 0);
                    verts[k].Color = Color.Gold.ToArgb();
                    k++;
                }
            }
        }

        private void GenerateNewVertex()
        {
            verts2 = new CustomVertex.PositionColored[8];

            verts2[0] = new CustomVertex.PositionColored(-1, 1, 1, Color.Red.ToArgb()); //indexbuffer chi lay 0124-6789. cap mau diem 0-9 | 1-8 | 2-7 | 4-6
            verts2[1] = new CustomVertex.PositionColored(-1, -1, 1, Color.Red.ToArgb());
            verts2[2] = new CustomVertex.PositionColored(1, 1, 1, Color.Blue.ToArgb());
            verts2[3] = new CustomVertex.PositionColored(1, -1, 1, Color.Blue.ToArgb());
            verts2[4] = new CustomVertex.PositionColored(-1, 1, -1, Color.Olive.ToArgb());
            verts2[5] = new CustomVertex.PositionColored(1, 1, -1, Color.Pink.ToArgb());
            verts2[6] = new CustomVertex.PositionColored(-1, -1, -1, Color.Olive.ToArgb()); //swap 2 ordinate x va y
            verts2[7] = new CustomVertex.PositionColored(1, -1, -1, Color.Pink.ToArgb());
        }

        private void GenerateIndex()
        {
            indices = new int[indCount];

            int k = 0;
            int l = 0;

            for (int i = 0; i < ((terWith - 1) * (terLength - 1) * 6); i += 6)
            {
                indices[i] = k;
                indices[i + 1] = k + terLength;
                indices[i + 2] = k + terLength + 1;
                indices[i + 3] = k;
                indices[i + 4] = k + terLength + 1;
                indices[i + 5] = k + 1;

                k++;
                l++;
                if (l == terLength - 1)
                {
                    l = 0;
                    k++;
                }
            }
            l = 0;
            k = terWith * terLength;
            for (int i = ((terWith - 1) * (terLength - 1) * 6); i < ((terWith - 1) * (terLength - 1) * 6 + (terWith - 1) * (terHeight - 1) * 6); i += 6)
            {
                indices[i] = k;
                indices[i + 1] = k + terLength;
                indices[i + 2] = k + terLength + 1;
                indices[i + 3] = k;
                indices[i + 4] = k + terLength + 1;
                indices[i + 5] = k + 1;

                k++;
                l++;
                if (l == terWith - 1)
                {
                    l = 0;
                    k++;
                }
            }
            l = 0;
            k = terWith * terLength + terWith * terHeight;
            for (int i = ((terWith - 1) * (terLength - 1) * 6 + (terWith - 1) * (terHeight - 1) * 6); i < indCount; i += 6)
            {
                indices[i] = k;
                indices[i + 1] = k + terLength;
                indices[i + 2] = k + terLength + 1;
                indices[i + 3] = k;
                indices[i + 4] = k + terLength + 1;
                indices[i + 5] = k + 1;

                k++;
                l++;
                if (l == terWith - 1)
                {
                    l = 0;
                    k++;
                }
            }

        }

        private void GenerateNewIndex()
        {
            indices2 = new int[] {
            0, 1, 2,  1, 3, 2, //frontside
            4, 5, 6,  6, 5, 7, //backside
            0, 5, 4,  0, 2, 5, //top
            1, 6, 7,  1, 7, 3, //bottom
            0, 6, 1,  4, 6, 0, //left side
            2, 3, 7,  5, 2, 7 //right side
                    };
        }
        private void OnKeyDown(object sender, KeyEventArgs e)
        {
            switch (e.KeyCode)
            {
                case (Keys.W):
                    {
                        camPosition.X += moveSpeed * (float)Math.Sin(rotY);  //chuyen tu camPosiion.Z += moveSpeed --> nhu trong code
                        camPosition.Z += moveSpeed * (float)Math.Cos(rotY);
                        break;
                    }
                case (Keys.D):
                    {
                        camPosition.X += moveSpeed * (float)Math.Sin(rotY + Math.PI / 2); //chuyen tu camPosiion.Z += moveSpeed --> nhu trong code
                        camPosition.Z += moveSpeed * (float)Math.Cos(rotY + Math.PI / 2);
                        break;
                    }
                case (Keys.S):
                    {
                        camPosition.X -= moveSpeed * (float)Math.Sin(rotY); //chuyen tu camPosiion.Z += moveSpeed --> nhu trong code
                        camPosition.Z -= moveSpeed * (float)Math.Cos(rotY);
                        break;
                    }
                case (Keys.A):
                    {
                        camPosition.X -= moveSpeed * (float)Math.Sin(rotY + Math.PI / 2); //chuyen tu camPosiion.Z += moveSpeed --> nhu trong code
                        camPosition.Z -= moveSpeed * (float)Math.Cos(rotY + Math.PI / 2);
                        break;
                    }
                case (Keys.Q):
                    {
                        rotY -= turnSpeed;
                        break;
                    }
                case (Keys.E):
                    {
                        rotY += turnSpeed;
                        break;
                    }
                case (Keys.Up): //up x
                    {
                        //
                        vectorZ += 0.2f;
                    }
                    break;
                case (Keys.Down): // down x
                    {
                        //
                        vectorZ -= 0.2f;
                    }
                    break;
                case (Keys.Left): // up y
                    {
                        //
                        vectorX -= 0.2f;
                    }
                    break;
                case (Keys.Right): // down y
                    {
                        //
                        vectorX += 0.2f;
                    }
                    break;
                case (Keys.NumPad1): //up z
                    {
                        //
                        vectorY += 0.2f;
                    }
                    break;
                case (Keys.NumPad2): // down z
                    {
                        //
                        vectorY -= 0.2f;
                    }
                    break;
                case (Keys.NumPad4): // down z
                    {
                        //
                        roll = 45;
                        pitch = 0;
                        yaw = 0;
                    }
                    break;
                case (Keys.NumPad5): // down z
                    {
                        //
                        roll = 0;
                        pitch = 45;
                        yaw = 0;
                    }
                    break;
                case (Keys.NumPad6): // down z
                    {
                        //
                        roll = 0;
                        pitch = 0;
                        yaw = 45;
                    }
                    break;
                case (Keys.NumPad3): // down z
                    {
                        //
                        roll = 0;
                        pitch = 0;
                        yaw = 0;
                    }
                    break;
                case (Keys.NumPad7): // down z
                    {
                        //
                        roll += 1.0f;
                    }
                    break;
                case (Keys.NumPad8): // down z
                    {
                        //
                        pitch += 1.0f;
                    }
                    break;
                case (Keys.NumPad9): // down z
                    {
                        //
                        yaw += 1.0f;
                    }
                    break;
            }
        }
        private void OnMouseScroll(object sender, MouseEventArgs e)
        {
            camPosition.Y -= e.Delta * 0.01f;
        }

        private void Form2_FormClosing(object sender, FormClosingEventArgs e)
        {
            enable = false;
        }

        private void Form2_Load(object sender, EventArgs e)
        {
            
        }



        /* private void btn1_Click(object sender, EventArgs e)
         {
             bool toggle = !invalidating;
             if(toggle == true)
             {
                 btn1.Text = "Pause";
                 invalidating = true;

             }
             else
             {
                 btn1.Text = "Plot";
                 invalidating = false;
             }
         }*/

        private void OnMouseMove(object sender, MouseEventArgs e)
        {
            if (isMiddleMouseDown)
            {
                rotY = tempY + e.X * turnSpeed;
            }
        }

        private void Form2_Paint(object sender, PaintEventArgs e)
        {
            device.Clear(ClearFlags.Target | ClearFlags.ZBuffer, Color.Black, 1, 0); //clearflags.zbuffer la set up cho phan ve de len nhau
            SetUpCamera();
            //
            device.BeginScene();

            device.VertexFormat = CustomVertex.PositionColored.Format;
            device.SetStreamSource(0, vb, 0);
            //2
            device.Indices = ib;

            //3
            device.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, verCount, 0, indCount / 3);

            device.EndScene();
            device.RenderState.FillMode = FillMode.Solid;
            device.SetStreamSource(0, vb2, 0);
            device.Indices = ib2;

            rollTxt.Text = roll.ToString();
            pitchTxt.Text = pitch.ToString();
            yawTxt.Text = yaw.ToString();

            float rx = (roll) * ((float)Math.PI) / 180;
            float py = (pitch) * ((float)Math.PI) / 180;
            float yz = (yaw) * ((float)Math.PI) / 180;

            rollTxt.Update();
            pitchTxt.Update();
            yawTxt.Update();
            label1.Update();
            label2.Update();
            label3.Update();
            label4.Update();
            label4.Update();
            label5.Update();
            label6.Update();
            label7.Update();
            label8.Update();
            //btn1.Update();

            // txtbox1.Update();
            // txtbox2.Update();
            //  txtbox3.Update();
            //roll = (float.Parse(rollTxt.Text)) * ((float)Math.PI) / 180;
            //pitch = (float.Parse(pitchTxt.Text)) * ((float)Math.PI) / 180;
            //yaw = (float.Parse(yawTxt.Text)) * ((float)Math.PI) / 180;
            device.Transform.World = Matrix.RotationYawPitchRoll(yz, rx, py) * Matrix.Translation(vectorX, vectorY, vectorZ);
            device.DrawIndexedPrimitives(PrimitiveType.TriangleList, 0, 0, 8, 0, 12);
            
            //
            device.Present();
            //if (invalidating)
            //{
                this.Invalidate();
            //}
        }

        private void OnMouseDown(object sender, MouseEventArgs e)
        {
            switch (e.Button)
            {
                case (MouseButtons.Left):
                    {
                        tempY = rotY - e.X * turnSpeed;
                        isMiddleMouseDown = true;
                        break;
                    }
            }
        }
        private void OnMouseUp(object sender, MouseEventArgs e)
        {
            switch (e.Button)
            {
                case (MouseButtons.Left):
                    {
                        isMiddleMouseDown = false;
                        break;
                    }
            }
        }
    }
}
