
namespace CSharpCustomPanelControl
{

	public class Form1 : System.Windows.Forms.Form
	{
		#region Windows Form Designer generated code
		//Form overrides dispose to clean up the component list.

		public Form1() : base()
		{
			//This call is required by the Windows Form Designer.
			InitializeComponent();
			//Add any initialization after the InitializeComponent() call
		}

		protected override void Dispose(bool disposing)
		{
			if (disposing) 
			{
				if (!((components == null))) 
				{
					components.Dispose();
				}
			}
			base.Dispose(disposing);
		}
		//Required by the Windows Form Designer
		internal CustomPanel CustomPanel1;
		//NOTE: The following procedure is required by the Windows Form Designer
		//It can be modified using the Windows Form Designer.  
		//Do not modify it using the code editor.
		private System.ComponentModel.IContainer components;
		internal CSharpCustomPanelControl.CustomPanel CustomPanel2;
		internal CSharpCustomPanelControl.CustomPanel CustomPanel3;
		internal CSharpCustomPanelControl.CustomPanel CustomPanel4;
		internal CSharpCustomPanelControl.CustomPanel CustomPanel5;
		internal CSharpCustomPanelControl.CustomPanel CustomPanel6;

		[System.Diagnostics.DebuggerStepThrough()]
		private void InitializeComponent()
		{
			this.CustomPanel1 = new CSharpCustomPanelControl.CustomPanel();
			this.CustomPanel2 = new CSharpCustomPanelControl.CustomPanel();
			this.CustomPanel3 = new CSharpCustomPanelControl.CustomPanel();
			this.CustomPanel4 = new CSharpCustomPanelControl.CustomPanel();
			this.CustomPanel5 = new CSharpCustomPanelControl.CustomPanel();
			this.CustomPanel6 = new CSharpCustomPanelControl.CustomPanel();
			this.CustomPanel1.SuspendLayout();
			this.SuspendLayout();
			// 
			// CustomPanel1
			// 
			this.CustomPanel1.BackColor = System.Drawing.SystemColors.Highlight;
			this.CustomPanel1.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D;
			this.CustomPanel1.Controls.Add(this.CustomPanel2);
			this.CustomPanel1.GradientMode = CSharpCustomPanelControl.LinearGradientMode.BackwardDiagonal;
			this.CustomPanel1.Location = new System.Drawing.Point(24, 16);
			this.CustomPanel1.Name = "CustomPanel1";
			this.CustomPanel1.Size = new System.Drawing.Size(200, 320);
			this.CustomPanel1.TabIndex = 0;
			// 
			// CustomPanel2
			// 
			this.CustomPanel2.BackColor = System.Drawing.Color.PeachPuff;
			this.CustomPanel2.BorderColor = System.Drawing.SystemColors.ControlLightLight;
			this.CustomPanel2.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
			this.CustomPanel2.Curvature = 20;
			this.CustomPanel2.Location = new System.Drawing.Point(32, 96);
			this.CustomPanel2.Name = "CustomPanel2";
			this.CustomPanel2.Size = new System.Drawing.Size(136, 160);
			this.CustomPanel2.TabIndex = 1;
			// 
			// CustomPanel3
			// 
			this.CustomPanel3.BackColor = System.Drawing.SystemColors.InactiveCaptionText;
			this.CustomPanel3.BackColor2 = System.Drawing.SystemColors.Highlight;
			this.CustomPanel3.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
			this.CustomPanel3.Curvature = 50;
			this.CustomPanel3.GradientMode = CSharpCustomPanelControl.LinearGradientMode.Horizontal;
			this.CustomPanel3.Location = new System.Drawing.Point(248, 16);
			this.CustomPanel3.Name = "CustomPanel3";
			this.CustomPanel3.Size = new System.Drawing.Size(100, 100);
			this.CustomPanel3.TabIndex = 2;
			this.CustomPanel3.MouseEnter += new System.EventHandler(this.CustomPanel3_MouseEnter);
			this.CustomPanel3.MouseLeave += new System.EventHandler(this.CustomPanel3_MouseEnter);
			// 
			// CustomPanel4
			// 
			this.CustomPanel4.BackColor = System.Drawing.SystemColors.InactiveCaptionText;
			this.CustomPanel4.BackColor2 = System.Drawing.SystemColors.Highlight;
			this.CustomPanel4.BorderColor = System.Drawing.SystemColors.HotTrack;
			this.CustomPanel4.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
			this.CustomPanel4.Curvature = 50;
			this.CustomPanel4.GradientMode = CSharpCustomPanelControl.LinearGradientMode.Vertical;
			this.CustomPanel4.Location = new System.Drawing.Point(246, 144);
			this.CustomPanel4.Name = "CustomPanel4";
			this.CustomPanel4.Size = new System.Drawing.Size(100, 27);
			this.CustomPanel4.TabIndex = 3;
			this.CustomPanel4.MouseEnter += new System.EventHandler(this.CustomPanel3_MouseEnter);
			this.CustomPanel4.MouseLeave += new System.EventHandler(this.CustomPanel3_MouseEnter);
			// 
			// CustomPanel5
			// 
			this.CustomPanel5.BackColor = System.Drawing.Color.LightSalmon;
			this.CustomPanel5.BackColor2 = System.Drawing.Color.DarkSalmon;
			this.CustomPanel5.BorderColor = System.Drawing.Color.DarkOrange;
			this.CustomPanel5.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
			this.CustomPanel5.BorderWidth = 10;
			this.CustomPanel5.Curvature = 20;
			this.CustomPanel5.GradientMode = CSharpCustomPanelControl.LinearGradientMode.BackwardDiagonal;
			this.CustomPanel5.Location = new System.Drawing.Point(248, 192);
			this.CustomPanel5.Name = "CustomPanel5";
			this.CustomPanel5.Size = new System.Drawing.Size(100, 100);
			this.CustomPanel5.TabIndex = 4;
			// 
			// CustomPanel6
			// 
			this.CustomPanel6.BackColor = System.Drawing.SystemColors.ControlLightLight;
			this.CustomPanel6.BackColor2 = System.Drawing.SystemColors.ControlDarkDark;
			this.CustomPanel6.BorderColor = System.Drawing.SystemColors.HotTrack;
			this.CustomPanel6.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
			this.CustomPanel6.Curvature = 2;
			this.CustomPanel6.GradientMode = CSharpCustomPanelControl.LinearGradientMode.Vertical;
			this.CustomPanel6.Location = new System.Drawing.Point(248, 312);
			this.CustomPanel6.Name = "CustomPanel6";
			this.CustomPanel6.Size = new System.Drawing.Size(100, 27);
			this.CustomPanel6.TabIndex = 5;
			this.CustomPanel6.MouseEnter += new System.EventHandler(this.CustomPanel3_MouseEnter);
			this.CustomPanel6.MouseLeave += new System.EventHandler(this.CustomPanel3_MouseEnter);
			// 
			// Form1
			// 
			this.AutoScaleBaseSize = new System.Drawing.Size(5, 13);
			this.ClientSize = new System.Drawing.Size(368, 366);
			this.Controls.Add(this.CustomPanel6);
			this.Controls.Add(this.CustomPanel5);
			this.Controls.Add(this.CustomPanel4);
			this.Controls.Add(this.CustomPanel3);
			this.Controls.Add(this.CustomPanel1);
			this.Name = "Form1";
			this.Text = "Custom Panel Demo";
			this.CustomPanel1.ResumeLayout(false);
			this.ResumeLayout(false);

		}
		#endregion

		private void CustomPanel3_MouseEnter(object sender, System.EventArgs e)
		{
			CustomPanel control = ((CustomPanel)sender);
			System.Drawing.Color tempColor = control.BackColor;
			control.BackColor = control.BackColor2;
			control.BackColor2 = tempColor;
			control.Invalidate();
		}

		public static void Main()
		{
			System.Windows.Forms.Application.Run(new Form1());
		}

	}
}