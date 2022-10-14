Public Class Form1
    Inherits System.Windows.Forms.Form

#Region " Windows Form Designer generated code "

    Public Sub New()
        MyBase.New()

        'This call is required by the Windows Form Designer.
        InitializeComponent()

        'Add any initialization after the InitializeComponent() call

    End Sub

    'Form overrides dispose to clean up the component list.
    Protected Overloads Overrides Sub Dispose(ByVal disposing As Boolean)
        If disposing Then
            If Not (components Is Nothing) Then
                components.Dispose()
            End If
        End If
        MyBase.Dispose(disposing)
    End Sub
    Friend CustomPanel1 As CustomPanel

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    Friend WithEvents CustomPanel2 As CustomPanelControl.CustomPanel
    Friend WithEvents CustomPanel3 As CustomPanelControl.CustomPanel
    Friend WithEvents CustomPanel4 As CustomPanelControl.CustomPanel
    Friend WithEvents CustomPanel5 As CustomPanelControl.CustomPanel
    Friend WithEvents CustomPanel6 As CustomPanelControl.CustomPanel
    <System.Diagnostics.DebuggerStepThrough()> Private Sub InitializeComponent()
        Me.CustomPanel1 = New CustomPanelControl.CustomPanel
        Me.CustomPanel2 = New CustomPanelControl.CustomPanel
        Me.CustomPanel3 = New CustomPanelControl.CustomPanel
        Me.CustomPanel4 = New CustomPanelControl.CustomPanel
        Me.CustomPanel5 = New CustomPanelControl.CustomPanel
        Me.CustomPanel6 = New CustomPanelControl.CustomPanel
        Me.CustomPanel1.SuspendLayout()
        Me.SuspendLayout()
        '
        'CustomPanel1
        '
        Me.CustomPanel1.BackColor = System.Drawing.SystemColors.Highlight
        Me.CustomPanel1.Controls.Add(Me.CustomPanel2)
        Me.CustomPanel1.GradientMode = CustomPanelControl.LinearGradientMode.BackwardDiagonal
        Me.CustomPanel1.Location = New System.Drawing.Point(24, 16)
        Me.CustomPanel1.Name = "CustomPanel1"
        Me.CustomPanel1.BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D
        Me.CustomPanel1.Size = New System.Drawing.Size(200, 320)
        Me.CustomPanel1.TabIndex = 0
        '
        'CustomPanel2
        '
        Me.CustomPanel2.BackColor = System.Drawing.SystemColors.InactiveCaptionText
        Me.CustomPanel2.BorderColor = System.Drawing.SystemColors.ControlLightLight
        Me.CustomPanel2.Curvature = 20
        Me.CustomPanel2.Location = New System.Drawing.Point(32, 80)
        Me.CustomPanel2.Name = "CustomPanel2"
        Me.CustomPanel2.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel2.Size = New System.Drawing.Size(136, 160)
        Me.CustomPanel2.TabIndex = 1
        '
        'CustomPanel3
        '
        Me.CustomPanel3.BackColor = System.Drawing.SystemColors.InactiveCaptionText
        Me.CustomPanel3.BackColor2 = System.Drawing.SystemColors.Highlight
        Me.CustomPanel3.Curvature = 50
        Me.CustomPanel3.GradientMode = CustomPanelControl.LinearGradientMode.Horizontal
        Me.CustomPanel3.Location = New System.Drawing.Point(248, 16)
        Me.CustomPanel3.Name = "CustomPanel3"
        Me.CustomPanel3.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel3.Size = New System.Drawing.Size(100, 100)
        Me.CustomPanel3.TabIndex = 2
        '
        'CustomPanel4
        '
        Me.CustomPanel4.BackColor = System.Drawing.SystemColors.InactiveCaptionText
        Me.CustomPanel4.BackColor2 = System.Drawing.SystemColors.Highlight
        Me.CustomPanel4.BorderColor = System.Drawing.SystemColors.HotTrack
        Me.CustomPanel4.Curvature = 50
        Me.CustomPanel4.GradientMode = CustomPanelControl.LinearGradientMode.Vertical
        Me.CustomPanel4.Location = New System.Drawing.Point(246, 144)
        Me.CustomPanel4.Name = "CustomPanel4"
        Me.CustomPanel4.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel4.Size = New System.Drawing.Size(100, 27)
        Me.CustomPanel4.TabIndex = 3
        '
        'CustomPanel5
        '
        Me.CustomPanel5.BackColor = System.Drawing.Color.LightSalmon
        Me.CustomPanel5.BackColor2 = System.Drawing.Color.DarkSalmon
        Me.CustomPanel5.BorderColor = System.Drawing.Color.DarkOrange
        Me.CustomPanel5.BorderWidth = 10
        Me.CustomPanel5.Curvature = 20
        Me.CustomPanel5.GradientMode = CustomPanelControl.LinearGradientMode.BackwardDiagonal
        Me.CustomPanel5.Location = New System.Drawing.Point(248, 192)
        Me.CustomPanel5.Name = "CustomPanel5"
        Me.CustomPanel5.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel5.Size = New System.Drawing.Size(100, 100)
        Me.CustomPanel5.TabIndex = 4
        '
        'CustomPanel6
        '
        Me.CustomPanel6.BackColor = System.Drawing.SystemColors.ControlLightLight
        Me.CustomPanel6.BackColor2 = System.Drawing.SystemColors.ControlDarkDark
        Me.CustomPanel6.BorderColor = System.Drawing.SystemColors.HotTrack
        Me.CustomPanel6.Curvature = 2
        Me.CustomPanel6.GradientMode = CustomPanelControl.LinearGradientMode.Vertical
        Me.CustomPanel6.Location = New System.Drawing.Point(248, 312)
        Me.CustomPanel6.Name = "CustomPanel6"
        Me.CustomPanel6.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel6.Size = New System.Drawing.Size(100, 27)
        Me.CustomPanel6.TabIndex = 5
        '
        'Form1
        '
        Me.AutoScaleBaseSize = New System.Drawing.Size(5, 13)
        Me.ClientSize = New System.Drawing.Size(368, 366)
        Me.Controls.Add(Me.CustomPanel6)
        Me.Controls.Add(Me.CustomPanel5)
        Me.Controls.Add(Me.CustomPanel4)
        Me.Controls.Add(Me.CustomPanel3)
        Me.Controls.Add(Me.CustomPanel1)
        Me.Name = "Form1"
        Me.Text = "Custom Panel Demo"
        Me.CustomPanel1.ResumeLayout(False)
        Me.ResumeLayout(False)

    End Sub

#End Region

    Private Sub CustomPanel3_MouseEnter(ByVal sender As Object, ByVal e As System.EventArgs) Handles CustomPanel3.MouseEnter, CustomPanel3.MouseLeave, CustomPanel4.MouseEnter, CustomPanel4.MouseLeave, CustomPanel6.MouseEnter, CustomPanel6.MouseLeave
        Dim control As CustomPanel = CType(sender, CustomPanel)
        Dim tempColor As System.Drawing.Color = control.BackColor
        control.BackColor = control.BackColor2
        control.BackColor2 = tempColor
        control.Invalidate()
    End Sub

End Class
