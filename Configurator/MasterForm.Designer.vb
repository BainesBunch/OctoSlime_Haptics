<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()>
Partial Class frmFloatConfig
    Inherits System.Windows.Forms.Form

    'Form overrides dispose to clean up the component list.
    <System.Diagnostics.DebuggerNonUserCode()>
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Required by the Windows Form Designer
    Private components As System.ComponentModel.IContainer

    'NOTE: The following procedure is required by the Windows Form Designer
    'It can be modified using the Windows Form Designer.  
    'Do not modify it using the code editor.
    <System.Diagnostics.DebuggerStepThrough()>
    Private Sub InitializeComponent()
        Me.components = New System.ComponentModel.Container()
        Me.FloatInterface = New System.IO.Ports.SerialPort(Me.components)
        Me.StatusStrip1 = New System.Windows.Forms.StatusStrip()
        Me.lblStatus = New System.Windows.Forms.ToolStripStatusLabel()
        Me.SelectFirmwareFile = New System.Windows.Forms.OpenFileDialog()
        Me.CustomPanel4 = New FloatConfigurator.CustomPanel()
        Me.CustomPanel1 = New FloatConfigurator.CustomPanel()
        Me.cmdRefresh = New System.Windows.Forms.Button()
        Me.ComPortSelect = New System.Windows.Forms.ComboBox()
        Me.Label10 = New System.Windows.Forms.Label()
        Me.Label9 = New System.Windows.Forms.Label()
        Me.CustomPanel3 = New FloatConfigurator.CustomPanel()
        Me.cmdFirmwareUpdate = New System.Windows.Forms.Button()
        Me.cmdRead = New System.Windows.Forms.Button()
        Me.cmdWrite = New System.Windows.Forms.Button()
        Me.cmdRestartVTS = New System.Windows.Forms.Button()
        Me.CustomPanel2 = New FloatConfigurator.CustomPanel()
        Me.chkHapticsActive = New System.Windows.Forms.CheckBox()
        Me.txtSSID = New System.Windows.Forms.TextBox()
        Me.txtPassword = New System.Windows.Forms.TextBox()
        Me.Label3 = New System.Windows.Forms.Label()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.Label8 = New System.Windows.Forms.Label()
        Me.Label4 = New System.Windows.Forms.Label()
        Me.StatusStrip1.SuspendLayout()
        Me.CustomPanel4.SuspendLayout()
        Me.CustomPanel1.SuspendLayout()
        Me.CustomPanel3.SuspendLayout()
        Me.CustomPanel2.SuspendLayout()
        Me.SuspendLayout()
        '
        'FloatInterface
        '
        Me.FloatInterface.BaudRate = 115200
        '
        'StatusStrip1
        '
        Me.StatusStrip1.GripMargin = New System.Windows.Forms.Padding(0)
        Me.StatusStrip1.ImageScalingSize = New System.Drawing.Size(24, 24)
        Me.StatusStrip1.Items.AddRange(New System.Windows.Forms.ToolStripItem() {Me.lblStatus})
        Me.StatusStrip1.Location = New System.Drawing.Point(0, 427)
        Me.StatusStrip1.Name = "StatusStrip1"
        Me.StatusStrip1.Size = New System.Drawing.Size(510, 22)
        Me.StatusStrip1.SizingGrip = False
        Me.StatusStrip1.TabIndex = 21
        Me.StatusStrip1.Text = "StatusStrip1"
        '
        'lblStatus
        '
        Me.lblStatus.BackColor = System.Drawing.Color.Transparent
        Me.lblStatus.Name = "lblStatus"
        Me.lblStatus.Size = New System.Drawing.Size(26, 17)
        Me.lblStatus.Text = "Idle"
        '
        'SelectFirmwareFile
        '
        Me.SelectFirmwareFile.DefaultExt = "bin"
        Me.SelectFirmwareFile.FileName = "*.Bin"
        '
        'CustomPanel4
        '
        Me.CustomPanel4.BackColor = System.Drawing.Color.Teal
        Me.CustomPanel4.BackColor2 = System.Drawing.Color.DarkSlateGray
        Me.CustomPanel4.Controls.Add(Me.CustomPanel1)
        Me.CustomPanel4.Controls.Add(Me.CustomPanel3)
        Me.CustomPanel4.Controls.Add(Me.CustomPanel2)
        Me.CustomPanel4.Dock = System.Windows.Forms.DockStyle.Fill
        Me.CustomPanel4.GradientMode = FloatConfigurator.LinearGradientMode.Vertical
        Me.CustomPanel4.Location = New System.Drawing.Point(0, 0)
        Me.CustomPanel4.Name = "CustomPanel4"
        Me.CustomPanel4.Size = New System.Drawing.Size(510, 427)
        Me.CustomPanel4.TabIndex = 31
        '
        'CustomPanel1
        '
        Me.CustomPanel1.BackColor = System.Drawing.Color.Teal
        Me.CustomPanel1.BackColor2 = System.Drawing.Color.DarkSlateGray
        Me.CustomPanel1.BorderColor = System.Drawing.SystemColors.HotTrack
        Me.CustomPanel1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel1.BorderWidth = 7
        Me.CustomPanel1.Controls.Add(Me.cmdRefresh)
        Me.CustomPanel1.Controls.Add(Me.ComPortSelect)
        Me.CustomPanel1.Controls.Add(Me.Label10)
        Me.CustomPanel1.Controls.Add(Me.Label9)
        Me.CustomPanel1.Curvature = 30
        Me.CustomPanel1.Location = New System.Drawing.Point(13, 16)
        Me.CustomPanel1.Name = "CustomPanel1"
        Me.CustomPanel1.Size = New System.Drawing.Size(483, 110)
        Me.CustomPanel1.TabIndex = 28
        '
        'cmdRefresh
        '
        Me.cmdRefresh.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdRefresh.Location = New System.Drawing.Point(384, 45)
        Me.cmdRefresh.Margin = New System.Windows.Forms.Padding(4)
        Me.cmdRefresh.Name = "cmdRefresh"
        Me.cmdRefresh.Size = New System.Drawing.Size(76, 23)
        Me.cmdRefresh.TabIndex = 21
        Me.cmdRefresh.Text = "Refresh"
        Me.cmdRefresh.UseVisualStyleBackColor = True
        '
        'ComPortSelect
        '
        Me.ComPortSelect.FormattingEnabled = True
        Me.ComPortSelect.Location = New System.Drawing.Point(103, 44)
        Me.ComPortSelect.Name = "ComPortSelect"
        Me.ComPortSelect.Size = New System.Drawing.Size(265, 24)
        Me.ComPortSelect.TabIndex = 19
        '
        'Label10
        '
        Me.Label10.AutoSize = True
        Me.Label10.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label10.Location = New System.Drawing.Point(27, 20)
        Me.Label10.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label10.Name = "Label10"
        Me.Label10.Size = New System.Drawing.Size(85, 16)
        Me.Label10.TabIndex = 23
        Me.Label10.Text = "Connection"
        '
        'Label9
        '
        Me.Label9.AutoSize = True
        Me.Label9.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label9.Location = New System.Drawing.Point(27, 47)
        Me.Label9.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label9.Name = "Label9"
        Me.Label9.Size = New System.Drawing.Size(69, 16)
        Me.Label9.TabIndex = 20
        Me.Label9.Text = "Com Port :"
        '
        'CustomPanel3
        '
        Me.CustomPanel3.BackColor = System.Drawing.Color.Teal
        Me.CustomPanel3.BackColor2 = System.Drawing.Color.DarkSlateGray
        Me.CustomPanel3.BorderColor = System.Drawing.SystemColors.HotTrack
        Me.CustomPanel3.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel3.BorderWidth = 7
        Me.CustomPanel3.Controls.Add(Me.cmdFirmwareUpdate)
        Me.CustomPanel3.Controls.Add(Me.cmdRead)
        Me.CustomPanel3.Controls.Add(Me.cmdWrite)
        Me.CustomPanel3.Controls.Add(Me.cmdRestartVTS)
        Me.CustomPanel3.Curvature = 30
        Me.CustomPanel3.Location = New System.Drawing.Point(13, 338)
        Me.CustomPanel3.Name = "CustomPanel3"
        Me.CustomPanel3.Size = New System.Drawing.Size(483, 78)
        Me.CustomPanel3.TabIndex = 30
        '
        'cmdFirmwareUpdate
        '
        Me.cmdFirmwareUpdate.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdFirmwareUpdate.Location = New System.Drawing.Point(136, 28)
        Me.cmdFirmwareUpdate.Margin = New System.Windows.Forms.Padding(4)
        Me.cmdFirmwareUpdate.Name = "cmdFirmwareUpdate"
        Me.cmdFirmwareUpdate.Size = New System.Drawing.Size(145, 23)
        Me.cmdFirmwareUpdate.TabIndex = 25
        Me.cmdFirmwareUpdate.Text = "Firmware Update"
        Me.cmdFirmwareUpdate.UseVisualStyleBackColor = True
        '
        'cmdRead
        '
        Me.cmdRead.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdRead.Location = New System.Drawing.Point(300, 28)
        Me.cmdRead.Margin = New System.Windows.Forms.Padding(4)
        Me.cmdRead.Name = "cmdRead"
        Me.cmdRead.Size = New System.Drawing.Size(76, 23)
        Me.cmdRead.TabIndex = 6
        Me.cmdRead.Text = "Read"
        Me.cmdRead.UseVisualStyleBackColor = True
        '
        'cmdWrite
        '
        Me.cmdWrite.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdWrite.Location = New System.Drawing.Point(384, 28)
        Me.cmdWrite.Margin = New System.Windows.Forms.Padding(4)
        Me.cmdWrite.Name = "cmdWrite"
        Me.cmdWrite.Size = New System.Drawing.Size(76, 23)
        Me.cmdWrite.TabIndex = 7
        Me.cmdWrite.Text = "Write"
        Me.cmdWrite.UseVisualStyleBackColor = True
        '
        'cmdRestartVTS
        '
        Me.cmdRestartVTS.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdRestartVTS.Location = New System.Drawing.Point(24, 28)
        Me.cmdRestartVTS.Margin = New System.Windows.Forms.Padding(4)
        Me.cmdRestartVTS.Name = "cmdRestartVTS"
        Me.cmdRestartVTS.Size = New System.Drawing.Size(104, 23)
        Me.cmdRestartVTS.TabIndex = 24
        Me.cmdRestartVTS.Text = "Reboot"
        Me.cmdRestartVTS.UseVisualStyleBackColor = True
        '
        'CustomPanel2
        '
        Me.CustomPanel2.BackColor = System.Drawing.Color.Teal
        Me.CustomPanel2.BackColor2 = System.Drawing.Color.DarkSlateGray
        Me.CustomPanel2.BorderColor = System.Drawing.SystemColors.HotTrack
        Me.CustomPanel2.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel2.BorderWidth = 7
        Me.CustomPanel2.Controls.Add(Me.chkHapticsActive)
        Me.CustomPanel2.Controls.Add(Me.txtSSID)
        Me.CustomPanel2.Controls.Add(Me.txtPassword)
        Me.CustomPanel2.Controls.Add(Me.Label3)
        Me.CustomPanel2.Controls.Add(Me.Label2)
        Me.CustomPanel2.Controls.Add(Me.Label8)
        Me.CustomPanel2.Controls.Add(Me.Label4)
        Me.CustomPanel2.Curvature = 30
        Me.CustomPanel2.Location = New System.Drawing.Point(13, 132)
        Me.CustomPanel2.Name = "CustomPanel2"
        Me.CustomPanel2.Size = New System.Drawing.Size(483, 185)
        Me.CustomPanel2.TabIndex = 29
        '
        'chkHapticsActive
        '
        Me.chkHapticsActive.AutoSize = True
        Me.chkHapticsActive.Location = New System.Drawing.Point(136, 126)
        Me.chkHapticsActive.Name = "chkHapticsActive"
        Me.chkHapticsActive.Size = New System.Drawing.Size(15, 14)
        Me.chkHapticsActive.TabIndex = 26
        Me.chkHapticsActive.UseVisualStyleBackColor = True
        '
        'txtSSID
        '
        Me.txtSSID.Location = New System.Drawing.Point(136, 51)
        Me.txtSSID.Name = "txtSSID"
        Me.txtSSID.Size = New System.Drawing.Size(263, 22)
        Me.txtSSID.TabIndex = 24
        '
        'txtPassword
        '
        Me.txtPassword.Location = New System.Drawing.Point(136, 91)
        Me.txtPassword.Name = "txtPassword"
        Me.txtPassword.Size = New System.Drawing.Size(263, 22)
        Me.txtPassword.TabIndex = 25
        '
        'Label3
        '
        Me.Label3.AutoSize = True
        Me.Label3.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label3.Location = New System.Drawing.Point(21, 56)
        Me.Label3.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label3.Name = "Label3"
        Me.Label3.Size = New System.Drawing.Size(69, 16)
        Me.Label3.TabIndex = 4
        Me.Label3.Text = "WIFI SSID"
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label2.Location = New System.Drawing.Point(21, 91)
        Me.Label2.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(98, 16)
        Me.Label2.TabIndex = 21
        Me.Label2.Text = "WIFI Password"
        '
        'Label8
        '
        Me.Label8.AutoSize = True
        Me.Label8.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label8.Location = New System.Drawing.Point(21, 27)
        Me.Label8.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label8.Name = "Label8"
        Me.Label8.Size = New System.Drawing.Size(61, 16)
        Me.Label8.TabIndex = 18
        Me.Label8.Text = "Device "
        '
        'Label4
        '
        Me.Label4.AutoSize = True
        Me.Label4.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label4.Location = New System.Drawing.Point(21, 126)
        Me.Label4.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label4.Name = "Label4"
        Me.Label4.Size = New System.Drawing.Size(100, 16)
        Me.Label4.TabIndex = 23
        Me.Label4.Text = "Enable Haptics"
        '
        'frmFloatConfig
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(8.0!, 16.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.BackColor = System.Drawing.Color.Teal
        Me.ClientSize = New System.Drawing.Size(510, 449)
        Me.Controls.Add(Me.CustomPanel4)
        Me.Controls.Add(Me.StatusStrip1)
        Me.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.75!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog
        Me.Margin = New System.Windows.Forms.Padding(4)
        Me.MaximizeBox = False
        Me.MinimizeBox = False
        Me.Name = "frmFloatConfig"
        Me.ShowIcon = False
        Me.ShowInTaskbar = False
        Me.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen
        Me.Text = "OctoSlime  Configuration Tool"
        Me.StatusStrip1.ResumeLayout(False)
        Me.StatusStrip1.PerformLayout()
        Me.CustomPanel4.ResumeLayout(False)
        Me.CustomPanel1.ResumeLayout(False)
        Me.CustomPanel1.PerformLayout()
        Me.CustomPanel3.ResumeLayout(False)
        Me.CustomPanel2.ResumeLayout(False)
        Me.CustomPanel2.PerformLayout()
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents Label3 As Label
    Friend WithEvents cmdRead As Button
    Friend WithEvents cmdWrite As Button
    Friend WithEvents Label8 As Label
    Friend WithEvents ComPortSelect As ComboBox
    Friend WithEvents Label9 As Label
    Friend WithEvents FloatInterface As IO.Ports.SerialPort
    Friend WithEvents StatusStrip1 As StatusStrip
    Friend WithEvents lblStatus As ToolStripStatusLabel
    Friend WithEvents cmdRefresh As Button
    Friend WithEvents Label10 As Label
    Friend WithEvents cmdRestartVTS As Button
    Friend WithEvents Label4 As Label
    Friend WithEvents Label2 As Label
    Friend WithEvents cmdFirmwareUpdate As Button
    Friend WithEvents SelectFirmwareFile As OpenFileDialog
    Friend WithEvents chkHapticsActive As CheckBox
    Friend WithEvents txtPassword As TextBox
    Friend WithEvents txtSSID As TextBox
    Friend WithEvents CustomPanel1 As CustomPanel
    Friend WithEvents CustomPanel2 As CustomPanel
    Friend WithEvents CustomPanel3 As CustomPanel
    Friend WithEvents CustomPanel4 As CustomPanel
End Class
