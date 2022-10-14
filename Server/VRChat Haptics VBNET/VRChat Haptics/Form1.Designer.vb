<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()>
Partial Class Form1
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
        Dim resources As System.ComponentModel.ComponentResourceManager = New System.ComponentModel.ComponentResourceManager(GetType(Form1))
        Me.MainTimer = New System.Windows.Forms.Timer(Me.components)
        Me.DGVNodes = New System.Windows.Forms.DataGridView()
        Me.Output = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.NodeName = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.Force = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.Test = New System.Windows.Forms.DataGridViewButtonColumn()
        Me.Node = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.Motor = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.GroupBox1 = New System.Windows.Forms.GroupBox()
        Me.GroupBox2 = New System.Windows.Forms.GroupBox()
        Me.DGVDevice = New System.Windows.Forms.DataGridView()
        Me.DataGridViewTextBoxColumn1 = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.DataGridViewTextBoxColumn2 = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.MenuStrip1 = New System.Windows.Forms.MenuStrip()
        Me.FileToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.LoadDeviceNodeDescriptionToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.SaveDeviceNodeDescriptionToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.CloseToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.SelectOSCFolderToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.OctoSlimeModeToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.RestartTimerToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.DeviceToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.AddDeviceToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.EditDeviceToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.RemoveDeviceToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.SetWifiOnDeviceToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.OFDSettings = New System.Windows.Forms.OpenFileDialog()
        Me.SFDSettings = New System.Windows.Forms.SaveFileDialog()
        Me.RichTextBox1 = New System.Windows.Forms.RichTextBox()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.FolderBrowserDialog1 = New System.Windows.Forms.FolderBrowserDialog()
        CType(Me.DGVNodes, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.GroupBox1.SuspendLayout()
        Me.GroupBox2.SuspendLayout()
        CType(Me.DGVDevice, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.MenuStrip1.SuspendLayout()
        Me.SuspendLayout()
        '
        'MainTimer
        '
        Me.MainTimer.Interval = 20
        '
        'DGVNodes
        '
        Me.DGVNodes.AllowUserToAddRows = False
        Me.DGVNodes.AllowUserToDeleteRows = False
        Me.DGVNodes.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize
        Me.DGVNodes.Columns.AddRange(New System.Windows.Forms.DataGridViewColumn() {Me.Output, Me.NodeName, Me.Force, Me.Test, Me.Node, Me.Motor})
        Me.DGVNodes.Location = New System.Drawing.Point(8, 29)
        Me.DGVNodes.Margin = New System.Windows.Forms.Padding(4, 5, 4, 5)
        Me.DGVNodes.Name = "DGVNodes"
        Me.DGVNodes.RowHeadersVisible = False
        Me.DGVNodes.RowHeadersWidth = 62
        Me.DGVNodes.Size = New System.Drawing.Size(507, 411)
        Me.DGVNodes.TabIndex = 51
        '
        'Output
        '
        Me.Output.HeaderText = "#"
        Me.Output.MinimumWidth = 8
        Me.Output.Name = "Output"
        Me.Output.ReadOnly = True
        Me.Output.Width = 20
        '
        'NodeName
        '
        Me.NodeName.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.Fill
        Me.NodeName.HeaderText = "Node Name"
        Me.NodeName.MinimumWidth = 8
        Me.NodeName.Name = "NodeName"
        '
        'Force
        '
        Me.Force.HeaderText = "Force"
        Me.Force.MinimumWidth = 8
        Me.Force.Name = "Force"
        Me.Force.Width = 35
        '
        'Test
        '
        Me.Test.HeaderText = "Test"
        Me.Test.MinimumWidth = 8
        Me.Test.Name = "Test"
        Me.Test.Text = "Test"
        Me.Test.Width = 35
        '
        'Node
        '
        Me.Node.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.None
        Me.Node.HeaderText = "Node#"
        Me.Node.MinimumWidth = 8
        Me.Node.Name = "Node"
        Me.Node.Visible = False
        Me.Node.Width = 50
        '
        'Motor
        '
        Me.Motor.HeaderText = "Motor#"
        Me.Motor.MinimumWidth = 8
        Me.Motor.Name = "Motor"
        Me.Motor.Visible = False
        Me.Motor.Width = 50
        '
        'GroupBox1
        '
        Me.GroupBox1.Controls.Add(Me.DGVNodes)
        Me.GroupBox1.Location = New System.Drawing.Point(3, 257)
        Me.GroupBox1.Margin = New System.Windows.Forms.Padding(4, 5, 4, 5)
        Me.GroupBox1.Name = "GroupBox1"
        Me.GroupBox1.Padding = New System.Windows.Forms.Padding(4, 5, 4, 5)
        Me.GroupBox1.Size = New System.Drawing.Size(524, 449)
        Me.GroupBox1.TabIndex = 54
        Me.GroupBox1.TabStop = False
        Me.GroupBox1.Text = "Nodes"
        '
        'GroupBox2
        '
        Me.GroupBox2.Controls.Add(Me.DGVDevice)
        Me.GroupBox2.Location = New System.Drawing.Point(3, 42)
        Me.GroupBox2.Margin = New System.Windows.Forms.Padding(4, 5, 4, 5)
        Me.GroupBox2.Name = "GroupBox2"
        Me.GroupBox2.Padding = New System.Windows.Forms.Padding(4, 5, 4, 5)
        Me.GroupBox2.Size = New System.Drawing.Size(524, 206)
        Me.GroupBox2.TabIndex = 55
        Me.GroupBox2.TabStop = False
        Me.GroupBox2.Text = "Device"
        '
        'DGVDevice
        '
        Me.DGVDevice.AllowUserToAddRows = False
        Me.DGVDevice.AllowUserToDeleteRows = False
        Me.DGVDevice.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize
        Me.DGVDevice.Columns.AddRange(New System.Windows.Forms.DataGridViewColumn() {Me.DataGridViewTextBoxColumn1, Me.DataGridViewTextBoxColumn2})
        Me.DGVDevice.Location = New System.Drawing.Point(8, 29)
        Me.DGVDevice.Margin = New System.Windows.Forms.Padding(4, 5, 4, 5)
        Me.DGVDevice.Name = "DGVDevice"
        Me.DGVDevice.ReadOnly = True
        Me.DGVDevice.RowHeadersVisible = False
        Me.DGVDevice.RowHeadersWidth = 62
        Me.DGVDevice.Size = New System.Drawing.Size(507, 168)
        Me.DGVDevice.TabIndex = 52
        '
        'DataGridViewTextBoxColumn1
        '
        Me.DataGridViewTextBoxColumn1.HeaderText = "Index"
        Me.DataGridViewTextBoxColumn1.MinimumWidth = 8
        Me.DataGridViewTextBoxColumn1.Name = "DataGridViewTextBoxColumn1"
        Me.DataGridViewTextBoxColumn1.ReadOnly = True
        Me.DataGridViewTextBoxColumn1.Width = 50
        '
        'DataGridViewTextBoxColumn2
        '
        Me.DataGridViewTextBoxColumn2.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.Fill
        Me.DataGridViewTextBoxColumn2.HeaderText = "Name"
        Me.DataGridViewTextBoxColumn2.MinimumWidth = 8
        Me.DataGridViewTextBoxColumn2.Name = "DataGridViewTextBoxColumn2"
        Me.DataGridViewTextBoxColumn2.ReadOnly = True
        '
        'MenuStrip1
        '
        Me.MenuStrip1.GripMargin = New System.Windows.Forms.Padding(2, 2, 0, 2)
        Me.MenuStrip1.ImageScalingSize = New System.Drawing.Size(24, 24)
        Me.MenuStrip1.Items.AddRange(New System.Windows.Forms.ToolStripItem() {Me.FileToolStripMenuItem, Me.DeviceToolStripMenuItem})
        Me.MenuStrip1.Location = New System.Drawing.Point(0, 0)
        Me.MenuStrip1.Name = "MenuStrip1"
        Me.MenuStrip1.Size = New System.Drawing.Size(807, 35)
        Me.MenuStrip1.TabIndex = 56
        Me.MenuStrip1.Text = "MenuStrip1"
        '
        'FileToolStripMenuItem
        '
        Me.FileToolStripMenuItem.DropDownItems.AddRange(New System.Windows.Forms.ToolStripItem() {Me.LoadDeviceNodeDescriptionToolStripMenuItem, Me.SaveDeviceNodeDescriptionToolStripMenuItem, Me.CloseToolStripMenuItem, Me.SelectOSCFolderToolStripMenuItem, Me.OctoSlimeModeToolStripMenuItem, Me.RestartTimerToolStripMenuItem})
        Me.FileToolStripMenuItem.Name = "FileToolStripMenuItem"
        Me.FileToolStripMenuItem.Size = New System.Drawing.Size(54, 29)
        Me.FileToolStripMenuItem.Text = "File"
        '
        'LoadDeviceNodeDescriptionToolStripMenuItem
        '
        Me.LoadDeviceNodeDescriptionToolStripMenuItem.Name = "LoadDeviceNodeDescriptionToolStripMenuItem"
        Me.LoadDeviceNodeDescriptionToolStripMenuItem.Size = New System.Drawing.Size(255, 34)
        Me.LoadDeviceNodeDescriptionToolStripMenuItem.Text = "Load Settings"
        '
        'SaveDeviceNodeDescriptionToolStripMenuItem
        '
        Me.SaveDeviceNodeDescriptionToolStripMenuItem.Name = "SaveDeviceNodeDescriptionToolStripMenuItem"
        Me.SaveDeviceNodeDescriptionToolStripMenuItem.Size = New System.Drawing.Size(255, 34)
        Me.SaveDeviceNodeDescriptionToolStripMenuItem.Text = "Save Settings"
        '
        'CloseToolStripMenuItem
        '
        Me.CloseToolStripMenuItem.Name = "CloseToolStripMenuItem"
        Me.CloseToolStripMenuItem.Size = New System.Drawing.Size(255, 34)
        Me.CloseToolStripMenuItem.Text = "Close"
        '
        'SelectOSCFolderToolStripMenuItem
        '
        Me.SelectOSCFolderToolStripMenuItem.Name = "SelectOSCFolderToolStripMenuItem"
        Me.SelectOSCFolderToolStripMenuItem.Size = New System.Drawing.Size(255, 34)
        Me.SelectOSCFolderToolStripMenuItem.Text = "Select OSC Folder"
        '
        'OctoSlimeModeToolStripMenuItem
        '
        Me.OctoSlimeModeToolStripMenuItem.Name = "OctoSlimeModeToolStripMenuItem"
        Me.OctoSlimeModeToolStripMenuItem.Size = New System.Drawing.Size(255, 34)
        Me.OctoSlimeModeToolStripMenuItem.Text = "OctoSlime Mode"
        '
        'RestartTimerToolStripMenuItem
        '
        Me.RestartTimerToolStripMenuItem.Name = "RestartTimerToolStripMenuItem"
        Me.RestartTimerToolStripMenuItem.Size = New System.Drawing.Size(255, 34)
        Me.RestartTimerToolStripMenuItem.Text = "Restart Timer"
        '
        'DeviceToolStripMenuItem
        '
        Me.DeviceToolStripMenuItem.DropDownItems.AddRange(New System.Windows.Forms.ToolStripItem() {Me.AddDeviceToolStripMenuItem, Me.EditDeviceToolStripMenuItem, Me.RemoveDeviceToolStripMenuItem, Me.SetWifiOnDeviceToolStripMenuItem})
        Me.DeviceToolStripMenuItem.Name = "DeviceToolStripMenuItem"
        Me.DeviceToolStripMenuItem.Size = New System.Drawing.Size(80, 29)
        Me.DeviceToolStripMenuItem.Text = "Device"
        '
        'AddDeviceToolStripMenuItem
        '
        Me.AddDeviceToolStripMenuItem.Name = "AddDeviceToolStripMenuItem"
        Me.AddDeviceToolStripMenuItem.Size = New System.Drawing.Size(258, 34)
        Me.AddDeviceToolStripMenuItem.Text = "Add Device"
        '
        'EditDeviceToolStripMenuItem
        '
        Me.EditDeviceToolStripMenuItem.Name = "EditDeviceToolStripMenuItem"
        Me.EditDeviceToolStripMenuItem.Size = New System.Drawing.Size(258, 34)
        Me.EditDeviceToolStripMenuItem.Text = "Edit Device"
        '
        'RemoveDeviceToolStripMenuItem
        '
        Me.RemoveDeviceToolStripMenuItem.Name = "RemoveDeviceToolStripMenuItem"
        Me.RemoveDeviceToolStripMenuItem.Size = New System.Drawing.Size(258, 34)
        Me.RemoveDeviceToolStripMenuItem.Text = "Remove Device"
        '
        'SetWifiOnDeviceToolStripMenuItem
        '
        Me.SetWifiOnDeviceToolStripMenuItem.Name = "SetWifiOnDeviceToolStripMenuItem"
        Me.SetWifiOnDeviceToolStripMenuItem.Size = New System.Drawing.Size(258, 34)
        Me.SetWifiOnDeviceToolStripMenuItem.Text = "Set Wifi on Device"
        '
        'OFDSettings
        '
        Me.OFDSettings.Filter = "VRHaptics File|*.vrh"
        '
        'SFDSettings
        '
        Me.SFDSettings.Filter = "VRHaptics File|*.vrh"
        '
        'RichTextBox1
        '
        Me.RichTextBox1.Location = New System.Drawing.Point(536, 66)
        Me.RichTextBox1.Margin = New System.Windows.Forms.Padding(4, 5, 4, 5)
        Me.RichTextBox1.Name = "RichTextBox1"
        Me.RichTextBox1.Size = New System.Drawing.Size(260, 638)
        Me.RichTextBox1.TabIndex = 57
        Me.RichTextBox1.Text = "0" & Global.Microsoft.VisualBasic.ChrW(10) & "1" & Global.Microsoft.VisualBasic.ChrW(10) & "2" & Global.Microsoft.VisualBasic.ChrW(10) & "3" & Global.Microsoft.VisualBasic.ChrW(10) & "4" & Global.Microsoft.VisualBasic.ChrW(10) & "5" & Global.Microsoft.VisualBasic.ChrW(10) & "6" & Global.Microsoft.VisualBasic.ChrW(10) & "7" & Global.Microsoft.VisualBasic.ChrW(10) & "8" & Global.Microsoft.VisualBasic.ChrW(10) & "9" & Global.Microsoft.VisualBasic.ChrW(10) & "10" & Global.Microsoft.VisualBasic.ChrW(10) & "11" & Global.Microsoft.VisualBasic.ChrW(10) & "12" & Global.Microsoft.VisualBasic.ChrW(10) & "13" & Global.Microsoft.VisualBasic.ChrW(10) & "14" & Global.Microsoft.VisualBasic.ChrW(10) & "15" & Global.Microsoft.VisualBasic.ChrW(10) & "16" & Global.Microsoft.VisualBasic.ChrW(10) & "17" & Global.Microsoft.VisualBasic.ChrW(10) & "19" & Global.Microsoft.VisualBasic.ChrW(10) & "20" & Global.Microsoft.VisualBasic.ChrW(10) & "21" & Global.Microsoft.VisualBasic.ChrW(10) & "22" & Global.Microsoft.VisualBasic.ChrW(10) & "23" & Global.Microsoft.VisualBasic.ChrW(10) & "24" & Global.Microsoft.VisualBasic.ChrW(10) & "25" & Global.Microsoft.VisualBasic.ChrW(10) & "26" & Global.Microsoft.VisualBasic.ChrW(10) & "27" & Global.Microsoft.VisualBasic.ChrW(10) & "28" & Global.Microsoft.VisualBasic.ChrW(10) & "29" & Global.Microsoft.VisualBasic.ChrW(10) & "30" & Global.Microsoft.VisualBasic.ChrW(10) & "3" &
    "1"
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Location = New System.Drawing.Point(536, 42)
        Me.Label1.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(96, 20)
        Me.Label1.TabIndex = 58
        Me.Label1.Text = "OSC History"
        '
        'FolderBrowserDialog1
        '
        '
        'Form1
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(9.0!, 20.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(807, 712)
        Me.Controls.Add(Me.Label1)
        Me.Controls.Add(Me.RichTextBox1)
        Me.Controls.Add(Me.GroupBox2)
        Me.Controls.Add(Me.GroupBox1)
        Me.Controls.Add(Me.MenuStrip1)
        Me.FormBorderStyle = System.Windows.Forms.FormBorderStyle.Fixed3D
        Me.Icon = CType(resources.GetObject("$this.Icon"), System.Drawing.Icon)
        Me.MainMenuStrip = Me.MenuStrip1
        Me.Margin = New System.Windows.Forms.Padding(4, 5, 4, 5)
        Me.Name = "Form1"
        Me.Text = "VRChat Haptics"
        CType(Me.DGVNodes, System.ComponentModel.ISupportInitialize).EndInit()
        Me.GroupBox1.ResumeLayout(False)
        Me.GroupBox2.ResumeLayout(False)
        CType(Me.DGVDevice, System.ComponentModel.ISupportInitialize).EndInit()
        Me.MenuStrip1.ResumeLayout(False)
        Me.MenuStrip1.PerformLayout()
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents MainTimer As Timer
    Friend WithEvents DGVNodes As DataGridView
    Friend WithEvents GroupBox1 As GroupBox
    Friend WithEvents GroupBox2 As GroupBox
    Friend WithEvents DGVDevice As DataGridView
    Friend WithEvents DataGridViewTextBoxColumn1 As DataGridViewTextBoxColumn
    Friend WithEvents DataGridViewTextBoxColumn2 As DataGridViewTextBoxColumn
    Friend WithEvents MenuStrip1 As MenuStrip
    Friend WithEvents FileToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents LoadDeviceNodeDescriptionToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents SaveDeviceNodeDescriptionToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents DeviceToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents AddDeviceToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents EditDeviceToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents RemoveDeviceToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents OFDSettings As OpenFileDialog
    Friend WithEvents SFDSettings As SaveFileDialog
    Friend WithEvents CloseToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents SetWifiOnDeviceToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents RichTextBox1 As RichTextBox
    Friend WithEvents Label1 As Label
    Friend WithEvents SelectOSCFolderToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents FolderBrowserDialog1 As FolderBrowserDialog
    Friend WithEvents OctoSlimeModeToolStripMenuItem As ToolStripMenuItem
    Friend WithEvents Output As DataGridViewTextBoxColumn
    Friend WithEvents NodeName As DataGridViewTextBoxColumn
    Friend WithEvents Force As DataGridViewTextBoxColumn
    Friend WithEvents Test As DataGridViewButtonColumn
    Friend WithEvents Node As DataGridViewTextBoxColumn
    Friend WithEvents Motor As DataGridViewTextBoxColumn
    Friend WithEvents RestartTimerToolStripMenuItem As ToolStripMenuItem
End Class
