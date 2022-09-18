<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()>
Partial Class OctoSlimeHapticsServer
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
        Dim resources As System.ComponentModel.ComponentResourceManager = New System.ComponentModel.ComponentResourceManager(GetType(OctoSlimeHapticsServer))
        Me.cmdStart = New System.Windows.Forms.Button()
        Me.HapticsControlerImages = New System.Windows.Forms.ImageList(Me.components)
        Me.cmdStop = New System.Windows.Forms.Button()
        Me.Octo_Haptics_Server_Settings_OFD = New System.Windows.Forms.OpenFileDialog()
        Me.Octo_Haptics_OSC_File_Location_Finder = New System.Windows.Forms.FolderBrowserDialog()
        Me.ActionsMenu = New System.Windows.Forms.MenuStrip()
        Me.Menu_Server = New System.Windows.Forms.ToolStripMenuItem()
        Me.Menu_Item_Start_Server = New System.Windows.Forms.ToolStripMenuItem()
        Me.Menu_Item_Stop_Server = New System.Windows.Forms.ToolStripMenuItem()
        Me.CloseServerToolStripMenuItem = New System.Windows.Forms.ToolStripMenuItem()
        Me.Configuration_Menu = New System.Windows.Forms.ToolStripMenuItem()
        Me.Menu_Item_Select_Osc_Folder = New System.Windows.Forms.ToolStripMenuItem()
        Me.CustomPanel3 = New OctoSlime_Haptics_Server.CustomPanel()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.txtIncommingPackets = New System.Windows.Forms.TextBox()
        Me.cmdClearHistory = New System.Windows.Forms.Button()
        Me.CustomPanel2 = New OctoSlime_Haptics_Server.CustomPanel()
        Me.lblDevice = New System.Windows.Forms.Label()
        Me.cmdSend = New System.Windows.Forms.Button()
        Me.Label6 = New System.Windows.Forms.Label()
        Me.pnlDigital = New System.Windows.Forms.Panel()
        Me.digitalValue = New System.Windows.Forms.ComboBox()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.pnlAnalogue = New System.Windows.Forms.Panel()
        Me.analogueValue = New System.Windows.Forms.NumericUpDown()
        Me.Label3 = New System.Windows.Forms.Label()
        Me.Label4 = New System.Windows.Forms.Label()
        Me.CustomPanel1 = New OctoSlime_Haptics_Server.CustomPanel()
        Me.Label5 = New System.Windows.Forms.Label()
        Me.ControlerTree = New System.Windows.Forms.TreeView()
        Me.ActionsMenu.SuspendLayout()
        Me.CustomPanel3.SuspendLayout()
        Me.CustomPanel2.SuspendLayout()
        Me.pnlDigital.SuspendLayout()
        Me.pnlAnalogue.SuspendLayout()
        CType(Me.analogueValue, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.CustomPanel1.SuspendLayout()
        Me.SuspendLayout()
        '
        'cmdStart
        '
        Me.cmdStart.BackColor = System.Drawing.Color.SlateGray
        Me.cmdStart.Cursor = System.Windows.Forms.Cursors.Default
        Me.cmdStart.FlatAppearance.BorderSize = 0
        Me.cmdStart.FlatStyle = System.Windows.Forms.FlatStyle.Flat
        Me.cmdStart.Font = New System.Drawing.Font("Microsoft Sans Serif", 16.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdStart.Location = New System.Drawing.Point(1052, 87)
        Me.cmdStart.Name = "cmdStart"
        Me.cmdStart.Size = New System.Drawing.Size(225, 69)
        Me.cmdStart.TabIndex = 1
        Me.cmdStart.Text = "Start Server"
        Me.cmdStart.UseVisualStyleBackColor = False
        '
        'HapticsControlerImages
        '
        Me.HapticsControlerImages.ImageStream = CType(resources.GetObject("HapticsControlerImages.ImageStream"), System.Windows.Forms.ImageListStreamer)
        Me.HapticsControlerImages.TransparentColor = System.Drawing.Color.Transparent
        Me.HapticsControlerImages.Images.SetKeyName(0, "MuxNode")
        Me.HapticsControlerImages.Images.SetKeyName(1, "ControlerNode")
        Me.HapticsControlerImages.Images.SetKeyName(2, "MotorNode")
        Me.HapticsControlerImages.Images.SetKeyName(3, "folder.png")
        '
        'cmdStop
        '
        Me.cmdStop.BackColor = System.Drawing.Color.SlateGray
        Me.cmdStop.Cursor = System.Windows.Forms.Cursors.Default
        Me.cmdStop.Enabled = False
        Me.cmdStop.FlatAppearance.BorderSize = 0
        Me.cmdStop.FlatStyle = System.Windows.Forms.FlatStyle.Flat
        Me.cmdStop.Font = New System.Drawing.Font("Microsoft Sans Serif", 16.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdStop.Location = New System.Drawing.Point(1052, 180)
        Me.cmdStop.Name = "cmdStop"
        Me.cmdStop.Size = New System.Drawing.Size(225, 69)
        Me.cmdStop.TabIndex = 12
        Me.cmdStop.Text = "Stop Server"
        Me.cmdStop.UseVisualStyleBackColor = False
        '
        'Octo_Haptics_Server_Settings_OFD
        '
        Me.Octo_Haptics_Server_Settings_OFD.Filter = "VRHaptics File|*.vrh"
        '
        'Octo_Haptics_OSC_File_Location_Finder
        '
        '
        'ActionsMenu
        '
        Me.ActionsMenu.GripMargin = New System.Windows.Forms.Padding(2, 2, 0, 2)
        Me.ActionsMenu.ImageScalingSize = New System.Drawing.Size(24, 24)
        Me.ActionsMenu.Items.AddRange(New System.Windows.Forms.ToolStripItem() {Me.Menu_Server, Me.Configuration_Menu})
        Me.ActionsMenu.Location = New System.Drawing.Point(0, 0)
        Me.ActionsMenu.Name = "ActionsMenu"
        Me.ActionsMenu.Size = New System.Drawing.Size(1312, 33)
        Me.ActionsMenu.TabIndex = 13
        Me.ActionsMenu.Text = "ActionsMenu"
        '
        'Menu_Server
        '
        Me.Menu_Server.DropDownItems.AddRange(New System.Windows.Forms.ToolStripItem() {Me.Menu_Item_Start_Server, Me.Menu_Item_Stop_Server, Me.CloseServerToolStripMenuItem})
        Me.Menu_Server.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Menu_Server.Name = "Menu_Server"
        Me.Menu_Server.Size = New System.Drawing.Size(92, 29)
        Me.Menu_Server.Text = "Server"
        '
        'Menu_Item_Start_Server
        '
        Me.Menu_Item_Start_Server.Name = "Menu_Item_Start_Server"
        Me.Menu_Item_Start_Server.Size = New System.Drawing.Size(241, 34)
        Me.Menu_Item_Start_Server.Text = "Start Servers"
        '
        'Menu_Item_Stop_Server
        '
        Me.Menu_Item_Stop_Server.Name = "Menu_Item_Stop_Server"
        Me.Menu_Item_Stop_Server.Size = New System.Drawing.Size(241, 34)
        Me.Menu_Item_Stop_Server.Text = "Stop Server"
        '
        'CloseServerToolStripMenuItem
        '
        Me.CloseServerToolStripMenuItem.Name = "CloseServerToolStripMenuItem"
        Me.CloseServerToolStripMenuItem.Size = New System.Drawing.Size(241, 34)
        Me.CloseServerToolStripMenuItem.Text = "Close Server"
        '
        'Configuration_Menu
        '
        Me.Configuration_Menu.DropDownItems.AddRange(New System.Windows.Forms.ToolStripItem() {Me.Menu_Item_Select_Osc_Folder})
        Me.Configuration_Menu.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Configuration_Menu.Name = "Configuration_Menu"
        Me.Configuration_Menu.Size = New System.Drawing.Size(157, 29)
        Me.Configuration_Menu.Text = "Configuration"
        '
        'Menu_Item_Select_Osc_Folder
        '
        Me.Menu_Item_Select_Osc_Folder.Name = "Menu_Item_Select_Osc_Folder"
        Me.Menu_Item_Select_Osc_Folder.Size = New System.Drawing.Size(296, 34)
        Me.Menu_Item_Select_Osc_Folder.Text = "Select OSC Folder"
        '
        'CustomPanel3
        '
        Me.CustomPanel3.BackColor = System.Drawing.Color.Teal
        Me.CustomPanel3.BackColor2 = System.Drawing.Color.DarkSlateGray
        Me.CustomPanel3.BorderColor = System.Drawing.SystemColors.HotTrack
        Me.CustomPanel3.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel3.BorderWidth = 7
        Me.CustomPanel3.Controls.Add(Me.Label1)
        Me.CustomPanel3.Controls.Add(Me.txtIncommingPackets)
        Me.CustomPanel3.Controls.Add(Me.cmdClearHistory)
        Me.CustomPanel3.Curvature = 30
        Me.CustomPanel3.GradientMode = OctoSlime_Haptics_Server.LinearGradientMode.Vertical
        Me.CustomPanel3.Location = New System.Drawing.Point(501, 319)
        Me.CustomPanel3.Name = "CustomPanel3"
        Me.CustomPanel3.Size = New System.Drawing.Size(776, 788)
        Me.CustomPanel3.TabIndex = 15
        '
        'Label1
        '
        Me.Label1.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label1.Location = New System.Drawing.Point(249, 13)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(279, 37)
        Me.Label1.TabIndex = 4
        Me.Label1.Text = "Server Messages"
        Me.Label1.TextAlign = System.Drawing.ContentAlignment.MiddleCenter
        '
        'txtIncommingPackets
        '
        Me.txtIncommingPackets.BackColor = System.Drawing.Color.SlateGray
        Me.txtIncommingPackets.BorderStyle = System.Windows.Forms.BorderStyle.None
        Me.txtIncommingPackets.Cursor = System.Windows.Forms.Cursors.Default
        Me.txtIncommingPackets.Font = New System.Drawing.Font("Microsoft Sans Serif", 12.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.txtIncommingPackets.ForeColor = System.Drawing.SystemColors.ControlLight
        Me.txtIncommingPackets.Location = New System.Drawing.Point(39, 54)
        Me.txtIncommingPackets.Multiline = True
        Me.txtIncommingPackets.Name = "txtIncommingPackets"
        Me.txtIncommingPackets.ReadOnly = True
        Me.txtIncommingPackets.ScrollBars = System.Windows.Forms.ScrollBars.Vertical
        Me.txtIncommingPackets.Size = New System.Drawing.Size(698, 627)
        Me.txtIncommingPackets.TabIndex = 2
        '
        'cmdClearHistory
        '
        Me.cmdClearHistory.BackColor = System.Drawing.Color.SlateGray
        Me.cmdClearHistory.Cursor = System.Windows.Forms.Cursors.Default
        Me.cmdClearHistory.FlatAppearance.BorderColor = System.Drawing.Color.Gray
        Me.cmdClearHistory.FlatAppearance.BorderSize = 0
        Me.cmdClearHistory.FlatStyle = System.Windows.Forms.FlatStyle.Flat
        Me.cmdClearHistory.Font = New System.Drawing.Font("Microsoft Sans Serif", 8.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdClearHistory.Location = New System.Drawing.Point(309, 705)
        Me.cmdClearHistory.Name = "cmdClearHistory"
        Me.cmdClearHistory.Size = New System.Drawing.Size(115, 38)
        Me.cmdClearHistory.TabIndex = 3
        Me.cmdClearHistory.Text = "Clear"
        Me.cmdClearHistory.UseVisualStyleBackColor = False
        '
        'CustomPanel2
        '
        Me.CustomPanel2.BackColor = System.Drawing.Color.Teal
        Me.CustomPanel2.BackColor2 = System.Drawing.Color.DarkSlateGray
        Me.CustomPanel2.BorderColor = System.Drawing.SystemColors.HotTrack
        Me.CustomPanel2.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel2.BorderWidth = 7
        Me.CustomPanel2.Controls.Add(Me.lblDevice)
        Me.CustomPanel2.Controls.Add(Me.cmdSend)
        Me.CustomPanel2.Controls.Add(Me.Label6)
        Me.CustomPanel2.Controls.Add(Me.pnlDigital)
        Me.CustomPanel2.Controls.Add(Me.pnlAnalogue)
        Me.CustomPanel2.Controls.Add(Me.Label4)
        Me.CustomPanel2.Curvature = 30
        Me.CustomPanel2.GradientMode = OctoSlime_Haptics_Server.LinearGradientMode.Vertical
        Me.CustomPanel2.Location = New System.Drawing.Point(512, 78)
        Me.CustomPanel2.Name = "CustomPanel2"
        Me.CustomPanel2.Size = New System.Drawing.Size(493, 231)
        Me.CustomPanel2.TabIndex = 14
        '
        'lblDevice
        '
        Me.lblDevice.BackColor = System.Drawing.Color.LightSteelBlue
        Me.lblDevice.Font = New System.Drawing.Font("Microsoft Sans Serif", 9.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.lblDevice.Location = New System.Drawing.Point(54, 169)
        Me.lblDevice.Name = "lblDevice"
        Me.lblDevice.Size = New System.Drawing.Size(385, 41)
        Me.lblDevice.TabIndex = 7
        Me.lblDevice.Text = "None"
        Me.lblDevice.TextAlign = System.Drawing.ContentAlignment.MiddleCenter
        '
        'cmdSend
        '
        Me.cmdSend.BackColor = System.Drawing.Color.SlateGray
        Me.cmdSend.Cursor = System.Windows.Forms.Cursors.Default
        Me.cmdSend.Enabled = False
        Me.cmdSend.FlatAppearance.BorderColor = System.Drawing.Color.Gray
        Me.cmdSend.FlatAppearance.BorderSize = 0
        Me.cmdSend.FlatStyle = System.Windows.Forms.FlatStyle.Flat
        Me.cmdSend.Font = New System.Drawing.Font("Microsoft Sans Serif", 8.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.cmdSend.Location = New System.Drawing.Point(314, 82)
        Me.cmdSend.Name = "cmdSend"
        Me.cmdSend.Size = New System.Drawing.Size(115, 38)
        Me.cmdSend.TabIndex = 4
        Me.cmdSend.Text = "Send"
        Me.cmdSend.UseVisualStyleBackColor = False
        '
        'Label6
        '
        Me.Label6.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label6.Location = New System.Drawing.Point(50, 141)
        Me.Label6.Name = "Label6"
        Me.Label6.Size = New System.Drawing.Size(245, 23)
        Me.Label6.TabIndex = 6
        Me.Label6.Text = "Haptic Point :"
        Me.Label6.TextAlign = System.Drawing.ContentAlignment.MiddleLeft
        '
        'pnlDigital
        '
        Me.pnlDigital.Controls.Add(Me.digitalValue)
        Me.pnlDigital.Controls.Add(Me.Label2)
        Me.pnlDigital.Location = New System.Drawing.Point(61, 63)
        Me.pnlDigital.Name = "pnlDigital"
        Me.pnlDigital.Size = New System.Drawing.Size(245, 70)
        Me.pnlDigital.TabIndex = 0
        '
        'digitalValue
        '
        Me.digitalValue.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.digitalValue.FormattingEnabled = True
        Me.digitalValue.Items.AddRange(New Object() {"On", "Off"})
        Me.digitalValue.Location = New System.Drawing.Point(149, 21)
        Me.digitalValue.MaxDropDownItems = 2
        Me.digitalValue.Name = "digitalValue"
        Me.digitalValue.Size = New System.Drawing.Size(75, 33)
        Me.digitalValue.TabIndex = 1
        Me.digitalValue.TabStop = False
        '
        'Label2
        '
        Me.Label2.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label2.Location = New System.Drawing.Point(16, 24)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(127, 23)
        Me.Label2.TabIndex = 0
        Me.Label2.Text = "Set State :"
        '
        'pnlAnalogue
        '
        Me.pnlAnalogue.Controls.Add(Me.analogueValue)
        Me.pnlAnalogue.Controls.Add(Me.Label3)
        Me.pnlAnalogue.Location = New System.Drawing.Point(61, 63)
        Me.pnlAnalogue.Name = "pnlAnalogue"
        Me.pnlAnalogue.Size = New System.Drawing.Size(245, 70)
        Me.pnlAnalogue.TabIndex = 1
        '
        'analogueValue
        '
        Me.analogueValue.BorderStyle = System.Windows.Forms.BorderStyle.None
        Me.analogueValue.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.analogueValue.Increment = New Decimal(New Integer() {8, 0, 0, 0})
        Me.analogueValue.Location = New System.Drawing.Point(149, 21)
        Me.analogueValue.Maximum = New Decimal(New Integer() {255, 0, 0, 0})
        Me.analogueValue.Name = "analogueValue"
        Me.analogueValue.Size = New System.Drawing.Size(75, 26)
        Me.analogueValue.TabIndex = 1
        '
        'Label3
        '
        Me.Label3.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label3.Location = New System.Drawing.Point(16, 24)
        Me.Label3.Name = "Label3"
        Me.Label3.Size = New System.Drawing.Size(127, 23)
        Me.Label3.TabIndex = 0
        Me.Label3.Text = "Set Level :"
        '
        'Label4
        '
        Me.Label4.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label4.Location = New System.Drawing.Point(165, 18)
        Me.Label4.Name = "Label4"
        Me.Label4.Size = New System.Drawing.Size(158, 23)
        Me.Label4.TabIndex = 5
        Me.Label4.Text = "Haptic Output"
        Me.Label4.TextAlign = System.Drawing.ContentAlignment.MiddleCenter
        '
        'CustomPanel1
        '
        Me.CustomPanel1.BackColor = System.Drawing.Color.Teal
        Me.CustomPanel1.BackColor2 = System.Drawing.Color.DarkSlateGray
        Me.CustomPanel1.BorderColor = System.Drawing.SystemColors.HotTrack
        Me.CustomPanel1.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.CustomPanel1.BorderWidth = 7
        Me.CustomPanel1.Controls.Add(Me.Label5)
        Me.CustomPanel1.Controls.Add(Me.ControlerTree)
        Me.CustomPanel1.Curvature = 30
        Me.CustomPanel1.GradientMode = OctoSlime_Haptics_Server.LinearGradientMode.Vertical
        Me.CustomPanel1.Location = New System.Drawing.Point(12, 78)
        Me.CustomPanel1.Name = "CustomPanel1"
        Me.CustomPanel1.Size = New System.Drawing.Size(474, 1029)
        Me.CustomPanel1.TabIndex = 5
        '
        'Label5
        '
        Me.Label5.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.Label5.Location = New System.Drawing.Point(26, 9)
        Me.Label5.Name = "Label5"
        Me.Label5.Size = New System.Drawing.Size(383, 42)
        Me.Label5.TabIndex = 10
        Me.Label5.Text = "Connected Haptic Devices"
        Me.Label5.TextAlign = System.Drawing.ContentAlignment.MiddleCenter
        '
        'ControlerTree
        '
        Me.ControlerTree.BackColor = System.Drawing.Color.SlateGray
        Me.ControlerTree.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle
        Me.ControlerTree.Font = New System.Drawing.Font("Microsoft Sans Serif", 10.0!, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        Me.ControlerTree.ForeColor = System.Drawing.SystemColors.Info
        Me.ControlerTree.ImageIndex = 0
        Me.ControlerTree.ImageList = Me.HapticsControlerImages
        Me.ControlerTree.LineColor = System.Drawing.Color.FromArgb(CType(CType(153, Byte), Integer), CType(CType(180, Byte), Integer), CType(CType(209, Byte), Integer))
        Me.ControlerTree.Location = New System.Drawing.Point(38, 66)
        Me.ControlerTree.Name = "ControlerTree"
        Me.ControlerTree.SelectedImageIndex = 0
        Me.ControlerTree.Size = New System.Drawing.Size(399, 920)
        Me.ControlerTree.TabIndex = 9
        Me.ControlerTree.TabStop = False
        '
        'OctoSlimeHapticsServer
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(9.0!, 20.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.BackColor = System.Drawing.Color.DarkRed
        Me.BackgroundImageLayout = System.Windows.Forms.ImageLayout.Stretch
        Me.ClientSize = New System.Drawing.Size(1312, 1145)
        Me.Controls.Add(Me.CustomPanel3)
        Me.Controls.Add(Me.CustomPanel2)
        Me.Controls.Add(Me.CustomPanel1)
        Me.Controls.Add(Me.cmdStop)
        Me.Controls.Add(Me.cmdStart)
        Me.Controls.Add(Me.ActionsMenu)
        Me.Cursor = System.Windows.Forms.Cursors.Default
        Me.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedDialog
        Me.Icon = CType(resources.GetObject("$this.Icon"), System.Drawing.Icon)
        Me.MainMenuStrip = Me.ActionsMenu
        Me.MaximizeBox = False
        Me.MinimizeBox = False
        Me.Name = "OctoSlimeHapticsServer"
        Me.Text = "OctoSlime Haptics Server"
        Me.ActionsMenu.ResumeLayout(False)
        Me.ActionsMenu.PerformLayout()
        Me.CustomPanel3.ResumeLayout(False)
        Me.CustomPanel3.PerformLayout()
        Me.CustomPanel2.ResumeLayout(False)
        Me.pnlDigital.ResumeLayout(False)
        Me.pnlAnalogue.ResumeLayout(False)
        CType(Me.analogueValue, System.ComponentModel.ISupportInitialize).EndInit()
        Me.CustomPanel1.ResumeLayout(False)
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents cmdStart As Button
    Friend WithEvents txtIncommingPackets As TextBox
    Friend WithEvents cmdClearHistory As Button
    Friend WithEvents ControlerTree As TreeView
    Friend WithEvents HapticsControlerImages As ImageList
    Friend WithEvents cmdSend As Button
    Friend WithEvents analogueValue As NumericUpDown
    Friend WithEvents Label3 As Label
    Friend WithEvents digitalValue As ComboBox
    Friend WithEvents Label2 As Label
    Friend WithEvents Label1 As Label
    Friend WithEvents Label4 As Label
    Friend WithEvents Label5 As Label
    Friend WithEvents cmdStop As Button
    Friend WithEvents lblDevice As Label
    Friend WithEvents Label6 As Label
    Friend WithEvents Octo_Haptics_Server_Settings_OFD As OpenFileDialog
    Friend WithEvents Octo_Haptics_OSC_File_Location_Finder As FolderBrowserDialog
    Friend WithEvents ActionsMenu As MenuStrip
    Friend WithEvents Menu_Server As ToolStripMenuItem
    Friend WithEvents Menu_Item_Start_Server As ToolStripMenuItem
    Friend WithEvents Menu_Item_Stop_Server As ToolStripMenuItem
    Friend WithEvents Configuration_Menu As ToolStripMenuItem
    Friend WithEvents Menu_Item_Select_Osc_Folder As ToolStripMenuItem
    Friend WithEvents CustomPanel1 As CustomPanel
    Friend WithEvents pnlAnalogue As Panel
    Friend WithEvents pnlDigital As Panel
    Friend WithEvents CustomPanel2 As CustomPanel
    Friend WithEvents CustomPanel3 As CustomPanel
    Friend WithEvents CloseServerToolStripMenuItem As ToolStripMenuItem
End Class
