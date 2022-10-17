Imports System.Management
Imports System.IO
Imports System.IO.Ports
Imports System.ComponentModel
Imports System.Data.OleDb
Imports System.Threading

Public Class frmFloatConfig

    Private OKRecieved As Boolean = False
    Private CmdRecieved As Boolean = False

    Private dt As New DataTable

    Const ASCII_STX As Byte = 2
    Const ASCII_ETX As Byte = 3
    Const ASCII_ENQ As Byte = 5
    Const ASCII_ACK As Byte = 6

    Const FiveSeconds As Long = 50000000


    Sub New()

        InitializeComponent()

        RoundButton(cmdFirmwareUpdate, Color.SlateGray)
        RoundButton(cmdRead, Color.SlateGray)
        RoundButton(cmdWrite, Color.SlateGray)
        RoundButton(cmdRead, Color.SlateGray)
        RoundButton(cmdRefresh, Color.SlateGray)
        RoundButton(cmdRestartVTS, Color.SlateGray)

    End Sub

    Private Function gen_MAC() As String

        Dim random = New Random()
        Dim buffer = New Byte(5) {}
        random.NextBytes(buffer)
        buffer(0) = 2
        Return String.Join("-", buffer.[Select](Function(x) String.Format("{0}", x.ToString("X2"))).ToArray())
    End Function


    Private Sub frmFloatConfig_Load(sender As Object, e As EventArgs) Handles Me.Load

        RefreshComList()

    End Sub

    Private Sub cmdRead_Click(sender As Object, e As EventArgs) Handles cmdRead.Click
        Cursor.Current = Cursors.WaitCursor

        Try
            If FloatInterface.IsOpen Then FloatInterface.Close()
            FloatInterface.PortName = GetPort(ComPortSelect.SelectedItem.ToString)
            FloatInterface.Open()
            FloatInterface.RtsEnable = False
            FloatInterface.RtsEnable = True
            FloatInterface.RtsEnable = False
            SetToolStripLableText(StatusStrip1, "lblStatus", "Port Open")
            Threading.Thread.Sleep(2000)
            FloatInterface.NewLine = vbCrLf
            FloatInterface.WriteLine("Q")
            WaitCmdRecieved()
        Catch ex As Exception
            If FloatInterface.IsOpen Then FloatInterface.Close()
            SetToolStripLableText(StatusStrip1, "lblStatus", ex.Message)
        End Try

        Cursor.Current = Cursors.Default
        Cursor.Show()
        Application.DoEvents()

    End Sub

    Private Sub cmdWrite_Click(sender As Object, e As EventArgs) Handles cmdWrite.Click

        Cursor.Current = Cursors.WaitCursor

        Try
            If FloatInterface.IsOpen Then FloatInterface.Close()
            FloatInterface.PortName = GetPort(ComPortSelect.SelectedItem.ToString)
            FloatInterface.Open()
            FloatInterface.RtsEnable = False
            FloatInterface.RtsEnable = True
            FloatInterface.RtsEnable = False
            SetToolStripLableText(StatusStrip1, "lblStatus", "Port Open")
            Threading.Thread.Sleep(2000)

            FloatInterface.NewLine = vbCrLf

            Do
                SetToolStripLableText(StatusStrip1, "lblStatus", "Writing WIFI SSID")
                FloatInterface.Write("S:")
                FloatInterface.WriteLine(txtSSID.Text)
            Loop Until WaitForOK()

            Do
                SetToolStripLableText(StatusStrip1, "lblStatus", "Writing WIFI Password")
                FloatInterface.Write("P:")
                FloatInterface.WriteLine(txtPassword.Text)
            Loop Until WaitForOK()

            Do
                SetToolStripLableText(StatusStrip1, "lblStatus", "Writing Haptics Active Status")
                FloatInterface.Write("H:")
                FloatInterface.WriteLine(chkHapticsActive.Checked.ToString)
            Loop Until WaitForOK()

            While FloatInterface.BytesToWrite > 0
                Application.DoEvents()
            End While

            SetToolStripLableText(StatusStrip1, "lblStatus", "Write Complete")


        Catch ex As Exception
            If FloatInterface.IsOpen Then FloatInterface.Close()
            SetToolStripLableText(StatusStrip1, "lblStatus", ex.Message)

        End Try


        FloatInterface.Close()

        Cursor.Current = Cursors.Default
        Cursor.Show()
        Application.DoEvents()



    End Sub

    Private Function GetPort(Port As String) As String
        If Port.StartsWith("COM") Then GetPort = Port
        If Port.Contains("("c) Then
            GetPort = Port.Substring(Port.LastIndexOf("("c)).Replace("("c, "").Replace(")"c, "")
        Else
            GetPort = Port
        End If
    End Function

    Private Sub FloatInterface_DataReceived(sender As Object, e As SerialDataReceivedEventArgs) Handles FloatInterface.DataReceived

        Try
            FloatInterface.ReadTimeout = 8000

            Dim IncomingByte As Integer

            While FloatInterface.BytesToRead > 0
                IncomingByte = FloatInterface.ReadChar()
                Select Case IncomingByte
                    Case ASCII_ETX
                        OKRecieved = True
                    Case ASCII_ENQ
                        CmdRecieved = True
                    Case Asc("Q")
                        Dim Config As String = FloatInterface.ReadLine()
                        Dim Pairs() As String = Config.Split(";"c)
                        For Each Pair As String In Pairs
                            Dim Params() As String = Pair.Split(","c)
                            Select Case Params(0)
                                Case "S"    ' SSID
                                    SetControlText(txtSSID, Params(1))
                                Case "P"    ' Password
                                    SetControlText(txtPassword, Params(1))
                                Case "H"    ' Haptic Status
                                    SetCheckBoxValue(chkHapticsActive, CBool(Params(1)))
                            End Select
                        Next



                        SetToolStripLableText(StatusStrip1, "lblStatus", "Read Complete")
                        FloatInterface.Close()
                        Exit While
                    Case Else
                        OKRecieved = False
                        CmdRecieved = False
                End Select
            End While

        Catch ex As Exception
            If FloatInterface.IsOpen Then FloatInterface.Close()
            SetToolStripLableText(StatusStrip1, "lblStatus", ex.Message)
        End Try

    End Sub

    Private Sub SetToolStripLableText(ByVal ctl As ToolStrip, ByVal Item As String, ByVal Text As String)
        Try

            If ctl.InvokeRequired Then
                ctl.BeginInvoke(New Action(Of ToolStrip, String, String)(AddressOf SetToolStripLableText), ctl, Item, Text)
            Else
                ctl.Items.Item(Item).Text = Text
                ctl.Update()
                Application.DoEvents()
            End If
        Catch ex As Exception

        End Try
    End Sub


    Private Sub SetControlSelectedIndex(ByVal ctl As ComboBox, ByVal Value As Integer)
        Try

            If ctl.InvokeRequired Then
                ctl.BeginInvoke(New Action(Of ComboBox, Integer)(AddressOf SetControlSelectedIndex), ctl, Value)
            Else
                ctl.SelectedIndex = Value
                Application.DoEvents()
            End If
        Catch ex As Exception

        End Try
    End Sub


    Private Sub SetControlValue(ByVal ctl As NumericUpDown, ByVal Value As Decimal)
        Try

            If ctl.InvokeRequired Then
                ctl.BeginInvoke(New Action(Of NumericUpDown, Decimal)(AddressOf SetControlValue), ctl, Value)
            Else
                ctl.Value = Value
                Application.DoEvents()
            End If
        Catch ex As Exception

        End Try
    End Sub



    Private Sub SetCheckBoxValue(ByVal ctl As CheckBox, ByVal value As Boolean)
        Try

            If ctl.InvokeRequired Then
                ctl.BeginInvoke(New Action(Of CheckBox, Boolean)(AddressOf SetCheckBoxValue), ctl, value)
            Else
                ctl.Checked = value
                ctl.Update()
                Application.DoEvents()
            End If
        Catch ex As Exception

        End Try
    End Sub


    Private Sub SetControlText(ByVal ctl As Control, ByVal Text As String)
        Try

            If ctl.InvokeRequired Then
                ctl.BeginInvoke(New Action(Of Control, String)(AddressOf SetControlText), ctl, Text)
            Else
                ctl.Text = Text
                ctl.Update()
                Application.DoEvents()
            End If
        Catch ex As Exception

        End Try
    End Sub

    Private Sub ComPortSelect_SelectedIndexChanged(sender As Object, e As EventArgs) Handles ComPortSelect.SelectedIndexChanged
        SetToolStripLableText(StatusStrip1, "lblStatus", "Data Port Set to " & GetPort(ComPortSelect.SelectedItem.ToString))
    End Sub

    Private Sub WaitCmdRecieved()
        CmdRecieved = False
        Dim Timeout As Long = Now.ToFileTimeUtc + FiveSeconds
        While ((Not CmdRecieved))
            FloatInterface.WriteLine("Q")
            Application.DoEvents()
            If Now.ToFileTimeUtc > Timeout Then
                Throw New Exception("Connection Timeout")
                FloatInterface.Close()
            End If
            Thread.Sleep(500)
        End While
    End Sub

    Private Function WaitForOK() As Boolean
        OKRecieved = False
        Dim Timeout As Long = Now.ToFileTimeUtc + FiveSeconds
        While ((Not OKRecieved))
            Application.DoEvents()
            If Now.ToFileTimeUtc > Timeout Then
                Return False
            End If
        End While
        Return True
    End Function

    Private Sub frmFloatConfig_Closing(sender As Object, e As CancelEventArgs) Handles Me.Closing
        If FloatInterface.IsOpen Then FloatInterface.Close()
    End Sub

    Private Sub cmdRefresh_Click(sender As Object, e As EventArgs) Handles cmdRefresh.Click
        RefreshComList()
    End Sub

    Private Sub RefreshComList()

        Cursor.Current = Cursors.WaitCursor

        ComPortSelect.Items.Clear()

        Try

            For Each sp As String In My.Computer.Ports.SerialPortNames
                ComPortSelect.Items.Add(sp)
            Next

        Catch err As ManagementException
            MessageBox.Show("An error occurred while querying for WMI data: " & err.Message)
        End Try

        If ComPortSelect.Items.Count > 0 Then ComPortSelect.SelectedIndex = 0

        Cursor.Current = Cursors.Default

    End Sub

    Private Sub Button1_Click(sender As Object, e As EventArgs) Handles cmdRestartVTS.Click
        Try
            If FloatInterface.IsOpen Then FloatInterface.Close()
            FloatInterface.PortName = GetPort(ComPortSelect.SelectedItem.ToString)
            FloatInterface.Open()
            FloatInterface.RtsEnable = False
            FloatInterface.RtsEnable = True
            FloatInterface.RtsEnable = False
            FloatInterface.Close()
        Catch ex As Exception
            If FloatInterface.IsOpen Then
                FloatInterface.Close()
            End If
        End Try

        Cursor.Current = Cursors.Default
        Cursor.Show()
        Application.DoEvents()
    End Sub


    Private Sub cmdFirmwareUpdate_Click(sender As Object, e As EventArgs) Handles cmdFirmwareUpdate.Click
        Dim folder = My.Application.Info.DirectoryPath & "\Firmware\"
        SelectFirmwareFile.InitialDirectory = folder
        SelectFirmwareFile.Filter = "OctoSlime Firmware|*.BIN"

        If SelectFirmwareFile.ShowDialog = DialogResult.OK Then

            Dim CommandString As String = "Uploader\python3 Uploader\upload.py --chip esp8266 --port  "
            CommandString &= GetPort(ComPortSelect.SelectedItem.ToString) & " "
            CommandString &= "--baud 921600 --before default_reset --after hard_reset write_flash 0x0 "
            CommandString &= "Firmware\" & System.IO.Path.GetFileName(SelectFirmwareFile.FileName)

            If MsgBox("Confirm Firmware Update", MsgBoxStyle.ApplicationModal Or MsgBoxStyle.Critical Or MsgBoxStyle.YesNo, "WARNING !") = MsgBoxResult.Yes Then
                CommandString.RunCMD(True, True, False)
            End If

        End If

        Cursor.Current = Cursors.Default
        Cursor.Show()
        Application.DoEvents()

    End Sub



    Private Sub RoundButton(btn As Button, Colour As Color)

        btn.FlatStyle = FlatStyle.Flat
        btn.FlatAppearance.BorderSize = 0
        btn.BackColor = Colour
        btn.ForeColor = Color.White
        btn.Cursor = Cursors.Default
        '  btn.Font = New Font("Century Gothic", 14)

        Dim Raduis As New Drawing2D.GraphicsPath

        Raduis.StartFigure()
        'appends an elliptical arc to the current figure
        'left corner top
        Raduis.AddArc(New Rectangle(0, 0, 20, 20), 180, 90)
        'appends a line segment to the current figure
        Raduis.AddLine(10, 0, btn.Width - 20, 0)
        'appends an elliptical arc to the current figure
        'right corner top
        Raduis.AddArc(New Rectangle(btn.Width - 20, 0, 20, 20), -90, 90)
        'appends a line segment to the current figure
        Raduis.AddLine(btn.Width, 20, btn.Width, btn.Height - 10)
        'appends an elliptical arc to the current figure 
        'right corner buttom
        Raduis.AddArc(New Rectangle(btn.Width - 25, btn.Height - 25, 25, 25), 0, 90)
        'appends a line segment to the current figure
        'left corner bottom
        Raduis.AddLine(btn.Width - 10, btn.Width, 20, btn.Height)
        'appends an elliptical arc to the current figure
        Raduis.AddArc(New Rectangle(0, btn.Height - 20, 20, 20), 90, 90)
        'Close the current figure and start a new one.
        Raduis.CloseFigure()
        'set the window associated with the control
        btn.Region = New Region(Raduis)
    End Sub


End Class
