Imports System.Net.Sockets
Imports System.Text
Imports System.IO
Imports System.Net
Imports System.Threading

Public Class OctoSlimeHapticsServer

#Region "Constants"
    Const PACKET_OCTOSLIME_HAPTICS_INFO = &H70
    Const PACKET_OCTOSLIME_HAPTICS_RECIEVE_INFO = &H71
    Const PACKET_OCTOSLIME_HAPTICS_HANDSHAKE = &H72
    Const PACKET_OCTOSLIME_HAPTICS_RECEIVE_HANDSHAKE = &H73
    Const PACKET_OCTOSLIME_HAPTICS_SET = &H74
    Const PACKET_OCTOSLIME_HAPTICS_RECEWIVE_SET = &H75
    Const PACKET_OCTOSLIME_HAPTICS_HEARTBEAT = &H76
    Const PACKET_OCTOSLIME_HAPTICS_RECEIVE_HEARTBEAT = &H77
#End Region

#Region "Variables"

    Private DefaultValues As String() = {"IsLocal", "Viseme", "Voice", "GestureLeft", "GestureRight", "GestureLeftWeight", "GestureRightWeight", "AngularY", "VelocityX", "VelocityY", "VelocityZ", "Upright", "Grounded", "Seated", "AFK", "TrackingType", "VRMode", "MuteSelf", "InStation", "Expression"}

    Private OutGoingMessages As Stack = New Stack()
    Private incommingMessages As Stack = New Stack()
    Private ServersRunning As Boolean = False
    Private ClickedControllerParams As ControllerParams

    Private Octoslime_Socket As UdpClient
    Private Octoslime_Socket_Endpoint As New IPEndPoint(IPAddress.Broadcast, 2022)

    Dim OSC_Socket_in As New UdpClient
    Dim OSC_Endpoint_In As New IPEndPoint(System.Net.IPAddress.Parse("127.0.0.1"), 0)

    Private Devices As New Dictionary(Of Integer, Devices_t)
    Private BroadcastBytes As Byte()
    Private LastPacketTime As Long = 0
    Private Server_Connected As Boolean = False

    Private User_Avitars_File_Names As New List(Of String)
    Private AppDatafolder As String = String.Empty


#End Region

#Region "Structures"
    Private Structure MotorLevels_t
        Public MotorID As Integer
        Public MotorLevel As Integer
        Public Sub New(Motor As Integer, Level As Integer)
            MotorID = Motor
            MotorLevel = Level
        End Sub
    End Structure
    Private Structure Devices_t
        Public MuxID As Integer
        Public ControlerID As Integer
        Public DeviceTypeID As Integer
        Public MotorLevels As Dictionary(Of Integer, Integer)
        Public Sub New(MUX As Integer, Controler As Integer, DeviceType As Integer)
            MuxID = MUX
            ControlerID = Controler
            DeviceTypeID = DeviceType
            MotorLevels = New Dictionary(Of Integer, Integer) From {{1, 0}, {2, 0}, {3, 0}, {4, 0}, {5, 0}, {6, 0}, {7, 0}, {8, 0}}
        End Sub
    End Structure
#End Region

#Region "Enmirations"
    Private Enum Controlertype_t As Byte
        Digital = 1
        Analogue = 2
    End Enum
#End Region

#Region "Methods"
    Sub New()

        InitializeComponent()

        RoundButton(cmdStart, Color.SlateGray)
        RoundButton(cmdStop, Color.SlateGray)
        RoundButton(cmdSend, Color.SlateGray)
        RoundButton(cmdClearHistory, Color.SlateGray)

        txtIncommingPackets.Text = String.Empty

        Try
            Octoslime_Socket = New UdpClient(2022)
            Octoslime_Socket.EnableBroadcast = True

            OSC_Socket_in = New UdpClient(9001)
            OSC_Socket_in.EnableBroadcast = True
            OSC_Socket_in.Client.ReceiveTimeout = 10 'How long to wait before moving on
            OSC_Socket_in.Client.Blocking = False 'Don't allow it to block processing

        Catch ex As Exception
            UpdateResults("[Error] socket creation" & ex.Message)
        End Try

    End Sub
    Private Sub StartThreads()

        Try
            ServersRunning = True

            Dim OctoSlime_Listner As New Thread(AddressOf OctoSlime_Listner_Thread)
            OctoSlime_Listner.IsBackground = True
            OctoSlime_Listner.Start()

            Dim OctoSlime_Sender As New Thread(AddressOf OctoSlime_Sender_Thread)
            OctoSlime_Sender.IsBackground = True
            OctoSlime_Sender.Start()

            Dim Server_Watchdog As New Thread(AddressOf Server_Watchdog_Thread)
            Server_Watchdog.IsBackground = True
            Server_Watchdog.Start()

            Dim OSC_Listener As New Thread(AddressOf OSC_Listner_Thread)
            OSC_Listener.IsBackground = True
            OSC_Listener.Start()

        Catch ex As Exception
            UpdateResults("[Error] Thread creation" & ex.Message)
        End Try

    End Sub
    Private Sub OSC_Listner_Thread()

        Dim OSC_Message As Byte()
        Dim Message_values(20) As Byte
        Dim Message_params(100) As Byte
        Dim Values_Pointer As Integer = 0
        Dim Params_Pointer As Integer = 0

        Do While ServersRunning

            Try
                If OSC_Socket_in.Available() Then

                    Values_Pointer = 0
                    Params_Pointer = 0

                    OSC_Message = OSC_Socket_in.Receive(OSC_Endpoint_In)

                    UpdateResults("[Incoming OSC Message] " & System.Text.UTF8Encoding.ASCII.GetString(OSC_Message))

                    For MessagePointer As Integer = 0 To OSC_Message.Length - 1 'For all the bytes...
                        If OSC_Message(MessagePointer) = 44 OrElse Values_Pointer > 0 Then 'look for the first comma (ASCII 44)
                            Message_values(Values_Pointer) = OSC_Message(MessagePointer) 'once you found it, save the value into a new byte set
                            Values_Pointer += 1
                        Else
                            Message_params(Params_Pointer) = OSC_Message(MessagePointer) 'If you haven't found it yet, save the bytes for parameter name
                            Params_Pointer += 1
                        End If
                    Next

                    Process_OCS_Messages(Message_params, Message_values)

                Else
                    Thread.Sleep(100)
                End If

            Catch ex As Exception
                UpdateResults("[Error] " & ex.Message)
            End Try
        Loop

    End Sub


    Private Sub Process_OCS_Messages(ByRef Param() As Byte, ByRef Value() As Byte)
        Try

            Dim OSCP As String = System.Text.UTF8Encoding.ASCII.GetString(Param) 'Convert the parameter name bytes to a string
            Dim OSCvalue As String = String.Empty


            If OSCP.ToUpper.Contains("OCTOHAPTICS") Then


                Select Case Value(1)
                    Case 102 ' float value
                        Dim floatvalue(3) As Byte 'Seperate those bytes
                        For i = 0 To 3
                            floatvalue(3 - i) = Value(4 + i) 'Reverse order them cause why not?
                        Next
                        Dim valuefloat As Single = BitConverter.ToSingle(floatvalue, 0) 'Use fancy windows magic to convert it lol
                        OSCvalue = valuefloat.ToString 'Save it to a string for displaying
                    Case 105 ' int value
                        Dim valueint As Integer = Value(7) 'Pull that int value out
                        OSCvalue = valueint.ToString 'Save it to string
                    Case 84 ' boolean value
                        OSCvalue = "True"
                    Case 70 ' boolean value
                        OSCvalue = "False"
                End Select

                If OSCvalue <> String.Empty Then

                    OSCP = OSCP.Substring(OSCP.LastIndexOf("/") + 1)

                    If Array.Exists(DefaultValues, Function(x) OSCP.IndexOf(x) <> -1) Then
                        ReadinJSONs()
                    Else
                        ActionOSCMessage(OSCP, OSCvalue)
                        UpdateResults("[Incomming OCS Vlaue] " & OSCP & " - " & OSCvalue)
                    End If

                End If

            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub

    Private Sub FolderBrowserDialog1_HelpRequest(sender As Object, e As EventArgs) Handles Octo_Haptics_OSC_File_Location_Finder.HelpRequest
        Try
            AppDatafolder = Octo_Haptics_OSC_File_Location_Finder.SelectedPath
            ReadinJSONs()
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub

    Public Sub ReadinJSONs()
        Try
            Dim VRC_Users_Folders As String() = System.IO.Directory.GetDirectories(AppDatafolder)
            For User_Folder_Pointer As Integer = 0 To VRC_Users_Folders.Count - 1
                Dim VRC_Users_Sub_Folders As String() = System.IO.Directory.GetDirectories(VRC_Users_Folders(User_Folder_Pointer))
                For Users_Sub_Folders_Pointer As Integer = 0 To VRC_Users_Sub_Folders.Count - 1
                    Dim VRC_Users_Avitars As String() = System.IO.Directory.GetFiles(VRC_Users_Sub_Folders(Users_Sub_Folders_Pointer))
                    For Users_Avitars_Pointer As Integer = 0 To VRC_Users_Avitars.Count - 1
                        User_Avitars_File_Names.Add(VRC_Users_Avitars(Users_Avitars_Pointer))
                    Next
                Next
            Next
            My.Settings.DefFolder = AppDatafolder
        Catch ex As Exception
            UpdateResults("[Error] Reading Avitar Files from : " & AppDatafolder)
        End Try

        Dim avatarfileindex As Integer = 0
        Try
            For i = 0 To User_Avitars_File_Names.Count - 1

                avatarfileindex = i

                Dim readstring As String()  'Define the string to read into
                Dim WriteList As New List(Of String) 'Define file to write back to

                readstring = System.IO.File.ReadAllLines(User_Avitars_File_Names(i)) 'Read the file into the string array!

                For i2 = 0 To readstring.Count - 1

                    If readstring(i2).Contains("{") Then
                        If readstring(i2 + 1).Contains("""name"":") Then

                            If Array.Exists(DefaultValues, Function(x) readstring(i2 + 1).IndexOf(x) <> -1) Then
                                i2 = i2 + 6
                            Else
                                WriteList.Add(readstring(i2))
                            End If

                        Else
                            WriteList.Add(readstring(i2))
                        End If
                    Else
                        WriteList.Add(readstring(i2))
                    End If
                Next

                'Write back file
                'But First, fix a JSON file thing to remove a comma
                WriteList(WriteList.Count - 3) = "    }"

                Dim writestring() As String = WriteList.ToArray 'Convert List To array for file writer

                System.IO.File.WriteAllLines(User_Avitars_File_Names(i), writestring) 'Write that file from the array.


            Next

        Catch ex As Exception
            UpdateResults("[Error] when modifying file: " & User_Avitars_File_Names(avatarfileindex) & " -> " & ex.Message)
        End Try


    End Sub
    Private Sub ActionOSCMessage(ByRef ParamName As String, ParamValue As String)
        Try

            Dim NodeName As Byte() = System.Text.Encoding.ASCII.GetBytes(ParamName.Replace("_", ""))
            Dim Value As Byte() = System.Text.Encoding.ASCII.GetBytes(ParamValue)

            If Value(0) >= 0 AndAlso Value(0) < 256 Then

                If NodeName.Length = 3 Then
                    If Devices.ContainsKey(CByte(NodeName(0)) * 8 + CByte(NodeName(1))) Then
                        SendServerResponseToken({PACKET_OCTOSLIME_HAPTICS_SET, NodeName(0), NodeName(1), NodeName(2), Value(0)})
                    Else
                        UpdateResults("[Unregistered Haptic Device] Node ID : " & NodeName(0) & " Controler ID : " & NodeName(1))
                    End If
                Else
                    UpdateResults("[Invalid Haptic] " & ParamName)
                End If

            Else
                UpdateResults("[Invalid Haptic Value {" & Value(0) & "} must be in range 0-255")
            End If

        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub



    Private Sub Server_Watchdog_Thread()
        Try
            LastPacketTime = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() + 5000
            Do While ServersRunning
                If (LastPacketTime + 3000) < DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() AndAlso Server_Connected Then
                    ClearTree()
                    UpdateResults("[Error] Octoslime Connection Lost, restart server")
                    SetEnabledXthread(cmdStop, False)
                    SetEnabledXthread(cmdStart, True)
                    SetEnabledXthread(cmdSend, False)
                    SetVisibleXthread(pnlAnalogue, False)
                    SetVisibleXthread(pnlDigital, False)
                    ServersRunning = False
                    Server_Connected = False
                    SetStateColour(Me, Color.DarkRed)
                    If Octoslime_Socket IsNot Nothing Then
                        Octoslime_Socket.Close()
                        Octoslime_Socket = Nothing
                    End If
                Else
                    SetStateColour(Me, Color.DarkGreen)
                End If
                Thread.Sleep(100)
            Loop
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub

    Private Sub SetStateColour(ByVal Target As Control, ByVal State As Color)
        Try
            If Target.InvokeRequired Then
                Target.BeginInvoke(New Action(Of Control, Color)(AddressOf SetStateColour), Target, State)
            Else
                Target.BackColor = State
            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub



    Private Sub SetEnabledXthread(ByVal Target As Control, ByVal State As Boolean)
        Try
            If Target.InvokeRequired Then
                Target.BeginInvoke(New Action(Of Control, Boolean)(AddressOf SetEnabledXthread), Target, State)
            Else
                Target.Enabled = State
            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub

    Private Sub SetVisibleXthread(ByVal Target As Control, ByVal State As Boolean)
        Try
            If Target.InvokeRequired Then
                Target.BeginInvoke(New Action(Of Control, Boolean)(AddressOf SetVisibleXthread), Target, State)
            Else
                Target.Visible = State
            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub
    Private Sub OctoSlime_Listner_Thread()
        Try
            If Octoslime_Socket Is Nothing Then Octoslime_Socket = New UdpClient(2022)

            Do While ServersRunning
                If Octoslime_Socket.Available > 0 Then
                    Dim ReceivedBytes As Byte() = Octoslime_Socket.Receive(Octoslime_Socket_Endpoint)
                    ProcessMessages(ReceivedBytes)
                Else
                    Thread.Sleep(100)
                End If
            Loop
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
            If Octoslime_Socket IsNot Nothing Then
                Octoslime_Socket.Close()
                Octoslime_Socket = Nothing
            End If

        End Try
    End Sub

    Private Sub PushBroadcastMessage(ByRef Token As String)
        Try
            If ServersRunning Then OutGoingMessages.Push(Token)
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub

    Private Sub OctoSlime_Sender_Thread()
        If ServersRunning Then

            If Octoslime_Socket Is Nothing Then
                Try
                    Octoslime_Socket = New UdpClient(2022)
                    Octoslime_Socket.EnableBroadcast = True
                Catch ex As Exception
                    UpdateResults("[Error] " & ex.Message)
                    ServersRunning = False
                End Try
            End If

            Do While ServersRunning
                Try
                    If OutGoingMessages.Count > 0 Then
                        BroadcastBytes = System.Text.Encoding.UTF8.GetBytes(OutGoingMessages.Pop)
                        Octoslime_Socket.Send(BroadcastBytes, BroadcastBytes.Length, Octoslime_Socket_Endpoint)
                    Else
                        Thread.Sleep(100)
                    End If
                Catch ex As Exception
                    UpdateResults("[Error] " & ex.Message)
                End Try
            Loop

        End If
    End Sub
    Private Sub ProcessMessages(bytRecieved As Byte())
        Try

            Select Case bytRecieved(3)

                Case PACKET_OCTOSLIME_HAPTICS_HANDSHAKE ' Handshake packet
                    SendServerResponseToken({PACKET_OCTOSLIME_HAPTICS_RECEIVE_HANDSHAKE})
                    Server_Connected = True


                Case PACKET_OCTOSLIME_HAPTICS_INFO ' Discovery packet
                    SendServerResponseToken({PACKET_OCTOSLIME_HAPTICS_RECIEVE_INFO, bytRecieved(4), bytRecieved(5), 1})

                    Dim DeviceID As Integer = (bytRecieved(4) * 8) + bytRecieved(5)
                    If Not Devices.ContainsKey(DeviceID) Then
                        Devices.Add(DeviceID, New Devices_t(bytRecieved(4), bytRecieved(5), bytRecieved(6)))
                        UpdateDevicesTree(bytRecieved(4), bytRecieved(5), bytRecieved(6))
                    End If

                Case PACKET_OCTOSLIME_HAPTICS_HEARTBEAT
                    SendServerResponseToken({PACKET_OCTOSLIME_HAPTICS_HEARTBEAT})

            End Select

            LastPacketTime = DateTimeOffset.UtcNow.ToUnixTimeMilliseconds()

        Catch ex As Exception
            UpdateResults("[Error] Creating UDP Client")
        End Try

    End Sub
    Private Sub SendServerResponseToken(ByRef Token As Byte())
        Try
            PushBroadcastMessage(System.Text.Encoding.UTF8.GetString(Token))
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub
    Private Sub UpdateDevicesTree(MUXID As Byte, ControlerID As Byte, DeviceTypeID As Byte)
        Try
            If (ControlerTree.InvokeRequired) Then
                ControlerTree.Invoke(Sub() UpdateDevicesTree(MUXID, ControlerID, DeviceTypeID))
            Else
                Dim MuxNode As TreeNode

                If Not ControlerTree.Nodes.ContainsKey(MUXID) Then
                    MuxNode = ControlerTree.Nodes.Add(MUXID, "MUX : " & MUXID)
                    MuxNode.ImageIndex = 0
                    MuxNode.SelectedImageIndex = 0
                Else
                    MuxNode = ControlerTree.Nodes(MUXID)
                End If

                If Not MuxNode.Nodes.ContainsKey(ControlerID) Then
                    Dim ControlerNode = MuxNode.Nodes.Add(ControlerID, "Controler : " & ControlerID & " Type : " & IIf(DeviceTypeID = Controlertype_t.Digital, "Digital", "Analogue"))
                    ControlerNode.ImageIndex = 1
                    ControlerNode.SelectedImageIndex = 1
                    For MotorID As Byte = 0 To 7
                        Dim MotorNode = ControlerNode.Nodes.Add(MotorID, "Haptic Point : " & MotorID)
                        MotorNode.Tag = New ControllerParams(MUXID, ControlerID, MotorID, DeviceTypeID)
                        MotorNode.ImageIndex = 2
                        MotorNode.SelectedImageIndex = 2
                    Next
                End If

            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub
    Private Sub UpdateResults(item As String)
        Try
            If (txtIncommingPackets.InvokeRequired) Then
                txtIncommingPackets.Invoke(Sub() UpdateResults(item))
            Else
                txtIncommingPackets.AppendText(item & vbCrLf)
            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub

    Private Sub ClearTree()
        Try
            If (ControlerTree.InvokeRequired) Then
                ControlerTree.Invoke(Sub() ClearTree())
            Else
                ControlerTree.Nodes.Clear()
            End If
            Devices.Clear()
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub



    Private Sub SendHapticLevel(Mux As Byte, Controller As Byte, Motor As Byte, Level As Byte)
        Try
            SendServerResponseToken({PACKET_OCTOSLIME_HAPTICS_SET, Mux, Controller, Motor, Level})
            UpdateResults("[Sending] Node : " & Mux & " Device : " & Controller & " Motor : " & Motor & " Value : " & Level)
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub
    Private Sub ControlerTree_AfterSelect(sender As Object, e As TreeViewEventArgs) Handles ControlerTree.AfterSelect
        Try
            If e.Node.Tag IsNot Nothing Then
                ClickedControllerParams = e.Node.Tag
                pnlAnalogue.Visible = False
                pnlDigital.Visible = False
                Select Case ClickedControllerParams.ControlerTypeID

                    Case Controlertype_t.Digital
                        pnlDigital.Visible = True
                    Case Controlertype_t.Analogue
                        pnlAnalogue.Visible = True

                End Select
                cmdSend.Enabled = True
                lblDevice.Text = "Node : " & ClickedControllerParams.MUXID & "    Device : " & ClickedControllerParams.ControlerID & "    Motor : " & ClickedControllerParams.MotorID

            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub

#End Region

#Region "Form Events"

    Private Sub Button2_Click(sender As Object, e As EventArgs) Handles cmdStart.Click

        Try
            If Not ServersRunning Then
                ServersRunning = True
                cmdStart.Enabled = False
                cmdStop.Enabled = True
                StartThreads()
                UpdateResults("[Action] Server Started")
                SetStateColour(Me, Color.DarkGreen)
            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub
    Private Sub Button3_Click(sender As Object, e As EventArgs) Handles cmdClearHistory.Click
        Try
            txtIncommingPackets.Text = String.Empty
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub
    Private Sub Button4_Click(sender As Object, e As EventArgs) Handles cmdStop.Click
        Try
            If ServersRunning Then
                ClearTree()
                ServersRunning = False
                cmdStart.Enabled = True
                cmdStop.Enabled = False
                UpdateResults("[Action] Server Stopped")
                SetStateColour(Me, Color.DarkRed)
            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub
    Private Sub cmdSend_Click(sender As Object, e As EventArgs) Handles cmdSend.Click
        Try
            If ServersRunning Then

                Dim Value As Byte
                If ClickedControllerParams.ControlerTypeID = Controlertype_t.Analogue Then
                    Value = analogueValue.Value
                Else
                    Value = IIf(digitalValue.SelectedItem = "On", 1, 0)
                End If

                SendHapticLevel(ClickedControllerParams.MUXID, ClickedControllerParams.ControlerID, ClickedControllerParams.MotorID, Value)

            End If
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub
    Private Sub Form1_Closed(sender As Object, e As EventArgs) Handles Me.Closed
        Try
            ServersRunning = False
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try

    End Sub

    Private Sub Menu_Item_Select_Osc_Folder_Click(sender As Object, e As EventArgs) Handles Menu_Item_Select_Osc_Folder.Click
        Try
            MsgBox("Looking for the folder: C:\Users\<Your name>\AppData\LocalLow\VRChat\VRChat\OSC")
            Octo_Haptics_OSC_File_Location_Finder.SelectedPath = AppDatafolder
            Octo_Haptics_OSC_File_Location_Finder.ShowDialog()
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub

    Private Sub Menu_Item_Start_Server_Click(sender As Object, e As EventArgs) Handles Menu_Item_Start_Server.Click
        Try
            cmdStart.PerformClick()
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub

    Private Sub Menu_Item_Stop_Server_Click(sender As Object, e As EventArgs) Handles Menu_Item_Stop_Server.Click
        Try
            cmdStart.PerformClick()
        Catch ex As Exception
            UpdateResults("[Error] " & ex.Message)
        End Try
    End Sub

#End Region


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

    Private Sub CloseServerToolStripMenuItem_Click(sender As Object, e As EventArgs) Handles CloseServerToolStripMenuItem.Click
        Me.Close()
    End Sub
End Class

Public Class ControllerParams

    Private MyMuxID As Byte
    Private MyControlerID As Byte
    Private MyMotorID As Byte
    Private MyControlerTypeID As Byte

    Public Sub New(MuxID As Byte, ControlerID As Byte, MotorID As Byte, ControlerTypeID As Byte)
        MyMuxID = MuxID
        MyControlerID = ControlerID
        MyMotorID = MotorID
        MyControlerTypeID = ControlerTypeID
    End Sub

    Public ReadOnly Property MUXID As Byte
        Get
            Return MyMuxID
        End Get
    End Property

    Public ReadOnly Property ControlerID As Byte
        Get
            Return MyControlerID
        End Get
    End Property
    Public ReadOnly Property MotorID As Byte
        Get
            Return MyMotorID
        End Get
    End Property
    Public ReadOnly Property ControlerTypeID As Byte
        Get
            Return MyControlerTypeID
        End Get
    End Property

End Class
