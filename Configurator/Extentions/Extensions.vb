Imports System.Runtime.CompilerServices
Public Module Extensions
    ''' <summary>
    ''' Extension method to run string as CMD command.
    ''' </summary>
    ''' <param name="command">[String] Command to run.</param>
    ''' <param name="ShowWindow">[Boolean](Default:False) Option to show CMD window.</param>
    ''' <param name="WaitForProcessComplete">[Boolean](Default:False) Option to wait for CMD process to complete before exiting sub.</param>
    ''' <param name="permanent">[Boolean](Default:False) Option to keep window visible after command has finished. Ignored if ShowWindow is False.</param>
    <Extension>
    Public Sub RunCMD(command As String, Optional ShowWindow As Boolean = False, Optional WaitForProcessComplete As Boolean = False, Optional permanent As Boolean = False)
        Dim p As Process = New Process()
        Dim pi As ProcessStartInfo = New ProcessStartInfo()
        pi.Arguments = " " & If(ShowWindow AndAlso permanent, "/K", "/C") & " " & command
        pi.FileName = "cmd.exe"
        pi.CreateNoWindow = Not ShowWindow
        If ShowWindow Then
            pi.WindowStyle = ProcessWindowStyle.Normal
        Else
            pi.WindowStyle = ProcessWindowStyle.Hidden
        End If
        p.StartInfo = pi
        p.Start()
        If WaitForProcessComplete Then Do Until p.HasExited : Loop
    End Sub
End Module

