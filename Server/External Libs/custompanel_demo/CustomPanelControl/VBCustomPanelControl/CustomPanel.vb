<System.Drawing.ToolboxBitmapAttribute(GetType(System.Windows.Forms.Panel))> _
 Public Class CustomPanel : Inherits System.Windows.Forms.Panel

    ' Fields
    Private _BackColour1 As System.Drawing.Color = System.Drawing.SystemColors.Window
    Private _BackColour2 As System.Drawing.Color = System.Drawing.SystemColors.Window
    Private _GradientMode As LinearGradientMode = LinearGradientMode.None
    Private _BorderStyle As System.Windows.Forms.BorderStyle = System.Windows.Forms.BorderStyle.None
    Private _BorderColour As System.Drawing.Color = System.Drawing.SystemColors.WindowFrame
    Private _BorderWidth As Integer = 1
    Private _Curvature As Integer = 0
    Private _CurveMode As CornerCurveMode = CornerCurveMode.All

    ' Properties

    '   Shadow the Backcolor property so that the base class will still render with a transparent backcolor
    <System.ComponentModel.DefaultValueAttribute(GetType(System.Drawing.Color), "Window"), _
     System.ComponentModel.CategoryAttribute("Appearance"), _
     System.ComponentModel.DescriptionAttribute("The primary background color used to display text and graphics in the control.")> _
    Public Shadows Property BackColor() As System.Drawing.Color
        Get
            Return Me._BackColour1
        End Get
        Set(ByVal Value As System.Drawing.Color)
            Me._BackColour1 = Value
            If Me.DesignMode = True Then
                Me.Invalidate()
            End If
        End Set
    End Property

    <System.ComponentModel.DefaultValueAttribute(GetType(System.Drawing.Color), "Window"), _
    System.ComponentModel.CategoryAttribute("Appearance"), _
    System.ComponentModel.DescriptionAttribute("The secondary background color used to paint the control.")> _
   Public Property BackColor2() As System.Drawing.Color
        Get
            Return Me._BackColour2
        End Get
        Set(ByVal Value As System.Drawing.Color)
            Me._BackColour2 = Value
            If Me.DesignMode = True Then
                Me.Invalidate()
            End If
        End Set
    End Property

    <System.ComponentModel.DefaultValueAttribute(GetType(LinearGradientMode), "None"), _
     System.ComponentModel.CategoryAttribute("Appearance"), _
     System.ComponentModel.DescriptionAttribute("The gradient direction used to paint the control.")> _
    Public Property GradientMode() As LinearGradientMode
        Get
            Return Me._GradientMode
        End Get
        Set(ByVal Value As LinearGradientMode)
            Me._GradientMode = Value
            If Me.DesignMode = True Then
                Me.Invalidate()
            End If
        End Set
    End Property

    <System.ComponentModel.DefaultValueAttribute(GetType(System.Windows.Forms.BorderStyle), "None"), _
     System.ComponentModel.CategoryAttribute("Appearance"), _
     System.ComponentModel.DescriptionAttribute("The border style used to paint the control.")> _
     Public Shadows Property BorderStyle() As System.Windows.Forms.BorderStyle
        Get
            Return Me._BorderStyle
        End Get
        Set(ByVal Value As System.Windows.Forms.BorderStyle)
            Me._BorderStyle = Value
            If Me.DesignMode = True Then
                Me.Invalidate()
            End If
        End Set
    End Property

    <System.ComponentModel.DefaultValueAttribute(GetType(System.Drawing.Color), "WindowFrame"), _
         System.ComponentModel.CategoryAttribute("Appearance"), _
         System.ComponentModel.DescriptionAttribute("The border color used to paint the control.")> _
        Public Property BorderColor() As System.Drawing.Color
        Get
            Return Me._BorderColour
        End Get
        Set(ByVal Value As System.Drawing.Color)
            Me._BorderColour = Value
            If Me.DesignMode = True Then
                Me.Invalidate()
            End If
        End Set
    End Property

    <System.ComponentModel.DefaultValueAttribute(GetType(Integer), "1"), _
     System.ComponentModel.CategoryAttribute("Appearance"), _
     System.ComponentModel.DescriptionAttribute("The width of the border used to paint the control.")> _
    Public Property BorderWidth() As Integer
        Get
            Return Me._BorderWidth
        End Get
        Set(ByVal Value As Integer)
            Me._BorderWidth = Value
            If Me.DesignMode = True Then
                Me.Invalidate()
            End If
        End Set
    End Property

    <System.ComponentModel.DefaultValueAttribute(GetType(Integer), "0"), _
     System.ComponentModel.CategoryAttribute("Appearance"), _
     System.ComponentModel.DescriptionAttribute("The radius of the curve used to paint the corners of the control.")> _
    Public Property Curvature() As Integer
        Get
            Return Me._Curvature
        End Get
        Set(ByVal Value As Integer)
            Me._Curvature = Value
            If Me.DesignMode = True Then
                Me.Invalidate()
            End If
        End Set
    End Property

    <System.ComponentModel.DefaultValueAttribute(GetType(CornerCurveMode), "All"), _
     System.ComponentModel.CategoryAttribute("Appearance"), _
     System.ComponentModel.DescriptionAttribute("The style of the curves to be drawn on the control.")> _
    Public Property CurveMode() As CornerCurveMode
        Get
            Return Me._CurveMode
        End Get
        Set(ByVal Value As CornerCurveMode)
            Me._CurveMode = Value
            If Me.DesignMode = True Then
                Me.Invalidate()
            End If
        End Set
    End Property

    Private ReadOnly Property adjustedCurve() As Integer
        Get
            Dim curve As Integer = 0
            If Not Me._CurveMode = CornerCurveMode.None Then
                If Me._Curvature > (Me.ClientRectangle.Width / 2) Then
                    curve = DoubleToInt(Me.ClientRectangle.Width / 2)
                Else
                    curve = Me._Curvature
                End If
                If curve > (Me.ClientRectangle.Height / 2) Then
                    curve = DoubleToInt(Me.ClientRectangle.Height / 2)
                End If
            End If
            Return curve
        End Get
    End Property

    Public Sub New()

        MyBase.New()
        Me.SetDefaultControlStyles()
        Me.customInitialisation()

    End Sub

    Private Sub SetDefaultControlStyles()

        Me.SetStyle(System.Windows.Forms.ControlStyles.DoubleBuffer, True)
        Me.SetStyle(System.Windows.Forms.ControlStyles.AllPaintingInWmPaint, False)
        Me.SetStyle(System.Windows.Forms.ControlStyles.ResizeRedraw, True)
        Me.SetStyle(System.Windows.Forms.ControlStyles.UserPaint, True)
        Me.SetStyle(System.Windows.Forms.ControlStyles.SupportsTransparentBackColor, True)

    End Sub

    Private Sub customInitialisation()

        Me.SuspendLayout()

        MyBase.BackColor = System.Drawing.Color.Transparent
        Me.BorderStyle = System.Windows.Forms.BorderStyle.None
        Me.ResumeLayout(False)

    End Sub

    Protected Overrides Sub OnPaintBackGround(ByVal e As System.Windows.Forms.PaintEventArgs)

        MyBase.OnPaintBackground(e)

        e.Graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.AntiAlias

        Dim graphPath As System.Drawing.Drawing2D.GraphicsPath
        graphPath = Me.GetPath()

        '	Create Gradient Brush (Cannot be width or height 0)
        Dim filler As System.Drawing.Drawing2D.LinearGradientBrush
        Dim rect As System.Drawing.Rectangle = Me.ClientRectangle

        If Me.ClientRectangle.Width = 0 Then
            rect.Width += 1
        End If

        If Me.ClientRectangle.Height = 0 Then
            rect.Height += 1
        End If

        If Me._GradientMode = LinearGradientMode.None Then
            filler = New System.Drawing.Drawing2D.LinearGradientBrush(rect, Me._BackColour1, Me._BackColour1, System.Drawing.Drawing2D.LinearGradientMode.Vertical)
        Else
            filler = New System.Drawing.Drawing2D.LinearGradientBrush(rect, Me._BackColour1, Me._BackColour2, CType(Me._GradientMode, System.Drawing.Drawing2D.LinearGradientMode))
        End If

        e.Graphics.FillPath(filler, graphPath)
        filler.Dispose()

        Select Case Me._BorderStyle
            Case System.Windows.Forms.BorderStyle.FixedSingle
                Dim borderPen As New System.Drawing.Pen(Me._BorderColour, Me._BorderWidth)
                e.Graphics.DrawPath(borderPen, graphPath)
                borderPen.Dispose()

            Case System.Windows.Forms.BorderStyle.Fixed3D
                Me.DrawBorder3D(e.Graphics, Me.ClientRectangle)

            Case System.Windows.Forms.BorderStyle.None

        End Select

        filler.Dispose()
        graphPath.Dispose()


    End Sub

    Protected Function GetPath() As System.Drawing.Drawing2D.GraphicsPath

        Dim graphPath As New System.Drawing.Drawing2D.GraphicsPath

        If Me._BorderStyle = System.Windows.Forms.BorderStyle.Fixed3D Then
            graphPath.AddRectangle(Me.ClientRectangle)
        Else

            Try

                Dim curve As Integer = 0

                Dim rect As System.Drawing.Rectangle = Me.ClientRectangle
                Dim offset As Integer = 0

                Select Case Me._BorderStyle

                    Case System.Windows.Forms.BorderStyle.FixedSingle
                        If Me._BorderWidth > 1 Then
                            offset = DoubleToInt(Me.BorderWidth / 2)
                        End If
                        curve = Me.adjustedCurve

                    Case System.Windows.Forms.BorderStyle.Fixed3D

                    Case System.Windows.Forms.BorderStyle.None
                        curve = Me.adjustedCurve

                End Select

                If curve = 0 Then
                    graphPath.AddRectangle(rect.Inflate(rect, -offset, -offset))
                Else

                    Dim rectWidth As Integer = rect.Width - 1 - offset
                    Dim rectHeight As Integer = rect.Height - 1 - offset
                    Dim curveWidth As Integer = 1

                    If (Me._CurveMode And CornerCurveMode.TopRight) <> 0 Then
                        curveWidth = (curve * 2)
                    Else
                        curveWidth = 1
                    End If
                    graphPath.AddArc(rectWidth - curveWidth, offset, curveWidth, curveWidth, 270, 90)

                    If (Me._CurveMode And CornerCurveMode.BottomRight) <> 0 Then
                        curveWidth = (curve * 2)
                    Else
                        curveWidth = 1
                    End If
                    graphPath.AddArc(rectWidth - curveWidth, rectHeight - curveWidth, curveWidth, curveWidth, 0, 90)

                    If (Me._CurveMode And CornerCurveMode.BottomLeft) <> 0 Then
                        curveWidth = (curve * 2)
                    Else
                        curveWidth = 1
                    End If
                    graphPath.AddArc(offset, rectHeight - curveWidth, curveWidth, curveWidth, 90, 90)

                    If (Me._CurveMode And CornerCurveMode.TopLeft) <> 0 Then
                        curveWidth = (curve * 2)
                    Else
                        curveWidth = 1
                    End If
                    graphPath.AddArc(offset, offset, curveWidth, curveWidth, 180, 90)

                    graphPath.CloseFigure()

                End If

            Catch ex As System.Exception
                graphPath.AddRectangle(Me.ClientRectangle)
            End Try

        End If

        Return graphPath

    End Function

    Public Shared Sub DrawBorder3D(ByVal graphics As System.Drawing.Graphics, ByVal rectangle As System.Drawing.Rectangle)

        graphics.SmoothingMode = System.Drawing.Drawing2D.SmoothingMode.Default
        graphics.DrawLine(System.Drawing.SystemPens.ControlDark, rectangle.X, rectangle.Y, rectangle.Width - 1, rectangle.Y)
        graphics.DrawLine(System.Drawing.SystemPens.ControlDark, rectangle.X, rectangle.Y, rectangle.X, rectangle.Height - 1)
        graphics.DrawLine(System.Drawing.SystemPens.ControlDarkDark, rectangle.X + 1, rectangle.Y + 1, rectangle.Width - 1, rectangle.Y + 1)
        graphics.DrawLine(System.Drawing.SystemPens.ControlDarkDark, rectangle.X + 1, rectangle.Y + 1, rectangle.X + 1, rectangle.Height - 1)
        graphics.DrawLine(System.Drawing.SystemPens.ControlLight, rectangle.X + 1, rectangle.Height - 2, rectangle.Width - 2, rectangle.Height - 2)
        graphics.DrawLine(System.Drawing.SystemPens.ControlLight, rectangle.Width - 2, rectangle.Y + 1, rectangle.Width - 2, rectangle.Height - 2)
        graphics.DrawLine(System.Drawing.SystemPens.ControlLightLight, rectangle.X, rectangle.Height - 1, rectangle.Width - 1, rectangle.Height - 1)
        graphics.DrawLine(System.Drawing.SystemPens.ControlLightLight, rectangle.Width - 1, rectangle.Y, rectangle.Width - 1, rectangle.Height - 1)

    End Sub

    Public Shared Function DoubleToInt(ByVal value As Double) As Integer
        Return System.Decimal.ToInt32(System.Decimal.Floor(System.Decimal.Parse((value).ToString)))
    End Function

End Class