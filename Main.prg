Global String ReadData$
Global String data$
Global String Camera_X$, Camera_Y$, Camera_Z$, Camera_U$, Camera_V$, Camera_W$

Function main
	Tool 5
	Call init
	Call TcpClient
	Do
		Call ReceiveData
		Go XY(Val(Camera_X$), Val(Camera_Y$), Val(Camera_Z$), Val(Camera_U$), Val(Camera_V$), Val(Camera_W$))
		Go 工位1
		Go afaf
		Go 仓库位置1
		Wait 5
		VesselOn
		Go afaf
		Go 工位1
		VesselOff
	Loop
	
Fend
Function VesselOn

	On 11

Fend
Function VesselOff
	
	Off 11

Fend
Function LightOn
	
	On 13

Fend
Function LightOff
	
	Off 13
	
Fend
Function TcpClient
	SetNet #203, "192.168.0.20", 65056, CRLF, NONE, 0
	OpenNet #203 As Client
	WaitNet #203
	Print #203, "275, 600, 460, 45, 0.7, 178"
	Input #203, data$
	Print data$
	Print "203 port connected successfully!"

Fend
Function init
	Motor On
	Power Low
	Speed 40
	Tool 5
	LightOn
	VesselOff
	P00 = XY(386.061, 637.934, 479.113, 86.835, 48.696, -93.537)
	Go P00
	
Fend
Function ReceiveData
	Integer RecNumber
	RecNumber = ChkNet(203)
	Print "等待连接.."
	Print "接受数据数：", RecNumber
	If RecNumber >= 0 Then
		Input #203, Camera_X$, Camera_Y$, Camera_Z$, Camera_U$, Camera_V$, Camera_W$
        Print "相机发送的坐标系:", " X的值:", Camera_X$, " Y的值:", Camera_Y$, " Z的值:", Camera_Z$, " U的值:", Camera_U$, " V的值:", Camera_V$, " W的值:", Camera_W$
    Else
    	CloseNet #203
    	OpenNet #203 As Client
    	Print "等待连接..."
    	WaitNet #203
	EndIf
Fend

