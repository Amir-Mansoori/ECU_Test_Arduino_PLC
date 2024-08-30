{attribute 'qualified_only'}
VAR_GLOBAL
	//w_CAN_SEND: ARRAY[0..23] OF BYTE;
	//AUX IO
	x_PEDAL_REAL: BOOL;
	x_PEDAL_TYPE: BOOL;

	x_DOG12 : BOOL	;//	J1.E1	Red Changing Light
	x_DOG01	: BOOL	;//	J1.E2	Contactor Supply Control Packs 4-6
	x_DOG02	: BOOL	;//	J1.E3	Contactor Supply Control Packs 7-9
	x_DOG03	: BOOL	;//	J1.E4	Start/Stop Orange LED
	x_DOG04	: BOOL	;//	J1.F1	Start/Stop Green LED
	x_DOG05	: BOOL	;//	J1.F2	Green Charging Light
	x_DOG06	: BOOL	;//	J1.F3 	Contactor Supply Control Packs 1-3
	x_DOG07	: BOOL	;//	J1.F4 	Blue Charging Light
	x_DOG08	: BOOL	;//	J1.G1 	TGC Off Command
	x_DOG09	: BOOL	;//	J1.G2 	TGC On Command
	x_DOG10	: BOOL	;//	J1.G3	LV ePTO Relay Coil control
	x_DOG11	: BOOL	;//	J1.G4	Differential Lock Solenoid LS
	
	x_DOV01	: BOOL	;//	J1.L1	VCM Wake 1 
	x_DOV02	: BOOL	;//	J1.L2	VCM Wake 2
	x_DOV03	: BOOL	;//	J1.L3	VCM Wake 3	
	x_DOV04	: BOOL	;//	J1.L4	Governor Valve
	x_DOV05	: BOOL	;//	J1.M1	Regeneration Valve
	x_DOV06	: BOOL	;//	J1.M2	VCM wake 4
	x_DOV07	: BOOL	;//	J1.M3	Auxiliary HVIL Out
	x_DOV08	: BOOL	;//	J1.M4	HV Battery 1-9 Wake
	
	
	x_DHLC01 : BOOL	;// J3.21 Cabin Tilt Status [Digital HS-LS Control 1] 
	x_DFC01	 : BOOL; // J3.21 Cabin Tilt Status [Digital Float Control 1]
	x_DHLC02 : BOOL	;// J3.22 Drive Req Signal 1 [Digital HS-LS Control 2] 
	x_DFC02	 : BOOL; // J3.22 Drive Req Signal 1 [Digital Float Control 2]
	x_DHLC03 : BOOL	;// J3.33 Drive Req Signal 2 [Digital HS-LS Control 3] 
	x_DFC03	 : BOOL; // J3.33 Drive Req Signal 2 [Digital Float Control 3]
	x_DHLC04 : BOOL	;// J3.34 Neutral Req Signal [Digital HS-LS Control 4] 
	x_DFC04	 : BOOL; // J3.34 Neutral Req Signal [Digital Float Control 4]	
	x_DHLC05 : BOOL	;// J3.45 Reverse Req Signal 1 [Digital HS-LS Control 5] 
	x_DFC05	 : BOOL; // J3.45 Reverse Req Signal 1 [Digital Float Control 5]
	x_DHLC06 : BOOL	;// J3.46 Reverse Req Signal 2 [Digital HS-LS Control 6] 
	x_DFC06	 : BOOL ; // J3.46 Reverse Req Signal 2 [Digital Float Control 6]

	x_CF01	 : BOOL ; // J3.23 Start/Stop Signal 2 [Control Float 1]
	x_CF02	 : BOOL ; // J3.24 Start/Stop Signal 1 [Control Float 2]
	x_CF03	 : BOOL ; // J3.25 APU Sensor Front Signal [Control Float 3]
	x_CF04	 : BOOL ; // J3.26 APU Sensor Rear Signal [Control Float 4]
	x_CF05	 : BOOL ; // J3.27 TGC Off Request [Control Float 5]
	x_CF06	 : BOOL ; // J3.28 TGC On Request [Control Float 6]
	x_CF07	 : BOOL ; // J3.35 FLEX-IN6 [Control Float 7]
	x_CF08	 : BOOL ; // J3.36 FLEX-IN7 [Control Float 8]
	x_CF09	 : BOOL ; // J3.37 FLEX-IN8 [Control Float 9]
	x_CF10	 : BOOL ; // J3.38 Positive Auxiliary Contactor Feedback [Control Float 10]
	x_CF11	 : BOOL ; // J3.39 Negative Auxiliary Contactor Feedback [Control Float 11]
	x_CF12	 : BOOL ; // J3.40 FLEX-IN11 [Control Float 12]
	x_CF13	 : BOOL ; // J3.47 HW E-Stop Signal [Control Float 13]
	x_CF14	 : BOOL ; // J3.48 TPMS Power [Control Float 14]
	x_CF15	 : BOOL ; // J3.49  Differential Lock Sensor Signal [Control Float 15]
	x_CF16	 : BOOL ; // J3.50  Charge Door Position [Control Float 16]	

	x_CA01	 : BOOL ; // J3.23 Start/Stop Signal 2 [Control Analog 1]
	x_CA02	 : BOOL ; // J3.24 Start/Stop Signal 1 [Control Analog 2]
	x_CA03	 : BOOL ; // J3.25 APU Sensor Front Signal [Control Analog 3]
	x_CA04	 : BOOL ; // J3.26 APU Sensor Rear Signal [Control Analog 4]
	x_CA05	 : BOOL ; // J3.27 TGC Off Request [Control Analog 5]
	x_CA06	 : BOOL ; // J3.28 TGC On Request [Control Analog 6]
	x_CA07	 : BOOL ; // J3.35 FLEX-IN6 [Control Analog 7]
	x_CA08	 : BOOL ; // J3.36 FLEX-IN7 [Control Analog 8]
	x_CA09	 : BOOL ; // J3.37 FLEX-IN8 [Control Analog 9]
	x_CA10	 : BOOL ; // J3.38 Positive Auxiliary Contactor Feedback [Control Analog 10]
	x_CA11	 : BOOL ; // J3.39 Negative Auxiliary Contactor Feedback [Control Analog 11]
	x_CA12	 : BOOL ; // J3.40 FLEX-IN11 [Control Analog 12]
	x_CA13	 : BOOL ; // J3.47 HW E-Stop Signal [Control Analog 13]
	x_CA14	 : BOOL ; // J3.48 TPMS Power [Control Analog 14]
	x_CA15	 : BOOL ; // J3.49  Differential Lock Sensor Signal [Control Analog 15]
	x_CA16	 : BOOL ; // J3.50  Charge Door Position [Control Analog 16]	


	w_FlexIN0	 : WORD ; // J3.23 Start/Stop Signal 2 [FlexIN0]
	w_FlexIN1	 : WORD ; // J3.24 Start/Stop Signal 1 [FlexIN1]
	w_FlexIN2	 : WORD ; // J3.25 APU Sensor Front Signal [FlexIN2]
	w_FlexIN3	 : WORD ; // J3.26 APU Sensor Rear Signal [FlexIN3]
	w_FlexIN4	 : WORD ; // J3.27 TGC Off Request [FlexIN4]
	w_FlexIN5	 : WORD ; // J3.28 TGC On Request [FlexIN5]
	w_FlexIN6	 : WORD ; // J3.35 FLEX-IN6 [FlexIN6]
	w_FlexIN7	 : WORD ; // J3.36 FLEX-IN7 [FlexIN7]
	w_FlexIN8	 : WORD ; // J3.37 FLEX-IN8 [FlexIN8]
	w_FlexIN9	 : WORD ; // J3.38 Positive Auxiliary Contactor Feedback [FlexIN9]
	w_FlexIN10	 : WORD ; // J3.39 Negative Auxiliary Contactor Feedback [FlexIN10]
	w_FlexIN11	 : WORD ; // J3.40 FLEX-IN11 [FlexIN11]
	w_FlexIN12	 : WORD ; // J3.47 HW E-Stop Signal [FlexIN12]
	w_FlexIN13	 : WORD ; // J3.48 TPMS Power [FlexIN13]
	w_FlexIN14	 : WORD ; // J3.49  Differential Lock Sensor Signal [FlexIN14]
	w_FlexIN15	 : WORD ; // J3.50  Charge Door Position [FlexIN15]

	w_AccPedalSig1			: WORD 	;	//J3.8 AccPedalSig1
	w_AccPedalSig2			: WORD  ;	//J3.9 AccPedalSig2
	w_ANC2					: WORD  ;	//J3.10 ANC2
	w_OilLevelSig			: WORD  ;	//J3.11 OilLevelSig
	w_Cabin_Sw_E_Stop_Sig	: WORD  ;	//J3.12 Cabin_Sw_E_Stop_Sig
	w_ANC5					: WORD  ;	//J3.13 ANC5
	
	
	b_PWM1_Control_Byte_0	: BYTE ;   // J3.19-20 PWM [Control Byte 0]
	w_PWM1_DO1				: WORD ;   // J3.19 Speed Pulse PWM [DO1]
	w_PWM1_DO2				: WORD ;   // J3.20 Distance Signal [DO2]
	b_PWM1_Control_Byte_2   : BYTE ;  // J3.31-32 PWM [Control Byte 2]
	w_PWM1_DO3				: WORD ;   // J3.31 FLEX_PWM2_NOT_USED [DO3]
	w_PWM1_DO4				: WORD ;   // J3.32 PYRO Crash Event Signal [DO4]
	
	b_PWM2_Control_Byte_0	: BYTE ;   // J3.43-44 PWM [Control Byte 0]
	w_PWM2_DO1				: WORD ;   // J3.43 FLEX_PWM4 [DO1]
	w_PWM2_DO2				: WORD ;   // J3.44 FLEX_PWM5 [DO2]	
	


	
//	x_DOV09	: BOOL	;//	X2.28	b
//	x_CrankHSS	: BOOL	;//	X2.36	Cranking Relay HSS	
//	x_DOV15	: BOOL	;//	X2.06	Single H out2
//	x_DOV16	: BOOL	;//	X2.04	Single H out1
	//LIN1-GND	: BOOL	;//	X1.05	LIN 1 GND
	x_TRX_P	: BOOL	;//	X1.07	Not Used
	x_TRX_M	: BOOL	;//	X1.08	Not Used
	//x_DIG_ACC_GND: BOOL	;//	X1.09	Accelerator pedal low idle switch
	//x_DIG16	: BOOL	;//	X1.10	PTO 3
	//x_DIG15	: BOOL	;//	X1.11	PTO 2
	//x_DIG14	: BOOL	;//	X1.12	PTO 1
	x_DIB15	: BOOL	;//	X1.13	Hill holder
	x_DIB15H	: BOOL	;//	X1.13	Hill holder
	//LIN2-GND	: BOOL	;//	X1.14	optional
	x_ASG02	: BOOL	;//	X1.15	Accelerator Pedal channel2 Ground
	//CAN6-H	: BOOL	;//	X1.16	Not Used
	//CAN6-L	: BOOL	;//	X1.17	Not Used
	x_ASG01	: BOOL	;//	X1.18	Accelerator Pedal channel1 Ground
	//CAN5-H	: BOOL	;//	X1.19	Body Computer Bus - CAN H
	//CAN5-L	: BOOL	;//	X1.20	Body Computer Bus - CAN L
	w_AI01	: WORD	;//	X1.21	Accelerator Pedal channel 1
	w_AI02	: WORD	;//	X1.22	Accelerator Pedal channel 2
	x_ACC_SW_GND: BOOL	;//	X1.23	Accelerator  pedal low idle switch Ground
	
	
	x_ASV01	: BOOL	;//	X1.26	Accelerator Pedal channel 1 Supply 
	x_ASV02	: BOOL	;//	X1.27	Accelerator Pedal channel 2 Supply 
	x_AWWS_B_In	: BOOL	;//	X1.28	optional
	x_AWWS_B_Out: BOOL	;//	X1.29	optional
	//LIN1-SIG	: BOOL	;//	X1.31	LIN 1 Bus
	//LIN2-SIG	: BOOL	;//	X1.32	optional
	//LIN1-SUP	: BOOL	;//	X1.33	LIN 1 supply
	//LIN2-SUP	: BOOL	;//	X1.34	optional
	x_AWWS_A_In	: BOOL	;//	X1.35	optional
	x_AWWS_A_Out: BOOL	;//	X1.36	optional
		
	x_TRM_30: BOOL	;//	X2.01	Power Supply
	//CAN9-H	: BOOL	;//	X2.02	optional
	//CAN9-L	: BOOL	;//	X2.03	optional

	x_AI03	: BOOL	;//	X2.07	Analog Input 3
	//x_DIG07	: BOOL	;//	X2.08	Gear High/Low
	x_TRM_15: BOOL	;//	X2.09	Ignition Key
	//CAN1-H	: BOOL	;//	X2.10	Vehicle Data Bus CAN H
	//CAN1-L	: BOOL	;//	X2.11	Vehicle Data Bus CAN L
	//CAN2-H	: BOOL	;//	X2.12	Engine Contro Bus - CAN H
	//CAN2-L	: BOOL	;//	X2.13	Engine Contro Bus - CAN L
	//CAN3-H	: BOOL	;//	X2.14	Expansion Module Bus - CAN H
	//CAN3-L	: BOOL	;//	X2.15	Expansion Module Bus - CAN L
	//CAN4-H	: BOOL	;//	X2.16	ADAS Data Bus CAN FD - CAN H 
	//CAN4-L	: BOOL	;//	X2.17	ADAS Data Bus CAN FD - CAN L
	x_TRM_31: BOOL	;//	X2.19	Power Supply GND
	x_DIV04	: BOOL	;//	X2.20	Alcohol Interlock
	//x_DIG12	: BOOL	;//	X2.21	BB Start
	//x_DIG13	: BOOL	;//	X2.22	BB Stop
	//cAN7-H	: BOOL	;//	X2.23	optional
	//CAN7-L	: BOOL	;//	X2.24	optional
	//CAN8-H	: BOOL	;//	X2.25	optional
	//CAN8-L	: BOOL	;//	X2.26	optional

	x_DIB16	: BOOL	;//	X2.30	Halt brake
	x_DIB16H: BOOL	;//	X2.30	Halt brake
	x_DIB17	: BOOL	;//	X2.31	ESP / ASR
	x_DIB17H: BOOL	;//	X2.31	ESP / ASR

	//x_DIG19	: BOOL	;//	X2.34	Clima 22
	//x_DIG18	: BOOL	;//	X2.35	Clima 18

	
// 	x_DIG01	: BOOL	;//	X3.01	CC Set+
// 	x_DIG02	: BOOL	;//	X3.02	CC Set-
// 	x_DIG03	: BOOL	;//	X3.03	CC Resume
// 	x_DIG04	: BOOL	;//	X3.04	CC Off
	x_DIB05	: BOOL	;//	X3.05	CC GAP-
	x_DIB05H: BOOL	;//	X3.05	CC GAP-
	x_DIB06	: BOOL	;//	X3.06	CC GAP +
	x_DIB06H: BOOL	;//	X3.06	CC GAP +
// 	x_DIG05	: BOOL	;//	X3.07	Ext/Int CC
// 	x_DIG20	: BOOL	;//	X3.08	D/Dm1
// 	x_DIG24	: BOOL	;//	X3.09	R/Rm1
// 	x_DIG22	: BOOL	;//	X3.10	N
// 	x_DIG21	: BOOL	;//	X3.11	D/Dm2
// 	x_DIG25	: BOOL	;//	X3.12	R/Rm2
	x_DIB12	: BOOL	;//	X3.13	Gear Up
	x_DIB12H	: BOOL	;//	X3.13	Gear Up
	x_DIB13	: BOOL	;//	X3.14	Gear Down
	x_DIB13H	: BOOL	;//	X3.14	Gear Down
	x_DIB04	: BOOL	;//	X3.15	FPSL  
	x_DIB04H	: BOOL	;//	X3.15	FPSL  
	x_DIB02	: BOOL	;//	X3.16	ECO Roll Swt
	x_DIB02H	: BOOL	;//	X3.16	ECO Roll Swt
	x_DIV03	: BOOL	;//	X3.17	Second SL Switch
	x_DIV02	: BOOL	;//	X3.18	Sec. Brake
	x_DIV01	: BOOL	;//	X3.19	Prim. Brake
	x_DIB11	: BOOL	;//	X3.20	ACC
	x_DIB11H	: BOOL	;//	X3.20	ACC
	x_DIB08	: BOOL	;//	X3.21	PECC
	x_DIB08H	: BOOL	;//	X3.21	PECC
	x_DIB01	: BOOL	;//	X3.22	Cranck request (K50)
	x_DIB01H	: BOOL	;//	X3.22	Cranck request (K50)
	x_DIB14	: BOOL	;//	X3.23	Parking Brake Switch
	x_DIB14H	: BOOL	;//	X3.23	Parking Brake Switch
	x_DIG11	: BOOL	;//	X3.24	Cabin Tilt
	x_DIG10	: BOOL	;//	X3.25	Neutral
	x_DIG06	: BOOL	;//	X3.26	Clutch 
	//x_DIG17	: BOOL	;//	X3.27	Clutch 2nd
	x_DIB10	: BOOL	;//	X3.28	ABS OFF
	x_DIB10H	: BOOL	;//	X3.28	ABS OFF
	//x_DIG09	: BOOL	;//	X3.29	EB Mode 1
	x_DIB07	: BOOL	;//	X3.30	EB request
	x_DIB07H	: BOOL	;//	X3.30	EB request
	//x_DIG27	: BOOL	;//	X3.31	CSF - LDWS
	//x_DIG23	: BOOL	;//	X3.32	Heavy Offroad
	x_DIB03	: BOOL	;//	X3.33	Rocking Mode
	x_DIB03H	: BOOL	;//	X3.33	Rocking Mode
	x_DIB09	: BOOL	;//	X3.34	 Power mode
	x_DIB09H	: BOOL	;//	X3.34	 Power mode
	//x_DIG26	: BOOL	;//	X3.35	Silent Mode
	//x_DIG08	: BOOL	;//	X3.36	ECO Mode
END_VAR