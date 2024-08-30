PROGRAM PLC_OUTPUT
VAR
	//w_q_658_1		AT  %QB0	:   ARRAY[0..23] OF BYTE;
	x_q_1504_1_1	AT	%QX24.0	:	BOOL	;
	x_q_1504_1_2	AT	%QX24.1	:	BOOL	;
	x_q_1504_1_3	AT	%QX24.2	:	BOOL	;
	x_q_1504_1_4	AT	%QX24.3	:	BOOL	;
	x_q_1504_1_5	AT	%QX24.4	:	BOOL	;
	x_q_1504_1_6	AT	%QX24.5	:	BOOL	;
	x_q_1504_1_7	AT	%QX24.6	:	BOOL	;
	x_q_1504_1_8	AT	%QX24.7	:	BOOL	;
	x_q_1504_1_9	AT	%QX24.8	:	BOOL	;
	x_q_1504_1_10	AT	%QX24.9	:	BOOL	;
	x_q_1504_1_11	AT	%QX24.10:	BOOL	;
	x_q_1504_1_12	AT	%QX24.11:	BOOL	;
	x_q_1504_1_13	AT	%QX24.12:	BOOL	;
	x_q_1504_1_14	AT	%QX24.13:	BOOL	;
	x_q_1504_1_15	AT	%QX24.14:	BOOL	;
	x_q_1504_1_16	AT	%QX24.15:	BOOL	;

	x_q_1504_2_1	AT	%QX26.0	:	BOOL	;
	x_q_1504_2_2	AT	%QX26.1	:	BOOL	;
	x_q_1504_2_3	AT	%QX26.2	:	BOOL	;
	x_q_1504_2_4	AT	%QX26.3	:	BOOL	;
	x_q_1504_2_5	AT	%QX26.4	:	BOOL	;
	x_q_1504_2_6	AT	%QX26.5	:	BOOL	;
	x_q_1504_2_7	AT	%QX26.6	:	BOOL	;
	x_q_1504_2_8	AT	%QX26.7	:	BOOL	;
	x_q_1504_2_9	AT	%QX26.8	:	BOOL	;
	x_q_1504_2_10	AT	%QX26.9	:	BOOL	;
	x_q_1504_2_11	AT	%QX26.10:	BOOL	;
	x_q_1504_2_12	AT	%QX26.11:	BOOL	;
	x_q_1504_2_13	AT	%QX26.12:	BOOL	;
	x_q_1504_2_14	AT	%QX26.13:	BOOL	;
	x_q_1504_2_15	AT	%QX26.14:	BOOL	;
	x_q_1504_2_16	AT	%QX26.15:	BOOL	;

	x_q_1504_3_1	AT	%QX28.0	:	BOOL	;
	x_q_1504_3_2	AT	%QX28.1	:	BOOL	;
	x_q_1504_3_3	AT	%QX28.2	:	BOOL	;
	x_q_1504_3_4	AT	%QX28.3	:	BOOL	;
	x_q_1504_3_5	AT	%QX28.4	:	BOOL	;
	x_q_1504_3_6	AT	%QX28.5	:	BOOL	;
	x_q_1504_3_7	AT	%QX28.6	:	BOOL	;
	x_q_1504_3_8	AT	%QX28.7	:	BOOL	;
	x_q_1504_3_9	AT	%QX28.8	:	BOOL	;
	x_q_1504_3_10	AT	%QX28.9	:	BOOL	;
	x_q_1504_3_11	AT	%QX28.10:	BOOL	;
	x_q_1504_3_12	AT	%QX28.11:	BOOL	;
	x_q_1504_3_13	AT	%QX28.12:	BOOL	;
	x_q_1504_3_14	AT	%QX28.13:	BOOL	;
	x_q_1504_3_15	AT	%QX28.14:	BOOL	;
	x_q_1504_3_16	AT	%QX28.15:	BOOL	;
	
	w_q_597_1_1	AT	%QW15	:	WORD	;
	w_q_597_1_2	AT	%QW16	:	WORD	;
	w_q_597_1_3	AT	%QW17	:	WORD	;
	w_q_597_1_4	AT	%QW18	:	WORD	;
	w_q_597_1_5	AT	%QW19	:	WORD	;
	w_q_597_1_6	AT	%QW20	:	WORD	;
	w_q_597_1_7	AT	%QW21	:	WORD	;
	w_q_597_1_8	AT	%QW22	:	WORD	;	

	w_q_597_2_1	AT	%QW23	:	WORD	;
	w_q_597_2_2	AT	%QW24	:	WORD	;
	w_q_597_2_3	AT	%QW25	:	WORD	;
	w_q_597_2_4	AT	%QW26	:	WORD	;
	w_q_597_2_5	AT	%QW27	:	WORD	;
	w_q_597_2_6	AT	%QW28	:	WORD	;
	w_q_597_2_7	AT	%QW29	:	WORD	;
	w_q_597_2_8	AT	%QW30	:	WORD	;	

	w_q_597_3_1	AT	%QW31	:	WORD	;
	w_q_597_3_2	AT	%QW32	:	WORD	;
	w_q_597_3_3	AT	%QW33	:	WORD	;
	w_q_597_3_4	AT	%QW34	:	WORD	;
	w_q_597_3_5	AT	%QW35	:	WORD	;
	w_q_597_3_6	AT	%QW36	:	WORD	;

	b_q_677_1_1 AT  %QB78	:	BYTE	; //Control Byte 1
	w_q_677_1_2 AT  %QW40	:	WORD	; // DO1
	//b_q_677_1_3 AT  %QB82	:	BYTE	; //Control Byte 2
	w_q_677_1_4 AT  %QW42	:	WORD	; // DO2	
	b_q_677_1_5 AT  %QB86	:	BYTE	; //Control Byte 3
	//w_q_677_1_6 AT  %QW44	:	WORD	; // DO3	
	//b_q_677_1_7 AT  %QB90	:	BYTE	; //Control Byte 4
	w_q_677_1_8 AT  %QW46	:	WORD	; // DO4

	b_q_677_2_1 AT  %QB94	:	BYTE	; //Control Byte 1
	w_q_677_2_2 AT  %QW48	:	WORD	; // DO1
	//b_q_677_2_3 AT  %QB98	:	BYTE	; //Control Byte 2
	w_q_677_2_4 AT  %QW50	:	WORD	; // DO2	
	//b_q_677_2_5 AT  %QB102	:	BYTE	; //Control Byte 3
	//w_q_677_2_6 AT  %QW52	:	WORD	; // DO3	
	//b_q_677_2_7 AT  %QB106	:	BYTE	; //Control Byte 4
	//w_q_677_2_8 AT  %QW54	:	WORD	; // DO4		
	
// 	x_q_1505_1_1	AT	%QX30.0	:	BOOL	;
// 	x_q_1505_1_2	AT	%QX30.1	:	BOOL	;
// 	x_q_1505_1_3	AT	%QX30.2	:	BOOL	;
// 	x_q_1505_1_4	AT	%QX30.3	:	BOOL	;
// 	x_q_1505_1_5	AT	%QX30.4	:	BOOL	;
// 	x_q_1505_1_6	AT	%QX30.5	:	BOOL	;
// 	x_q_1505_1_7	AT	%QX30.6	:	BOOL	;
// 	x_q_1505_1_8	AT	%QX30.7	:	BOOL	;
// 	x_q_1505_1_9	AT	%QX30.8	:	BOOL	;
// 	x_q_1505_1_10	AT	%QX30.9	:	BOOL	;
// 	x_q_1505_1_11	AT	%QX30.10:	BOOL	;
// 	x_q_1505_1_12	AT	%QX30.11:	BOOL	;
// 	x_q_1505_1_13	AT	%QX30.12:	BOOL	;
// 	x_q_1505_1_14	AT	%QX30.13:	BOOL	;
// 	x_q_1505_1_15	AT	%QX30.14:	BOOL	;
// 	x_q_1505_1_16	AT	%QX30.15:	BOOL	;
// 	
// 	x_q_1505_2_1	AT	%QX32.0	:	BOOL	;
// 	x_q_1505_2_2	AT	%QX32.1	:	BOOL	;
// 	x_q_1505_2_3	AT	%QX32.2	:	BOOL	;
// 	x_q_1505_2_4	AT	%QX32.3	:	BOOL	;
// 	x_q_1505_2_5	AT	%QX32.4	:	BOOL	;
// 	x_q_1505_2_6	AT	%QX32.5	:	BOOL	;
// 	x_q_1505_2_7	AT	%QX32.6	:	BOOL	;
// 	x_q_1505_2_8	AT	%QX32.7	:	BOOL	;
// 	x_q_1505_2_9	AT	%QX32.8	:	BOOL	;
// 	x_q_1505_2_10	AT	%QX32.9	:	BOOL	;
// 	x_q_1505_2_11	AT	%QX32.10:	BOOL	;
// 	x_q_1505_2_12	AT	%QX32.11:	BOOL	;
// 	x_q_1505_2_13	AT	%QX32.12:	BOOL	;
// 	x_q_1505_2_14	AT	%QX32.13:	BOOL	;
// 	x_q_1505_2_15	AT	%QX32.14:	BOOL	;
// 	x_q_1505_2_16	AT	%QX32.15:	BOOL	;
// 
// 	w_q_550_1_1		AT %QW17: WORD;
// 	w_q_550_1_2		AT %QW18: WORD;
// 	//w_q_559_1_3		AT %QW17: WORD;
// 	//w_q_559_1_4		AT %QW17: WORD;
	
END_VAR



//w_q_658_1		:=	GVL_IO.w_CAN_SEND;

x_q_1504_1_1	:=	GVL_IO.x_DHLC01	;
x_q_1504_1_2	:=	GVL_IO.x_DFC01	;
x_q_1504_1_3	:=	GVL_IO.x_DHLC02	;
x_q_1504_1_4	:=	GVL_IO.x_DFC02;
x_q_1504_1_5	:=	GVL_IO.x_DHLC03	;
x_q_1504_1_6	:=	GVL_IO.x_DFC03;
x_q_1504_1_7	:=	GVL_IO.x_DHLC04;
x_q_1504_1_8	:=	GVL_IO.x_DFC04;
x_q_1504_1_9	:=	GVL_IO.x_DHLC05;
x_q_1504_1_10	:=	GVL_IO.x_DFC05	;
x_q_1504_1_11	:=	GVL_IO.x_DHLC06;
x_q_1504_1_12	:=	GVL_IO.x_DFC06	;
x_q_1504_1_13	:=	GVL_IO.x_DIB05;
x_q_1504_1_14	:=	GVL_IO.x_DIB05H	;
//x_q_1504_1_15	:=	GVL_IO.x_DIB06;
//x_q_1504_1_16	:=	GVL_IO.x_DIB06H	;

x_q_1504_2_1	:=	GVL_IO.x_CF01	;
x_q_1504_2_2	:=	GVL_IO.x_CF02	;
x_q_1504_2_3	:=	GVL_IO.x_CF03	;
x_q_1504_2_4	:=	GVL_IO.x_CF04	;
x_q_1504_2_5	:=	GVL_IO.x_CF05	;
x_q_1504_2_6	:=	GVL_IO.x_CF06	;
x_q_1504_2_7	:=	GVL_IO.x_CF07	;
x_q_1504_2_8	:=	GVL_IO.x_CF08	;
x_q_1504_2_9	:=	GVL_IO.x_CF09	;
x_q_1504_2_10	:=	GVL_IO.x_CF10	;
x_q_1504_2_11	:=	GVL_IO.x_CF11	;
x_q_1504_2_12	:=	GVL_IO.x_CF12	;
x_q_1504_2_13	:=	GVL_IO.x_CF13	;
x_q_1504_2_14	:=	GVL_IO.x_CF14	;
x_q_1504_2_15	:=	GVL_IO.x_CF15	;
x_q_1504_2_16	:=	GVL_IO.x_CF16	;
			
x_q_1504_3_1	:=	GVL_IO.x_CA01	;
x_q_1504_3_2	:=	GVL_IO.x_CA02	;
x_q_1504_3_3	:=	GVL_IO.x_CA03	;
x_q_1504_3_4	:=	GVL_IO.x_CA04	;
x_q_1504_3_5	:=	GVL_IO.x_CA05	;
x_q_1504_3_6	:=	GVL_IO.x_CA06	;
x_q_1504_3_7	:=	GVL_IO.x_CA07	;
x_q_1504_3_8	:=	GVL_IO.x_CA08	;
x_q_1504_3_9	:=	GVL_IO.x_CA09	;
x_q_1504_3_10	:=	GVL_IO.x_CA10	;
x_q_1504_3_11	:=	GVL_IO.x_CA11	;
x_q_1504_3_12	:=	GVL_IO.x_CA12	;
x_q_1504_3_13	:=	GVL_IO.x_CA13	;
x_q_1504_3_14	:=	GVL_IO.x_CA14	;//alco
x_q_1504_3_15	:=	GVL_IO.x_CA15	;
x_q_1504_3_16	:=	GVL_IO.x_CA16	;

w_q_597_1_1		:=	GVL_IO.w_FlexIN0 ;
w_q_597_1_2		:=	GVL_IO.w_FlexIN1 ;
w_q_597_1_3		:=	GVL_IO.w_FlexIN2 ;
w_q_597_1_4		:=	GVL_IO.w_FlexIN3 ;
w_q_597_1_5		:=	GVL_IO.w_FlexIN4 ;
w_q_597_1_6		:=	GVL_IO.w_FlexIN5 ;
w_q_597_1_7		:=	GVL_IO.w_FlexIN6 ;
w_q_597_1_8		:=	GVL_IO.w_FlexIN7 ;

w_q_597_2_1		:=	GVL_IO.w_FlexIN8 ;
w_q_597_2_2		:=	GVL_IO.w_FlexIN9 ;
w_q_597_2_3		:=	GVL_IO.w_FlexIN10 ;
w_q_597_2_4		:=	GVL_IO.w_FlexIN11 ;
w_q_597_2_5		:=	GVL_IO.w_FlexIN12 ;
w_q_597_2_6		:=	GVL_IO.w_FlexIN13 ;
w_q_597_2_7		:=	GVL_IO.w_FlexIN14 ;
w_q_597_2_8		:=	GVL_IO.w_FlexIN15 ;

w_q_597_3_1		:=	GVL_IO.w_AccPedalSig1 ;
w_q_597_3_2		:=	GVL_IO.w_AccPedalSig2 ;
w_q_597_3_3		:=	GVL_IO.w_ANC2 ;
w_q_597_3_4		:=	GVL_IO.w_OilLevelSig ;
w_q_597_3_5		:=	GVL_IO.w_Cabin_Sw_E_Stop_Sig ;
w_q_597_3_6		:=	GVL_IO.w_ANC5 ;

b_q_677_1_1		:=	GVL_IO.b_PWM1_Control_Byte_0	;
w_q_677_1_2		:=	GVL_IO.w_PWM1_DO1;
//b_q_677_1_3		:=	GVL_IO.b_PWM1_Control_Byte_1	;
w_q_677_1_4		:=	GVL_IO.w_PWM1_DO2 ;
b_q_677_1_5		:=	GVL_IO.b_PWM1_Control_Byte_2 ;
//w_q_677_1_6		:=	GVL_IO.w_PWM1_DO3 ;
//b_q_677_1_7		:=	GVL_IO.b_PWM1_Control_Byte_3
w_q_677_1_8		:=	GVL_IO.w_PWM1_DO4 ;

 b_q_677_2_1		:=	GVL_IO.b_PWM2_Control_Byte_0	;
 w_q_677_2_2		:=	GVL_IO.w_PWM2_DO1;
 w_q_677_2_4		:=	GVL_IO.w_PWM2_DO2;
			
// _q_1505_1_1	:=	GVL_IO.x_DIG14	;//PTO1
// x_q_1505_1_2	:=	GVL_IO.x_DIG15	;//PTO2
// x_q_1505_1_3	:=	GVL_IO.x_DIG16	;//PTO3
// x_q_1505_1_4	:=	GVL_IO.x_DIG13	;
// x_q_1505_1_5	:=	GVL_IO.x_DIG12	;
// x_q_1505_1_6	:=	GVL_IO.x_DIG01	;
// x_q_1505_1_7	:=	GVL_IO.x_DIG02	;
// x_q_1505_1_8	:=	GVL_IO.x_DIG03	;
// x_q_1505_1_9	:=	GVL_IO.x_DIG04	;
// x_q_1505_1_10	:=	GVL_IO.x_DIG05	;
// x_q_1505_1_11	:=	GVL_IO.x_DIG20	;
// x_q_1505_1_12	:=	GVL_IO.x_DIG24	;
// x_q_1505_1_13	:=	GVL_IO.x_DIG22	;
// x_q_1505_1_14	:=	GVL_IO.x_DIG21	;
// x_q_1505_1_15	:=	GVL_IO.x_DIG25	;
// x_q_1505_1_16	:=	GVL_IO.x_DIG11	;
// 			
// x_q_1505_2_1	:=	GVL_IO.x_DIG10	;
// x_q_1505_2_2	:=	GVL_IO.x_DIG06	;
// x_q_1505_2_3	:=	GVL_IO.x_DIG17	;
// x_q_1505_2_4	:=	GVL_IO.x_DIG09	;
// x_q_1505_2_5	:=	GVL_IO.x_DIG27	;
// x_q_1505_2_6	:=	GVL_IO.x_DIG23	;
// x_q_1505_2_7	:=	GVL_IO.x_DIG26	;
// x_q_1505_2_8	:=	GVL_IO.x_DIG08	;
// //x_q_1505_2_9	:=	GVL_IO.	;
// x_q_1505_2_10	:=	GVL_IO.x_DIG07	;
// x_q_1505_2_11	:=	GVL_IO.x_DIG19	;
// x_q_1505_2_12	:=	GVL_IO.x_DIG18	;
// //x_q_1505_2_13	:=	GVL_IO.	;
// //x_q_1505_2_14	:=	GVL_IO.	;
// //x_q_1505_2_15	:=	GVL_IO.	;
// //x_q_1505_2_16	:=	GVL_IO.	;
// 			
// w_q_550_1_1	:=	GVL_IO.w_AI01	;
// w_q_550_1_2	:=	GVL_IO.w_AI02	;
// //w_q_559_1_3	:=	GVL_IO.	;
// //w_q_559_1_4	:=	GVL_IO.	;