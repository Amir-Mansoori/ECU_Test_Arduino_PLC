FUNCTION_BLOCK FB_RXCanMessageElaboration
VAR_INPUT
	wRxId:WORD;
	bRxNBytes:BYTE;
	aRxBuffer: ARRAY [0..7] OF BYTE;	
END_VAR
VAR_OUTPUT
END_VAR
VAR
	_FbCanTx11BitFrame : FbCanTx11BitFrame;
	xCANTxTrigger : BOOL;
	lut1,lut2: LUT;
END_VAR


xCANTxTrigger := FALSE;//no answer required

CASE wRxId OF
// 	16#75C:
// 			//ON MESSAGE ACTION
// 			IF aRxBuffer[0] = 0 THEN
// 				GVL.DO01 := FALSE;
// 			ELSE
// 				GVL.DO01 := TRUE;
// 			END_IF
// 			xCANTxTrigger := TRUE;
	16#11://BANCO_VCM4_Command
			GVL_IO.x_PEDAL_TYPE := aRxBuffer[0].0;//Pedal_Type
			GVL_IO.x_TRM_30 := aRxBuffer[0].1;//Power_Supply
			GVL_IO.x_TRM_15 := aRxBuffer[0].2;//Ignition_Key
			GVL_IO.x_PEDAL_REAL	:= aRxBuffer[0].3;//Accelerator_Pedal
			
	16#12://BANCO_VCM4_ANALOG_Critical_ADC (597_3) ++
			//ON MESSAGE ACTION	
			GVL_IO.w_AccPedalSig1 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[1]),8) + BYTE_TO_WORD(aRxBuffer[0]))) * 0.001 * 32767) ); // Refer to the manual of 597 page 34
			GVL_IO.w_AccPedalSig2 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[3]),8) + BYTE_TO_WORD(aRxBuffer[2]))) * 0.001 * 32767) ); // Refer to the manual of 597 page 34
			GVL_IO.w_OilLevelSig := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[5]),8) + BYTE_TO_WORD(aRxBuffer[4]))) * 0.001 * 32767) ); // Refer to the manual of 597 page 34
			GVL_IO.w_Cabin_Sw_E_Stop_Sig := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[7]),8) + BYTE_TO_WORD(aRxBuffer[6]))) * 0.001 * 32767) ); // Refer to the manual of 597 page 34					
				
	16#14://BANCO_VCM4_DI ++
			GVL_IO.x_DHLC01 := aRxBuffer[0].0; // Cabin_Tilt_Status_HS_LS
			GVL_IO.x_DFC01  := aRxBuffer[0].1; // Cabin_Tilt_Status_Float
			GVL_IO.x_DHLC02 := aRxBuffer[0].2; // Drive_Req_Signal_1_HS_LS
			GVL_IO.x_DFC02  := aRxBuffer[0].3; // Drive_Req_Signal_1_Float
			GVL_IO.x_DHLC03 := aRxBuffer[0].4; // Drive_Req_Signal_2_HS_LS
			GVL_IO.x_DFC03  := aRxBuffer[0].5; // Drive_Req_Signal_2_Float
			GVL_IO.x_DHLC04 := aRxBuffer[0].6; // Neutral_Req_Signal_HS_LS
			GVL_IO.x_DFC04  := aRxBuffer[0].7; // Neutral_Req_Signal_Float
			GVL_IO.x_DHLC05 := aRxBuffer[1].0; // Reverse_Req_Signal_1_HS_LS
			GVL_IO.x_DFC05  := aRxBuffer[1].1; // Reverse_Req_Signal_1_Float
			GVL_IO.x_DHLC06 := aRxBuffer[1].2; // Reverse_Req_Signal_2_HS_LS
			GVL_IO.x_DFC06  := aRxBuffer[1].3; // Reverse_Req_Signal_2_Float
			GVL_IO.x_DIB05  := aRxBuffer[1].4; // Power_Control
			GVL_IO.x_DIB05H := aRxBuffer[1].5; // K15-Ignition-Control			
			

			
// 	16#15://BANCO_VCM4_PWM_IN ++
// 			GVL_IO.b_PWM1_Control_Byte_0 :=  aRxBuffer[0];
// 			GVL_IO.b_PWM1_Control_Byte_2 :=  aRxBuffer[1];
// 			GVL_IO.w_PWM1_DO1 := REAL_TO_WORD((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[3]),8) + BYTE_TO_WORD(aRxBuffer[2]))));
// 			GVL_IO.w_PWM1_DO2 := REAL_TO_WORD((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[5]),8) + BYTE_TO_WORD(aRxBuffer[4]))));
// 			GVL_IO.w_PWM1_DO4 := REAL_TO_WORD((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[7]),8) + BYTE_TO_WORD(aRxBuffer[6]))));
			
	16#16://BANCO_VCM4_Analog_Control ++
			GVL_IO.x_CF01 := aRxBuffer[0].0;//Start_Stop_Signal_2_Float 0
			GVL_IO.x_CA01 := aRxBuffer[0].1;//Start_Stop_Signal_2_Analog 0
			GVL_IO.x_CF02 := aRxBuffer[0].2;//Start_Stop_Signal_1_Float 1
			GVL_IO.x_CA02 := aRxBuffer[0].3;//Start_Stop_Signal_1_Analog 1
			GVL_IO.x_CF03 := aRxBuffer[0].4;//APU_Sensor_Front_Signal_Float 2
			GVL_IO.x_CA03 := aRxBuffer[0].5;//APU_Sensor_Front_Signal_Analog 2
			GVL_IO.x_CF04 := aRxBuffer[0].6;//APU_Sensor_Rear_Signal_Float 3
			GVL_IO.x_CA04 := aRxBuffer[0].7;//APU_Sensor_Rear_Signal_Analog 3
			GVL_IO.x_CF05 := aRxBuffer[1].0;//TGC_Off_Request_Float 4
			GVL_IO.x_CA05 := aRxBuffer[1].1;//TGC_Off_Request_Analog 4
			GVL_IO.x_CF06 := aRxBuffer[1].2;//TGC_On_Request_Float 5
			GVL_IO.x_CA06 := aRxBuffer[1].3;//TGC_On_Request_Analog 5
			GVL_IO.x_CF07 := aRxBuffer[1].4;//Not_Used_FLEX_IN6_Float 6
			GVL_IO.x_CA07 := aRxBuffer[1].5;//Not_Used_FLEX_IN6_Analog 6
			GVL_IO.x_CF08 := aRxBuffer[1].6;//Not_Used_FLEX_IN7_Float 7
			GVL_IO.x_CA08 := aRxBuffer[1].7;//Not_Used_FLEX_IN7_Analog 7
			GVL_IO.x_CF09 := aRxBuffer[2].0;//Not_Used_FLEX_IN8_Float 8
			GVL_IO.x_CA09 := aRxBuffer[2].1;//Not_Used_FLEX_IN8_Analog 8
			GVL_IO.x_CF10 := aRxBuffer[2].2;//Positive_Auxiliary_Contactor_Feedback_Float 9
			GVL_IO.x_CA10 := aRxBuffer[2].3;//Positive_Auxiliary_Contactor_Feedback_Analog 9
			GVL_IO.x_CF11 := aRxBuffer[2].4;//Negative_Auxiliary_Contactor_Feedback_Float 10
			GVL_IO.x_CA11 := aRxBuffer[2].5;//Negative_Auxiliary_Contactor_Feedback_Analog 10
			GVL_IO.x_CF12 := aRxBuffer[2].6;//Not_Used_FLEX_IN11_Float 11
			GVL_IO.x_CA12 := aRxBuffer[2].7;//Not_Used_FLEX_IN11_Analog 11
			GVL_IO.x_CF13 := aRxBuffer[3].0;//HW_E_Stop_Signal_Float 12
			GVL_IO.x_CA13 := aRxBuffer[3].1;//HW_E_Stop_Signal_Analog 12
			GVL_IO.x_CF14 := aRxBuffer[3].2;//TPMS_Power_Float 13
			GVL_IO.x_CA14 := aRxBuffer[3].3;//TPMS_Power_Analog 13
			GVL_IO.x_CF15 := aRxBuffer[3].4;//Differential_Lock_Sensor_Signal_Float 14
			GVL_IO.x_CA15 := aRxBuffer[3].5;//Differential_Lock_Sensor_Signal_Analog 14
			GVL_IO.x_CF16 := aRxBuffer[3].6;//Charge_Door_Position_Float 15
			GVL_IO.x_CA16 := aRxBuffer[3].7;//Charge_Door_Position_Analog 15
			
// /// Arduino ///
// 		16#18: //BANCO_VCM4_PWM_HBO_0 
// 		16#19: //BANCO_VCM4_PWM_HBO_1 
// 		16#1A: //BANCO_VCM4_PWM_HBO_2 
// 		16#1B: //BANCO_VCM4_PWM_HBO_3 
		
		16#1C: //BANCO_VCM4_Analog_IN_0_3 ++
			GVL_IO.w_FlexIN0 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[1]),8) + BYTE_TO_WORD(aRxBuffer[0]))) * 0.001 * 32767) ); // Refer to the manual of 597 page 34
			GVL_IO.w_FlexIN1 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[3]),8) + BYTE_TO_WORD(aRxBuffer[2]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN2 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[5]),8) + BYTE_TO_WORD(aRxBuffer[4]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN3 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[7]),8) + BYTE_TO_WORD(aRxBuffer[6]))) * 0.001 * 32767) ); 			
		16#1D: //BANCO_VCM4_Analog_IN_4_7 ++
			GVL_IO.w_FlexIN4 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[1]),8) + BYTE_TO_WORD(aRxBuffer[0]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN5 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[3]),8) + BYTE_TO_WORD(aRxBuffer[2]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN6 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[5]),8) + BYTE_TO_WORD(aRxBuffer[4]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN7 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[7]),8) + BYTE_TO_WORD(aRxBuffer[6]))) * 0.001 * 32767) );		
		16#1E: //BANCO_VCM4_Analog_IN_8_11 ++
			GVL_IO.w_FlexIN8 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[1]),8) + BYTE_TO_WORD(aRxBuffer[0]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN9 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[3]),8) + BYTE_TO_WORD(aRxBuffer[2]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN10 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[5]),8) + BYTE_TO_WORD(aRxBuffer[4]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN11 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[7]),8) + BYTE_TO_WORD(aRxBuffer[6]))) * 0.001 * 32767) );
		16#1F: //BANCO_VCM4_Analog_IN_12_15 ++
			GVL_IO.w_FlexIN12 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[1]),8) + BYTE_TO_WORD(aRxBuffer[0]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN13 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[3]),8) + BYTE_TO_WORD(aRxBuffer[2]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN14 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[5]),8) + BYTE_TO_WORD(aRxBuffer[4]))) * 0.001 * 32767) ); 
			GVL_IO.w_FlexIN15 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[7]),8) + BYTE_TO_WORD(aRxBuffer[6]))) * 0.001 * 32767) );

// 		16#20: //BANCO_VCM4_PWM_IN_NOT_USED ++
// 			GVL_IO.b_PWM1_Control_Byte_2 :=  aRxBuffer[0];
// 			GVL_IO.b_PWM2_Control_Byte_0 :=  aRxBuffer[1];
// 			GVL_IO.w_PWM1_DO3 := REAL_TO_WORD((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[3]),8) + BYTE_TO_WORD(aRxBuffer[2]))));
// 			GVL_IO.w_PWM2_DO1 := REAL_TO_WORD((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[5]),8) + BYTE_TO_WORD(aRxBuffer[4]))));
// 			GVL_IO.w_PWM2_DO2 := REAL_TO_WORD((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[7]),8) + BYTE_TO_WORD(aRxBuffer[6]))));			
			
		
		16#21: //BANCO_VCM4_Analog_Critical_ADC_NOT_USED ++	
			GVL_IO.w_ANC2 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[1]),8) + BYTE_TO_WORD(aRxBuffer[0]))) * 0.001 * 32767) ); 
			GVL_IO.w_ANC5 := INT_TO_WORD(REAL_TO_INT((WORD_TO_REAL(SHL(BYTE_TO_WORD(aRxBuffer[3]),8) + BYTE_TO_WORD(aRxBuffer[2]))) * 0.001 * 32767) ); 	
END_CASE

//ON MESSAGE ANSWER
_FbCanTx11BitFrame( xEnable := TRUE,
					I_Port := CAN_Gateway,
					wCanId := 16#75D,
					xRtrFrame := FALSE,
					aTxBuffer := aRxBuffer,
					bTxNBytes := bRxNBytes,
					xTxTrigger := xCANTxTrigger,
					xValid =>,
					xError =>,
					xBusy =>,
					oStatus =>
					);
