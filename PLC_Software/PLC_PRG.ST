PROGRAM PLC_PRG
VAR

	COUNTER : ARRAY[0..7] OF BYTE;
	_FB_InitPeriodicMessage : FB_InitPeriodicMessage;
	//_FB_InitRXMessage : FB_InitRXMessage;
	_GetDateAndTime : dtu.GetDateAndTime;
	_FbModuleConfigurationAndStatus : FbModuleConfigurationAndStatus;
	_FbCanL2Open : FbCanL2Open;
	_FbCanErrorInfo : FbCanErrorInfo;
	_FbCanRx11BitFrameAll : FbCanRx11BitFrameAll;
	_FB_Millis : FB_Millis;
	_FB_RXCanMessageElaboration : FB_RXCanMessageElaboration;
	_FB_handlePeriodicMessages : FB_handlePeriodicMessages;
	msTimer : TON := (IN :=TRUE, PT:= UDINT_TO_TIME(4294967295));
	dtTime : DATE_AND_TIME;
	tActualTime, tLastElapsedTime : TIME;
	stTypFullConfiguration: WagoSysModule_75x_658.typFullConfiguration; 
	xCANRxTrigger, xCANdone,xCANbusy,xCANerror,xCAN29BitEnabled,xCANSdoEnabled,xCANDiagOverMbx0Enabled,xCANDiagnosisReset,xCANCommunicationReset: BOOL;
	udiCANBaudrate: udint;
	I_CANport : WagoSysModule_75x_658.I_WagoModule_75x_658Extended;
	eCANBitrate : WagoSysModule_75x_658.eBitrate;
	eCANMode : WagoSysModule_75x_658.eMode := WagoSysModule_75x_658.eMode.CAN_Transparent;
	wCANRxId,wCANCounter:WORD;
	xCANValid,xCANRtrFrame,xCANTriggerResetCounter:BOOL;
	oCANStatus :WagoAppCanLayer2.WagoSysErrorBase.FbResult;
	bCANRxNBytes:BYTE;
	aCANRxBuffer,aCANTxBuffer:ARRAY[1..8]OF BYTE;
	aCANTxBuffer1:ARRAY[0..23]OF BYTE;
	iCanStateMachine, iCANbufferCursor: INT :=0;
	xPeriodicMessageInitDone:BOOL:=FALSE;
	wAliveCounter:WORD;
END_VAR



PLC_INPUT();
PLC_OUTPUT();

IF (NOT xPeriodicMessageInitDone) THEN
	_FB_InitPeriodicMessage(aPeriodicMessages => GVL.aPeriodicMessages);

	I_CANport := CAN_Gateway;
	eCANBitrate := WagoSysModule_75x_658.eBitrate.CAN_500kBits;
	udiCANBaudrate := 500000;	
	eCANMode := WagoSysModule_75x_658.eMode.CAN_Transparent;
	xCAN29BitEnabled := FALSE;
	xCANSdoEnabled := FALSE;
	xCANDiagOverMbx0Enabled := FALSE;
	xCANCommunicationReset := FALSE;
	
	xPeriodicMessageInitDone := TRUE;
	//_FB_InitRXMessage(aRXMessages => gvl.aRXMessages);
END_IF

_GetDateAndTime(xExecute := TRUE,
				dtDateAndTime => dtTime
				);
msTimer();
tActualTime := msTimer.ET;


CASE iCanStateMachine OF
	0: //write config values
		_FbModuleConfigurationAndStatus(I_Port := I_CANport,
										xExecute := TRUE,
										xWriteRead := FALSE,//NOT xCANdone,
										xSaveValues := TRUE,
										dtTime := dtTime,
										eBitrate := eCANBitrate,
										eMode := eCANMode,
										x29BitEnabled := xCAN29BitEnabled,
										xSdoEnabled := xCANSdoEnabled,
										xDiagOverMbx0Enabled := xCANDiagOverMbx0Enabled,
										typConfigData := stTypFullConfiguration,
										xDiagnosisReset := xCANDiagnosisReset,
										xCommunicationReset := xCANCommunicationReset,
										xCheckConfiguration := TRUE,
										xDone => xCANdone,
										xBusy => xCANbusy,
										xError => xCANerror,
										oStatus =>oCANStatus,
										xDataMatches =>,
										eDiagnosisErrorCode =>,
										bDiagnosisChannelNo =>,
										eDiagnosisMode =>,
										eDiagnosisType =>,
										wDiagnosisErrorCounter =>,
										wDiagnosisWarningCounter =>,
										wDiagnosisTotalCounter =>,
										xStatusBitModulError =>,
										xStatusBitRxOverflow =>,
										xStatusBitRxOverflow =>,
										xStatusBitCanError =>
										);
										
		IF _FbModuleConfigurationAndStatus.xDone AND NOT _FbModuleConfigurationAndStatus.xError THEN
			iCanStateMachine := 1;
		ELSIF _FbModuleConfigurationAndStatus.xError THEN
			iCanStateMachine := 99;
		END_IF
		
	1: //look for errors
		_FbCanErrorInfo(xEnable:= TRUE,
						I_Port := I_CANport,
						xTriggerResetCounter := xCANTriggerResetCounter,
						xValid=>,
						xError=>,
						oStatus=>,
						wBusState=>,
						wBusDiag=>);
						
		IF _FbCanErrorInfo.xValid AND NOT _FbCanErrorInfo.xError THEN
			iCanStateMachine := 2;
		ELSIF _FbCanErrorInfo.xError THEN
			iCanStateMachine := 99;
		END_IF
		
	2: //open can layer2 port + RX + TX					
		_FbModuleConfigurationAndStatus(I_Port := I_CANport,
								xExecute := TRUE,
								xWriteRead := FALSE,//NOT xCANdone,
								xSaveValues := FALSE,
								dtTime := dtTime,
								eBitrate := eCANBitrate,
								eMode := eCANMode,
								x29BitEnabled := xCAN29BitEnabled,
								xSdoEnabled := xCANSdoEnabled,
								xDiagOverMbx0Enabled := xCANDiagOverMbx0Enabled,
								typConfigData := stTypFullConfiguration,
								xDiagnosisReset := xCANDiagnosisReset,
								xCommunicationReset := xCANCommunicationReset,
								xCheckConfiguration := TRUE,
								xDone => ,
								xBusy => ,
								xError => ,
								oStatus =>,
								xDataMatches =>,
								eDiagnosisErrorCode =>,
								bDiagnosisChannelNo =>,
								eDiagnosisMode =>,
								eDiagnosisType =>,
								wDiagnosisErrorCounter =>,
								wDiagnosisWarningCounter =>,
								wDiagnosisTotalCounter =>,
								xStatusBitModulError =>,
								xStatusBitRxOverflow =>,
								xStatusBitRxOverflow =>,
								xStatusBitCanError =>
								);
						
		_FbCanL2Open(xEnable:= TRUE,
						I_Port := I_CANport,
						udiBaudrate := udiCANBaudrate,
						dwFlags := 0,
						dwpara := 0);
						
		IF _FbModuleConfigurationAndStatus.xDone AND NOT _FbModuleConfigurationAndStatus.xError THEN
			xCANRxTrigger := TRUE;	
			_FbCanRx11BitFrameAll(	xEnable := _FbModuleConfigurationAndStatus.xDone,
									I_Port := I_CANport,
									xBufferMode := TRUE,
									xRxTrigger := xCANRxTrigger,
									xValid => xCANValid,
									xError => xCANError,
									xBusy => xCANBusy,
									oStatus => oCANStatus,
									wCounter => ,
									xRtrFrame => xCANRtrFrame,
									bRxNBytes => bCANRxNBytes,
									aRxBuffer => ,
									wRxId => wCANRxId);	
		END_IF
		
		//RECEIVED MESSAGES
		IF _FbCanRx11BitFrameAll.xValid AND (_FbCanRx11BitFrameAll.wFrames>0) THEN
			//more than one message received
			FOR iCANbufferCursor := 1 TO WORD_TO_INT(_FbCanRx11BitFrameAll.wFrames) DO
				
				_FB_RXCanMessageElaboration(wRxId := DWORD_TO_WORD(_FbCanRx11BitFrameAll.auxRxData[iCANbufferCursor].dwID),
									bRxNBytes := _FbCanRx11BitFrameAll.auxRxData[iCANbufferCursor].bLen,
									aRxBuffer := _FbCanRx11BitFrameAll.auxRxData[iCANbufferCursor].aData
									);
			END_FOR
			wCANCounter := _FbCanRx11BitFrameAll.wCounter;
		ELSIF  _FbCanRx11BitFrameAll.xValid AND (_FbCanRx11BitFrameAll.wFrames = 0) AND (_FbCanRx11BitFrameAll.wCounter>wCANCounter) THEN
			//only one message received
			_FB_RXCanMessageElaboration(wRxId := _FbCanRx11BitFrameAll.wRxId,
								bRxNBytes := _FbCanRx11BitFrameAll.bRxNBytes,
								aRxBuffer := _FbCanRx11BitFrameAll.aRxBuffer
								);
			wCANCounter := _FbCanRx11BitFrameAll.wCounter;
		ELSIF _FbCanRx11BitFrameAll.xError OR _FbModuleConfigurationAndStatus.xError THEN	
			iCanStateMachine := 99;
		END_IF
		
		//TRANSMITTED MESSAGES: periodic
		FOR iCANbufferCursor:=1 TO GVL.uiPeriodcMessageN DO
			mem.MemFill(pMemoryBlock := ADR(aCANTxBuffer),
						uiLength := SIZEOF(aCANTxBuffer),
						byfillvalue :=0);
						
			CASE iCANbufferCursor OF
				//insert here the data field of the message
				1:  //BANCO_VCM4_Alive
					IF wAliveCounter<65535 THEN
						wAliveCounter := wAliveCounter + 1;
					ELSE
						wAliveCounter := 0;
					END_IF
					aCANTxBuffer[1] := WORD_TO_BYTE(SHR(wAliveCounter,8));
					aCANTxBuffer[2] := WORD_TO_BYTE(wAliveCounter);
					
				2: 	//BANCO_VCM4_DOUT

					aCANTxBuffer[1].0 := GVL_IO.x_DOV01;
					aCANTxBuffer[1].1 := GVL_IO.x_DOV02;
					aCANTxBuffer[1].2 := GVL_IO.x_DOV03;
					aCANTxBuffer[1].3 := GVL_IO.x_DOV04; 
					aCANTxBuffer[1].4 := GVL_IO.x_DOV05;
					aCANTxBuffer[1].5 := GVL_IO.x_DOV06;
					aCANTxBuffer[1].6 := GVL_IO.x_DOV07;
					aCANTxBuffer[1].7 := GVL_IO.x_DOV08;
					aCANTxBuffer[2].0 := GVL_IO.x_DOG01;
					aCANTxBuffer[2].1 := GVL_IO.x_DOG02;
					aCANTxBuffer[2].2 := GVL_IO.x_DOG03;
					aCANTxBuffer[2].3 := GVL_IO.x_DOG04;
					aCANTxBuffer[2].4 := GVL_IO.x_DOG05;
					aCANTxBuffer[2].5 := GVL_IO.x_DOG06;
					aCANTxBuffer[2].6 := GVL_IO.x_DOG07;
					aCANTxBuffer[3].0 := GVL_IO.x_DOG08;
					aCANTxBuffer[3].1 := GVL_IO.x_DOG09;
					aCANTxBuffer[3].2 := GVL_IO.x_DOG10;
					aCANTxBuffer[3].3 := GVL_IO.x_DOG11;
					aCANTxBuffer[3].4 := GVL_IO.x_DOG12;					
			END_CASE
			
			_FB_handlePeriodicMessages( tActualTime := tActualTime,
										tLastElapsedTime := GVL.aPeriodicMessages[iCANbufferCursor].tLastElapsedTime,
										tPeriod := GVL.aPeriodicMessages[iCANbufferCursor].tPeriod,
										wTxId := GVL.aPeriodicMessages[iCANbufferCursor].wTxId,
										bTxNBytes := GVL.aPeriodicMessages[iCANbufferCursor].bTxNBytes,
										aTxBuffer := aCANTxBuffer,
										);

			aCANTxBuffer1[0] := 1;
			aCANTxBuffer1[1] := 2;
			aCANTxBuffer1[2] := 3;
			aCANTxBuffer1[3] := 4;
			aCANTxBuffer1[4] := 5;
			aCANTxBuffer1[5] := 6;
			aCANTxBuffer1[6] := 7;
			aCANTxBuffer1[7] := 8;
			
// 			GVL_IO.w_CAN_SEND[0] := aCANTxBuffer1[0];
// 			GVL_IO.w_CAN_SEND[1] := aCANTxBuffer1[1];
// 			GVL_IO.w_CAN_SEND[2] := aCANTxBuffer1[2];
// 			GVL_IO.w_CAN_SEND[3] := aCANTxBuffer1[3];
// 			GVL_IO.w_CAN_SEND[4] := aCANTxBuffer1[4];
// 			GVL_IO.w_CAN_SEND[5] := aCANTxBuffer1[5];
// 			GVL_IO.w_CAN_SEND[6] := aCANTxBuffer1[6];
// 			GVL_IO.w_CAN_SEND[7] := aCANTxBuffer1[7];
			
			
										
// 			_FB_handlePeriodicMessages( tActualTime := tActualTime,
// 										tLastElapsedTime := GVL.aPeriodicMessages[iCANbufferCursor].tLastElapsedTime,
// 										tPeriod := GVL.aPeriodicMessages[iCANbufferCursor].tPeriod,
// 										wTxId := GVL.aPeriodicMessages[iCANbufferCursor].wTxId,
// 										bTxNBytes := GVL.aPeriodicMessages[iCANbufferCursor].bTxNBytes,
// 										aTxBuffer := aCANTxBuffer1,
// 										);										



										
		END_FOR
	99: //error --> retry
		iCanStateMachine := 2;
		//GVL_IO.x_DHLC01 := TRUE;
		
END_CASE
