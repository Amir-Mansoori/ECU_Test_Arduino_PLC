{attribute 'qualified_only'}
VAR_GLOBAL CONSTANT
	uiPeriodcMessageN : UINT := 2;
	uiRXMessageN : UINT := 3;
END_VAR

VAR_GLOBAL
	aPeriodicMessages: ARRAY[1..GVL.uiPeriodcMessageN] OF PERIODIC_MESSAGE;
	aRXMessages: ARRAY[1..GVL.uiRXMessageN] OF RX_MESSAGE;


END_VAR