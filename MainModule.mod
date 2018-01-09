MODULE MainModule
!
!  Released under MIT License
!
!  David Pollard
!  Bristol Robotics Laboratory
!
!  Original author
!  Michael Dawson-Haggerty
!
!

    !
    !
    ! Main process... Calls ServerMain
	PROC main()
        ! Set enable signal
		ChangeExtruderState(1);
        
        ! Start server
        ServerMain;
	ENDPROC
    
    
    !
    !
    ! Resets all signals to low
    PROC ResetAllSignals()
        SetDO DO10_1, 0;
        SetDO DO10_2, 0;
        SetDO DO10_3, 0;
        SetDO DO10_4, 0;
        SetDO DO10_5, 0;
        SetDO DO10_6, 0;
        SetDO DO10_7, 0;
        SetDO DO10_8, 0;
        SetDO DO10_9, 0;
        SetDO DO10_10, 0;
        SetDO DO10_11, 0;
        SetDO DO10_12, 0;
        TPWrite "Signals reset";
    ENDPROC
    
    !
    !
    ! Rehomes robot
    PROC MoveToZeroPos()
        VAR jointtarget HomePose;
        VAR robtarget pTemp;
        ActUnit STN1;
        pTemp := CRobT();
        pTemp.extax.eax_b := 0;
        pTemp.extax.eax_c := 0;
        HomePose := [[0,0,0,0,0,0], pTemp.extax];
        ResetAllSignals;
        MoveAbsJ HomePose, v200, fine, tool0;
        DeactUnit STN1;
        EXIT;
    ENDPROC
    
    !
    !
    ! Test group output signal
    PROC CountUp()
        SetDO DO10_1, 1;
        SetDO DO10_2, 1;
        FOR reg1 FROM 0 TO 255 DO
            TPWrite "val: ", \Num:=reg1;
            SetGO GO_Signal, reg1;
            WaitTime 0.25;
        ENDFOR
    ENDPROC
    
    !
    !
    ! Test digital output signals
	PROC TestDO()
        TPWrite "DO10_1";
		SetDO DO10_1, 1;
        WaitTime 2;
        SetDO DO10_1, 0;
        WaitTime 2;
        
        TPWrite "DO10_2";
		SetDO DO10_2, 1;
        WaitTime 2;
        SetDO DO10_2, 0;
        WaitTime 2;
        
        TPWrite "DO10_3";
		!SetDO DO10_3, 1;
        WaitTime 2;
       ! SetDO DO10_3, 0;
        WaitTime 2;
        
        TPWrite "DO10_4";
		!SetDO DO10_4, 1;
        WaitTime 2;
        !SetDO DO10_4, 0;
        WaitTime 2;
        
        TPWrite "DO10_5";
		SetDO DO10_5, 1;
        WaitTime 2;
        SetDO DO10_5, 0;
        WaitTime 2;
        
        TPWrite "DO10_6";
		SetDO DO10_6, 1;
        WaitTime 2;
        SetDO DO10_6, 0;
        WaitTime 2;
        
        TPWrite "DO10_7";
		SetDO DO10_7, 1;
        WaitTime 2;
        SetDO DO10_7, 0;
        WaitTime 2;
        
        TPWrite "DO10_8";
		SetDO DO10_8, 1;
        WaitTime 2;
        SetDO DO10_8, 0;
        WaitTime 2;
        
        TPWrite "DO10_9";
		SetDO DO10_9, 1;
        WaitTime 2;
        SetDO DO10_9, 0;
        WaitTime 2;
        
        TPWrite "DO10_10";
		SetDO DO10_10, 1;
        WaitTime 2;
        SetDO DO10_10, 0;
        WaitTime 2;
        
        TPWrite "DO10_11";
		SetDO DO10_11, 1;
        WaitTime 2;
        SetDO DO10_11, 0;
        WaitTime 2;
        
        TPWrite "DO10_12";
		SetDO DO10_12, 1;
        WaitTime 2;
        SetDO DO10_12, 0;
        WaitTime 10;
	ENDPROC
ENDMODULE