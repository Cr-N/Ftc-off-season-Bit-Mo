package org.firstinspires.ftc.teamcode.HardWare;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class DriveBase {

    private LinearOpMode myOpMode = null;

    // Declare motor objects
    public MotorEx FL = null;
    public MotorEx FR = null;
    public MotorEx BL = null;
    public MotorEx BR = null;

    HardwareMap hardwareMap;

    public DriveBase(LinearOpMode opMode){
        myOpMode = opMode;
        this.hardwareMap = myOpMode.hardwareMap;
    }

    public void init(){

        FR = new MotorEx(myOpMode.hardwareMap , "FR", Motor.GoBILDA.RPM_223);
        BL = new MotorEx(myOpMode.hardwareMap , "BL",Motor.GoBILDA.RPM_223);
        BR = new MotorEx(myOpMode.hardwareMap , "BR",Motor.GoBILDA.RPM_223);
        FL = new MotorEx(myOpMode.hardwareMap , "FL",Motor.GoBILDA.RPM_223);

        BL.setInverted(true);
        FL.setInverted(true);

        myOpMode.telemetry.addData(">","Hardware Initialized");
        myOpMode.telemetry.update();
    }
}
