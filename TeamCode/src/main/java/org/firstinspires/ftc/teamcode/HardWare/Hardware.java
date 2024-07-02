package org.firstinspires.ftc.teamcode.HardWare;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Hardware {

    private LinearOpMode myOpMode = null;

    // Declare motor objects
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public BNO055IMU imu;
    public ServoEx claw;
    HardwareMap hardwareMap;


    public Hardware(LinearOpMode opMode){
        myOpMode = opMode;
        this.hardwareMap = myOpMode.hardwareMap;
    }

    public void init(){

        FR = myOpMode.hardwareMap.get(DcMotor.class,"FR");
        BL = myOpMode.hardwareMap.get(DcMotor.class,"BL");
        BR = myOpMode.hardwareMap.get(DcMotor.class, "BR");
        FL = myOpMode.hardwareMap.get(DcMotor.class,"FL");


        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        myOpMode.telemetry.addData(">","Hardware Initialized");
        myOpMode.telemetry.update();
    }










    /*public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.FL = new DcMotorEx(hardwareMap,"FL", Motor.GoBILDA.RPM_223);
        this.FR = new DcMotorEx(hardwareMap,"FR", Motor.GoBILDA.RPM_223);
        this.BL = new DcMotorEx(hardwareMap,"BL", Motor.GoBILDA.RPM_223);
        this.BR = new DcMotorEx(hardwareMap,"BR", Motor.GoBILDA.RPM_223);
        this.claw = new SimpleServo(hardwareMap,"claw",0,360, AngleUnit.DEGREES);


        FL.setRunMode(Motor.RunMode.RawPower);
        FR.setRunMode(Motor.RunMode.RawPower);
        BL.setRunMode(Motor.RunMode.RawPower);
        BR.setRunMode(Motor.RunMode.RawPower);


        this.imu = hardwareMap.get(GyroEx.class , "imu");
        this.imu.init();
    }*/
}
