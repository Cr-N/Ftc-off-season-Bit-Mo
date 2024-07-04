package org.firstinspires.ftc.teamcode.HardWare;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;


public class Hardware {

    private LinearOpMode myOpMode = null;
    Intake.Params PARAMETERES = new Intake.Params();

    // Declare motor objects
    public DcMotor FL = null;
    public DcMotor FR = null;
    public DcMotor BL = null;
    public DcMotor BR = null;
    public BNO055IMU imu;
   // public ServoEx claw;

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

       // claw = new SimpleServo(hardwareMap,"claw",0,360, AngleUnit.DEGREES);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit= BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu.initialize(parameters);

       // claw = new SimpleServo(myOpMode.hardwareMap,"claw",0,270, AngleUnit.DEGREES);
       // claw.turnToAngle(Par.intakePosition);

        myOpMode.telemetry.addData(">","Hardware Initialized");
        myOpMode.telemetry.update();
    }
}
