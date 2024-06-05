package org.firstinspires.ftc.teamcode.HardWare;

import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareMaps{

    public  HardwareMap hardwareMap;
    // Declare motor objects
    public Motor frontLeftMotor,frontRightMotor,backLeftMotor,backRightMotor;
    public GyroEx imu;
    /***
     * Constructor for RobotHardwareMap
     *
     * @param hardwareMap - The hardware map obtained from the TeleOp or Autonomous program
     */
    public HardwareMaps(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.frontLeftMotor = new Motor(hardwareMap,"FL", Motor.GoBILDA.RPM_312);
        this.frontRightMotor = new Motor(hardwareMap,"FR", Motor.GoBILDA.RPM_312);
        this.backLeftMotor = new Motor(hardwareMap,"BL", Motor.GoBILDA.RPM_312);
        this.backRightMotor = new Motor(hardwareMap,"BR", Motor.GoBILDA.RPM_312);

        frontLeftMotor.setRunMode(Motor.RunMode.RawPower);
        frontRightMotor.setRunMode(Motor.RunMode.RawPower);
        backLeftMotor.setRunMode(Motor.RunMode.RawPower);
        backRightMotor.setRunMode(Motor.RunMode.RawPower);

        this.imu = hardwareMap.get(GyroEx.class , "imu");
    }

}
