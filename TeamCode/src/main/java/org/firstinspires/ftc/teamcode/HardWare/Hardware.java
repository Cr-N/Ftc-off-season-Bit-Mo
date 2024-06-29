package org.firstinspires.ftc.teamcode.HardWare;

import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Hardware {

    public   HardwareMap hardwareMap;
    // Declare motor objects
    public MotorEx FL,FR,BL,BR;
    public GyroEx imu;
    /***
     * Constructor for RobotHardwareMap
     *
     * @param hardwareMap - The hardware map obtained from the TeleOp or Autonomous program
     */
    public Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.FL = new MotorEx(hardwareMap,"FL", Motor.GoBILDA.RPM_223);
        this.FR = new MotorEx(hardwareMap,"FR", Motor.GoBILDA.RPM_223);
        this.BL = new MotorEx(hardwareMap,"BL", Motor.GoBILDA.RPM_223);
        this.BR = new MotorEx(hardwareMap,"BR", Motor.GoBILDA.RPM_223);


        FL.setRunMode(Motor.RunMode.RawPower);
        FR.setRunMode(Motor.RunMode.RawPower);
        BL.setRunMode(Motor.RunMode.RawPower);
        BR.setRunMode(Motor.RunMode.RawPower);


        this.imu = hardwareMap.get(GyroEx.class , "imu");
        this.imu.init();
    }
}
