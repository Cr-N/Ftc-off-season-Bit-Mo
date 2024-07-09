package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Arm implements Subsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;
    public static class Params{
        public double Arm_Pick_Up_Position =30;
        public double Arm_Deploy_Position = 150;
        public double Arm_HangSafe_Position = 150;
        public enum ArmStates{
            AT_PICK_UP_POSITION,
            AT_HangSafe_POSITION,
            AT_DEPLOY_POSITION
        };

    }
    public static Params PARAMETERS = new Params();
    Params.ArmStates ArmState = Params.ArmStates.AT_PICK_UP_POSITION;
    private final ServoEx Arm;
    public Arm(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        if (this.hardwareMap != null) {
            Arm = new SimpleServo(hardwareMap, "Arm", 0, 270, AngleUnit.DEGREES);
        } else {
            throw new NullPointerException("HardwareMap is null.");
        }
    }
    public void Arm_To_Pick_Up_Position() {
        if(ArmState != Params.ArmStates.AT_PICK_UP_POSITION)
            Arm.turnToAngle(PARAMETERS.Arm_Pick_Up_Position);
        ArmState = Params.ArmStates.AT_PICK_UP_POSITION;
    }
    public void Arm_To_DeployPosition() {
        if(ArmState != Params.ArmStates.AT_DEPLOY_POSITION)
            Arm.turnToAngle(PARAMETERS.Arm_Deploy_Position);
        ArmState = Params.ArmStates.AT_DEPLOY_POSITION;
    }
    public void Arm_To_HangSafePosition() {
        if(ArmState != Params.ArmStates.AT_HangSafe_POSITION)
            Arm.turnToAngle(PARAMETERS.Arm_HangSafe_Position);
        ArmState = Params.ArmStates.AT_HangSafe_POSITION;
    }
    public double getCurrent_Arm_Angle(){
        return Arm.getAngle();
    }
    public double getCurrent_Arm_Position(){
        return Arm.getPosition();
    }
    public Params.ArmStates getArmState(){
        return ArmState;
    }

}
