package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Rotate implements Subsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;
    public static class Params{

        public double Rotate_Pick_Up_Position = 110;
        public double Rotate_Deploy_Position =27;
        public double Rotate_START_Position =50;
        public double Rotate_HangSafe_Position=120; // to be determined

        public enum RotationState{
            AT_PICK_UP_POSITION,
            AT_DEPLOY_POSITION,
            AT_START_POSITION,
            AT_HangSafe_Position
        }
    }
    public static Rotate.Params PARAMETERS = new Rotate.Params();
    Params.RotationState StateOfRotation = Params.RotationState.AT_START_POSITION;
    private final ServoEx Rotate;

    public Rotate(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        if (this.hardwareMap != null) {
            Rotate = new SimpleServo(hardwareMap, "Rotate", 0, 180, AngleUnit.DEGREES);
        } else {
            throw new NullPointerException("HardwareMap is null.");
        }
    }
    public void Rotate_To_Pick_Up_Position(){
        if(StateOfRotation != Params.RotationState.AT_PICK_UP_POSITION)
            Rotate.turnToAngle(PARAMETERS.Rotate_Pick_Up_Position);
        StateOfRotation = Params.RotationState.AT_PICK_UP_POSITION;
    }
    public void Rotate_To_Deploy_Position(){
        if(StateOfRotation != Params.RotationState.AT_DEPLOY_POSITION)
            Rotate.turnToAngle(PARAMETERS.Rotate_Deploy_Position);
        StateOfRotation = Params.RotationState.AT_DEPLOY_POSITION;
    }
    public void Rotate_To_HangSafe_Position(){
        if(StateOfRotation != Params.RotationState.AT_HangSafe_Position)
            Rotate.turnToAngle(PARAMETERS.Rotate_HangSafe_Position);
        StateOfRotation = Params.RotationState.AT_HangSafe_Position;
    }
    public void Rotate_To_Intermediary_Position(){
        if(StateOfRotation != Params.RotationState.AT_START_POSITION)
            Rotate.turnToAngle(PARAMETERS.Rotate_START_Position);
        StateOfRotation = Params.RotationState.AT_START_POSITION;
    }
    public Rotate.Params.RotationState Get_State_Of_Rotate(){
        return StateOfRotation;
    }
    public double Get_Current_Rotate_Angle(){
        return Rotate.getAngle();
    }
    public double Get_Current_Rotate_Position(){
        return Rotate.getPosition();
    }
}
