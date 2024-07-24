package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RotateWithActions{

    public static class Params{

        public double Rotate_Pick_Up_Position = 110;
        public double Rotate_Deploy_Position =27;
        public double Rotate_START_Position =50;
        public double Rotate_Purple_Pixel_Deploy_Position =110;
        public double Rotate_Stack_Pickup_Position =130;
        public double Rotate_HangSafe_Position=120; // to be determined
        public double WAIT_TIME = 1.5;
        public double rotate_angle;
        public enum RotationState{
            AT_PICK_UP_POSITION,
            AT_DEPLOY_POSITION,
            AT_START_POSITION,
            AT_HangSafe_Position,
            AT_PURPLE_PIXEL_DEPLOY_POSITION,
            AT_STACK_PICKUP
        }
    }

    public static Params PARAMETERS = new RotateWithActions.Params();

    Params.RotationState StateOfRotation = Params.RotationState.AT_START_POSITION;

    private final ServoEx Rotate;

    public RotateWithActions(HardwareMap hardwareMap) {
        Rotate = new SimpleServo(hardwareMap, "Rotate", 0, 180, AngleUnit.DEGREES);
    }

    public class Rotate_To_Stack_Pickup implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(StateOfRotation != Params.RotationState.AT_STACK_PICKUP){
                    Rotate.turnToAngle(PARAMETERS.Rotate_Stack_Pickup_Position);
                }
            new SleepAction(PARAMETERS.WAIT_TIME);
            StateOfRotation = Params.RotationState.AT_STACK_PICKUP;
            return false;
        }
    }

    public class Rotate_To_Pick_Up_Position implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(StateOfRotation != Params.RotationState.AT_PICK_UP_POSITION){
                    Rotate.turnToAngle(PARAMETERS.Rotate_Pick_Up_Position);
                }
            new SleepAction(PARAMETERS.WAIT_TIME);
            StateOfRotation = Params.RotationState.AT_PICK_UP_POSITION;
            return false;
        }
    }

    public class Rotate_To_Deploy_Position implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(StateOfRotation != Params.RotationState.AT_DEPLOY_POSITION){
                        Rotate.turnToAngle(PARAMETERS.Rotate_Deploy_Position);
                    }
                new SleepAction(PARAMETERS.WAIT_TIME);
                StateOfRotation = Params.RotationState.AT_DEPLOY_POSITION;
                return false;
        }

    }

    public class Rotate_To_HangSafe_Position implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(StateOfRotation != Params.RotationState.AT_HangSafe_Position){
                        Rotate.turnToAngle(PARAMETERS.Rotate_HangSafe_Position);
                    }
                // false stops action
                new SleepAction(PARAMETERS.WAIT_TIME);
                StateOfRotation = Params.RotationState.AT_HangSafe_Position;
                return false;
        }
    }

    public class Rotate_To_Purple_Pixel_Deploy_Position implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(StateOfRotation != Params.RotationState.AT_PURPLE_PIXEL_DEPLOY_POSITION){
                    Rotate.turnToAngle(PARAMETERS.Rotate_Purple_Pixel_Deploy_Position);
                }
                new SleepAction(PARAMETERS.WAIT_TIME);
                StateOfRotation = Params.RotationState.AT_PURPLE_PIXEL_DEPLOY_POSITION;
                return false;
        }
    }
    public class Rotate_To_Start_For_Tele implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(StateOfRotation != Params.RotationState.AT_START_POSITION){
                Rotate.turnToAngle(PARAMETERS.Rotate_START_Position);
            }
            new SleepAction(PARAMETERS.WAIT_TIME);
            StateOfRotation = Params.RotationState.AT_START_POSITION;
            return false;
        }
    }
    public Action Rotate_To_Pick_Up_Position(){
        return new Rotate_To_Pick_Up_Position();
    }
    public Action Rotate_To_Deploy_Position(){
        return new Rotate_To_Deploy_Position();
    }
    public Action Rotate_To_HangSafe_Position(){
        return new Rotate_To_HangSafe_Position();
    }
    public Action Rotate_To_Start_For_Tele(){
        return new Rotate_To_Start_For_Tele();
    }

    public Action Rotate_To_Purple_Pixel_Deploy_Position(){
            return new Rotate_To_Purple_Pixel_Deploy_Position();
    }
    public Action Rotate_To_Stack_Intake_Position(){
        return new Rotate_To_Stack_Pickup();

    }

    public RotateWithActions.Params.RotationState Get_State_Of_Rotate(){
        return StateOfRotation;
    }

    public double Get_Current_Rotate_Angle(){
        return Rotate.getAngle();
    }

    public double Get_Current_Rotate_Position(){
        return Rotate.getPosition();
    }

}
