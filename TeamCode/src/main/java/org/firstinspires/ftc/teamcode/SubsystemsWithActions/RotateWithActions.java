package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class RotateWithActions{

    public static class Params{

        public double Rotate_Pick_Up_Position = 110;
        public double Rotate_Deploy_Position =27;
        public double Rotate_START_Position =50;
        public double Rotate_HangSafe_Position=120; // to be determined
        public double lower_rotate_error =0.02;
        public double upper_rotate_error=0.02;
        public double rotate_angle;
        public enum RotationState{
            AT_PICK_UP_POSITION,
            AT_DEPLOY_POSITION,
            AT_START_POSITION,
            AT_HangSafe_Position
        }
    }

    public static Params PARAMETERS = new RotateWithActions.Params();

    Params.RotationState StateOfRotation = Params.RotationState.AT_START_POSITION;

    private final ServoEx Rotate;

    public RotateWithActions(HardwareMap hardwareMap) {
        Rotate = new SimpleServo(hardwareMap, "Rotate", 0, 180, AngleUnit.DEGREES);
    }

    public class Rotate_To_Pick_Up_Position implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                if(StateOfRotation != Params.RotationState.AT_PICK_UP_POSITION){
                    Rotate.turnToAngle(PARAMETERS.Rotate_Pick_Up_Position);
                    initialized = true;
                }

            }

            PARAMETERS.rotate_angle = Rotate.getAngle();

            telemetryPacket.put("claw angle ", PARAMETERS.rotate_angle);
            telemetryPacket.put("claw state ", StateOfRotation);

            if(PARAMETERS.rotate_angle < PARAMETERS.Rotate_Pick_Up_Position - PARAMETERS.lower_rotate_error || PARAMETERS.rotate_angle > PARAMETERS.Rotate_Pick_Up_Position + PARAMETERS.upper_rotate_error){
                // true reruns action
                return true;
            }
            else{
                // false stops action
                StateOfRotation = Params.RotationState.AT_PICK_UP_POSITION;
                return false;
            }
        }
    }

    public class Rotate_To_Deploy_Position implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                if(StateOfRotation != Params.RotationState.AT_DEPLOY_POSITION){
                    Rotate.turnToAngle(PARAMETERS.Rotate_Deploy_Position);
                    initialized = true;
                }

            }

            PARAMETERS.rotate_angle = Rotate.getAngle();

            telemetryPacket.put("claw angle ", PARAMETERS.rotate_angle);
            telemetryPacket.put("claw state ", StateOfRotation);

            if(PARAMETERS.rotate_angle < PARAMETERS.Rotate_Deploy_Position- PARAMETERS.lower_rotate_error || PARAMETERS.rotate_angle > PARAMETERS.Rotate_Deploy_Position + PARAMETERS.upper_rotate_error){
                // true reruns action
                return true;
            }
            else{
                // false stops action
                StateOfRotation = Params.RotationState.AT_DEPLOY_POSITION;
                return false;
            }
        }
    }

    public class Rotate_To_HangSafe_Position implements Action {

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                if(StateOfRotation != Params.RotationState.AT_HangSafe_Position){
                    Rotate.turnToAngle(PARAMETERS.Rotate_HangSafe_Position);
                    initialized = true;
                }

            }

            PARAMETERS.rotate_angle = Rotate.getAngle();

            telemetryPacket.put("claw angle ", PARAMETERS.rotate_angle);
            telemetryPacket.put("claw state ", StateOfRotation);

            if(PARAMETERS.rotate_angle < PARAMETERS.Rotate_HangSafe_Position- PARAMETERS.lower_rotate_error || PARAMETERS.rotate_angle > PARAMETERS.Rotate_HangSafe_Position + PARAMETERS.upper_rotate_error){
                // true reruns action
                return true;
            }
            else{
                // false stops action
                StateOfRotation = Params.RotationState.AT_HangSafe_Position;
                return false;
            }
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
