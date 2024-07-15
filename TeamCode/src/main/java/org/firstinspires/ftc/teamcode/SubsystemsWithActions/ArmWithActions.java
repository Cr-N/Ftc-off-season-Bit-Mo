package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmWithActions{

    public static class Params{
        public double Arm_Pick_Up_Position =5;
        public double Arm_Deploy_Position = 135;
        public double Arm_HangSafe_Position = 140; // nu
        public double Arm_Intermediary_Position =25;
        public double arm_angle;
        public double lower_arm_error =0.02;
        public double upper_arm_error =0.02;
        public enum ArmStates{
            AT_PICK_UP_POSITION,
            AT_HangSafe_POSITION,
            AT_DEPLOY_POSITION,
            AT_INTERMEDIARY_POSITION
        };
    }

    public static Params PARAMETERS = new Params();

    Params.ArmStates ArmState = Params.ArmStates.AT_PICK_UP_POSITION;

    private final ServoEx Arm;

    public ArmWithActions(HardwareMap hardwareMap){
        Arm = new SimpleServo(hardwareMap, "Arm", 0, 180, AngleUnit.DEGREES);
    }

    public class Arm_To_PickUp_Position implements Action{

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                if(ArmState != Params.ArmStates.AT_PICK_UP_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_Pick_Up_Position);
                    initialized = true;
                }

            }

            PARAMETERS.arm_angle = Arm.getAngle();

            telemetryPacket.put("arm angle ", PARAMETERS.arm_angle);
            telemetryPacket.put("arm state ", ArmState);

            if(PARAMETERS.arm_angle < PARAMETERS.Arm_Pick_Up_Position- PARAMETERS.lower_arm_error || PARAMETERS.arm_angle > PARAMETERS.Arm_Pick_Up_Position + PARAMETERS.upper_arm_error){
                // true reruns action
                return true;
            }
            else{
                // false stops action
                ArmState = Params.ArmStates.AT_PICK_UP_POSITION;
                return false;
            }
        }
    }

    public class Arm_To_Deploy_Position implements Action{

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                if(ArmState != Params.ArmStates.AT_DEPLOY_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_Deploy_Position);
                    initialized = true;
                }

            }

            PARAMETERS.arm_angle = Arm.getAngle();

            telemetryPacket.put("arm angle ", PARAMETERS.arm_angle);
            telemetryPacket.put("arm state ", ArmState);

            if(PARAMETERS.arm_angle < PARAMETERS.Arm_Deploy_Position - PARAMETERS.lower_arm_error || PARAMETERS.arm_angle > PARAMETERS.Arm_Deploy_Position + PARAMETERS.upper_arm_error){
                // true reruns action
                return true;
            }
            else{
                // false stops action
                ArmState = Params.ArmStates.AT_DEPLOY_POSITION;
                return false;
            }
        }
    }

    public class Arm_To_HangSafe_Position implements Action{

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                if(ArmState != Params.ArmStates.AT_HangSafe_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_HangSafe_Position);
                    initialized = true;
                }
            }

            PARAMETERS.arm_angle = Arm.getAngle();

            telemetryPacket.put("arm angle ", PARAMETERS.arm_angle);
            telemetryPacket.put("arm state ", ArmState);

            if(PARAMETERS.arm_angle < PARAMETERS.Arm_HangSafe_Position- PARAMETERS.lower_arm_error || PARAMETERS.arm_angle > PARAMETERS.Arm_HangSafe_Position + PARAMETERS.upper_arm_error){
                // true reruns action
                return true;
            }
            else{
                // false stops action
                ArmState = Params.ArmStates.AT_HangSafe_POSITION;
                return false;
            }
        }
    }

    public class Arm_To_Intermediary_Position implements Action{
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                if(ArmState != Params.ArmStates.AT_INTERMEDIARY_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_Intermediary_Position);
                    initialized = true;
                }
            }

            PARAMETERS.arm_angle = Arm.getAngle();

            telemetryPacket.put("arm angle ", PARAMETERS.arm_angle);
            telemetryPacket.put("arm state ", ArmState);

            if(PARAMETERS.arm_angle < PARAMETERS.Arm_Intermediary_Position- PARAMETERS.lower_arm_error || PARAMETERS.arm_angle > PARAMETERS.Arm_Intermediary_Position + PARAMETERS.upper_arm_error){
                // true reruns action
                return true;
            }
            else{
                // false stops action
                ArmState = Params.ArmStates.AT_INTERMEDIARY_POSITION;
                return false;
            }
        }
    }

    public Action Arm_To_PickUp_Position(){
        return new Arm_To_PickUp_Position();
    }


    public Action Arm_To_Deploy_Position(){
        return new Arm_To_Deploy_Position();
    }

    public Action Arm_To_HangSafe_Position(){
        return new Arm_To_HangSafe_Position();
    }

    public Action Arm_To_Intermediary_Position(){
        return new Arm_To_Intermediary_Position();
    }

    public double Get_Current_Arm_Angle(){
        return Arm.getAngle();
    }
    public double Get_Current_Arm_Position(){
        return Arm.getPosition();
    }
    public Params.ArmStates Get_Arm_State(){
        return ArmState;
    }

}
