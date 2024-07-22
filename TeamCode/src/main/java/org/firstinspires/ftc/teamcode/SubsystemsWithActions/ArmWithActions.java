package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ArmWithActions{

    public static class Params{
        public double Arm_Pick_Up_Position =25;
        public double Arm_Deploy_Position = 145;
        public double Arm_HangSafe_Position = 140; // nu
        public double Arm_Intermediary_Position =35;
        public double Arm_Purple_Pixel_Deploy_Position=35;
        public double Arm_Stack_Intake_Position = 35;
        public enum ArmStates{
            AT_PICK_UP_POSITION,
            AT_HangSafe_POSITION,
            AT_DEPLOY_POSITION,
            AT_INTERMEDIARY_POSITION,
            AT_PURPLE_PIXEL_DEPLOY_POSITION,
            AT_STACK_INTAKE_POSITION
        };
    }

    public static Params PARAMETERS = new Params();

    Params.ArmStates ArmState = Params.ArmStates.AT_PICK_UP_POSITION;

    private final ServoEx Arm;

    public ArmWithActions(HardwareMap hardwareMap){
        Arm = new SimpleServo(hardwareMap, "Arm", 0, 180, AngleUnit.DEGREES);
    }

    public class Arm_To_PickUp_Position implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if(ArmState != Params.ArmStates.AT_PICK_UP_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_Pick_Up_Position);
                }
                ArmState = Params.ArmStates.AT_PICK_UP_POSITION;
                return false;
        }
    }

    public class Arm_To_Deploy_Position implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if(ArmState != Params.ArmStates.AT_DEPLOY_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_Deploy_Position);
                }

                // false stops action
                ArmState = Params.ArmStates.AT_DEPLOY_POSITION;
                return false;
            }
        }


    public class Arm_To_HangSafe_Position implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(ArmState != Params.ArmStates.AT_HangSafe_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_HangSafe_Position);
                }
                // false stops action
                ArmState = Params.ArmStates.AT_HangSafe_POSITION;
                new SleepAction(1.5);
                return false;

        }
    }

    public class Arm_To_Intermediary_Position implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if(ArmState != Params.ArmStates.AT_INTERMEDIARY_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_Intermediary_Position);
                }

                // false stops action
                ArmState = Params.ArmStates.AT_INTERMEDIARY_POSITION;
                return false;
            }

    }

    public class Arm_To_Purple_Pixel_Deploy_Positon implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if(ArmState != Params.ArmStates.AT_PURPLE_PIXEL_DEPLOY_POSITION){
                    Arm.turnToAngle(PARAMETERS.Arm_Purple_Pixel_Deploy_Position);
                }
                // false stops action
                new SleepAction(1.5);
                ArmState = Params.ArmStates.AT_PURPLE_PIXEL_DEPLOY_POSITION;
                return false;

        }
    }

    public class Arm_To_Stack_Intake_Position implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


            if(ArmState != Params.ArmStates.AT_STACK_INTAKE_POSITION){
                Arm.turnToAngle(PARAMETERS.Arm_Stack_Intake_Position);
            }
            // false stops action
            ArmState = Params.ArmStates.AT_STACK_INTAKE_POSITION;
            return false;

        }
    }

    public SequentialAction Arm_To_PickUp_Position(){
        return new SequentialAction(
                new SleepAction(1.5),
                Arm_To_PickUp_Position()
        );
    }

    public SequentialAction Arm_To_Deploy_Position(){
        return new SequentialAction(
                new SleepAction(1.5),
                Arm_To_Deploy_Position()
        );
    }

    public Action Arm_To_HangSafe_Position(){
        return new Arm_To_HangSafe_Position();
    }


    public SequentialAction Arm_To_Intermediary_Position(){
        return new SequentialAction(
                new SleepAction(1.5),
                Arm_To_Intermediary_Position()
        );
    }
    public SequentialAction Arm_To_Stack_Pickup_Position(){
        return new SequentialAction(
                new SleepAction(1.5),
                Arm_To_Stack_Pickup_Position()
        );
    }
    public Action Arm_To_Purple_Pixel_Deploy_Positon(){
         return new Arm_To_Purple_Pixel_Deploy_Positon();
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
