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

public class IntakeWithActions {

    public static class Params{
        public double grab_position =43;
        public double pick_up_position = 18;
        public double DEPLOY_1 = 25;
        public double DEPLOY_2 = 18;
        public double claw_angle;
        public double lower_intake_error = 0.02;
        public double uppper_intake_error = 0.02;
        public enum States{
            DEPLOYED_1,
            DEPLOYED_2,
            TO_INTAKE,
            HAS_INTAKED


        };
    }

    public static Params PARAMETERS = new Params();

    Params.States intakeState = Params.States.TO_INTAKE;

    private final ServoEx claw;

    public IntakeWithActions(HardwareMap hardwareMap){
        claw = new SimpleServo(hardwareMap, "claw", 0, 270, AngleUnit.DEGREES);
    }

    public class PICKUP implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if(intakeState == Params.States.TO_INTAKE || intakeState == Params.States.DEPLOYED_2 || intakeState == Params.States.DEPLOYED_1){
                    claw.turnToAngle(PARAMETERS.pick_up_position);
                    intakeState = Params.States.TO_INTAKE;

                }

                // false stops action
                return false;
        }
    }

    public class GRAB implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(intakeState == Params.States.TO_INTAKE){
                    claw.turnToAngle(PARAMETERS.grab_position);
                    intakeState = Params.States.HAS_INTAKED;

                }
                new SleepAction(1);
                // false stops action
                return false;
        }
    }

    public class DEPLOY_1 implements Action{

        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if(intakeState == Params.States.HAS_INTAKED){
                    claw.turnToAngle(PARAMETERS.DEPLOY_1);
                    intakeState = Params.States.DEPLOYED_1;

                }

                // false stops action
                return false;

        }
    }

    public class DEPLOY_2 implements Action{


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {


                if(intakeState == Params.States.DEPLOYED_1){
                    claw.turnToAngle(PARAMETERS.DEPLOY_2);
                    intakeState = Params.States.DEPLOYED_2;

                }

                // false stops action
                return false;

        }
    }

    public SequentialAction PICKUP(){
        return new SequentialAction(
                new SleepAction(0.5),
                PICKUP()
        );
    }

    public Action GRAB(){
             return new GRAB();
    }

    public SequentialAction DEPLOY_1(){
        return new SequentialAction(
                new SleepAction(0.5),
                DEPLOY_1()
        );    }

    public SequentialAction DEPLOY_2(){
        return new SequentialAction(
                new SleepAction(0.5),
                DEPLOY_2()
        );
    }

    public void Handle_Intaking(){
        switch (intakeState){
            case TO_INTAKE:
                GRAB();
                break;
            case HAS_INTAKED:
                DEPLOY_1();
                break;
            case DEPLOYED_1:
                DEPLOY_2();
                break;
            case DEPLOYED_2:
                PICKUP();
                break;
        }
    }

    public Params.States Get_Intake_State(){
        return intakeState;
    }

}
