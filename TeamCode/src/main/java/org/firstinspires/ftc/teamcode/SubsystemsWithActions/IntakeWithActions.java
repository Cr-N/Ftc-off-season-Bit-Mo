package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakeWithActions {

    public static class Params{
        public double grab_position =43;
        public double pick_up_position = 18;
        public double DEPLOY_1 = 27;
        public double DEPLOY_2 = 18;
        public double claw_angle;
        public double lower_intake_error = 0.02;
        public double uppper_intake_error = 0.02;
        public double WAIT_TIME = 0.5;
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

                }
                // false stops action
                new SleepAction(PARAMETERS.WAIT_TIME);
                intakeState = Params.States.TO_INTAKE;
                return false;
        }
    }

    public class GRAB implements Action{

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(intakeState == Params.States.TO_INTAKE){
                    claw.turnToAngle(PARAMETERS.grab_position);

                }
                new SleepAction(PARAMETERS.WAIT_TIME);
                intakeState = Params.States.HAS_INTAKED;
                return false;
        }
    }

    public class DEPLOY_1 implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(intakeState == Params.States.HAS_INTAKED){
                    claw.turnToAngle(PARAMETERS.DEPLOY_1);

                }
                new SleepAction(PARAMETERS.WAIT_TIME);
                intakeState = Params.States.DEPLOYED_1;
                return false;

        }
    }

    public class DEPLOY_2 implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(intakeState == Params.States.DEPLOYED_1){
                    claw.turnToAngle(PARAMETERS.DEPLOY_2);
                }
                // false stops action
            new SleepAction(PARAMETERS.WAIT_TIME);
            intakeState = Params.States.DEPLOYED_2;
            return false;
        }
    }

    public Action PICKUP(){
        return new PICKUP();
    }

    public Action GRAB(){
             return new GRAB();
    }

    public Action DEPLOY_1(){
        return new DEPLOY_1();
    }

    public Action DEPLOY_2(){
        return new DEPLOY_2();
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
