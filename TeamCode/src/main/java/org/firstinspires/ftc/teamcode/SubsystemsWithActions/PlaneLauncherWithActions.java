package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PlaneLauncherWithActions implements Subsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;

    public static class Params{
        public double start_position=155;
        public double lauch_position=250;
        public enum States{
            LAUNCHED,
            NOT_LAUNCHED
        };
    }
    public static Params PARAMETERS = new Params();
    Params.States planeState = Params.States.NOT_LAUNCHED;
    public final ServoEx laucher;

    //public Intake(LinearOpMode opMode){
    //  myOpmode = opMode;
    //this.hardwareMap = myOpmode.hardwareMap;
    //claw = new SimpleServo(hardwareMap,"claw",0,270, AngleUnit.DEGREES);
    //}
    public PlaneLauncherWithActions(HardwareMap hardwareMap) {
            laucher = new SimpleServo(hardwareMap, "launcher", 0, 270, AngleUnit.DEGREES);
    }


    public class LAUCH_PLANE implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(planeState == Params.States.NOT_LAUNCHED)
                laucher.turnToAngle(PARAMETERS.lauch_position);
            new SleepAction(0.5);
            planeState = Params.States.LAUNCHED;
            return false;
        }
    }
    public class PREP_PLANE implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            laucher.turnToAngle(PARAMETERS.start_position);
            new SleepAction(0.5);
            planeState = Params.States.NOT_LAUNCHED;
            return false;
        }
    }
    public Action LAUCH_PLANE(){
        return new LAUCH_PLANE();
    }
    public Action PREP_PLANE(){
        return new PREP_PLANE();
    }

    public Params.States Get_Plane_State(){
        return planeState;
    }
}
