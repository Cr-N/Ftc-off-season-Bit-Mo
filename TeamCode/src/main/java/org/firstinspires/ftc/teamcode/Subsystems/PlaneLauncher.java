package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class PlaneLauncher implements Subsystem {
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
    private final ServoEx laucher;

    //public Intake(LinearOpMode opMode){
    //  myOpmode = opMode;
    //this.hardwareMap = myOpmode.hardwareMap;
    //claw = new SimpleServo(hardwareMap,"claw",0,270, AngleUnit.DEGREES);
    //}
    public PlaneLauncher(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        if (this.hardwareMap != null) {
            laucher = new SimpleServo(hardwareMap, "launcher", 0, 270, AngleUnit.DEGREES);
        } else {
            throw new NullPointerException("HardwareMap is null.");
        }
    }


    public void LAUCH_PLANE(){
       if(planeState == Params.States.NOT_LAUNCHED){
            laucher.turnToAngle(PARAMETERS.lauch_position,AngleUnit.DEGREES);
            planeState = Params.States.LAUNCHED;
       }
    }

    public void REST_PLANE(){
            laucher.turnToAngle(PARAMETERS.start_position,AngleUnit.DEGREES);
            planeState = Params.States.NOT_LAUNCHED;

    }
    public Params.States Get_Plane_State(){
        return planeState;
    }
}
