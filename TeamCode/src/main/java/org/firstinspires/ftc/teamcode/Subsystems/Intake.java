package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Intake implements Subsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;
    public static class Params{
        public double intakePosition = 19;
        public double DEPLOY_1 = 12;
        public double DEPLOY_2 = 19;
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

    //public Intake(LinearOpMode opMode){
      //  myOpmode = opMode;
        //this.hardwareMap = myOpmode.hardwareMap;
        //claw = new SimpleServo(hardwareMap,"claw",0,270, AngleUnit.DEGREES);
    //}
    public Intake(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        if (this.hardwareMap != null) {
            claw = new SimpleServo(hardwareMap, "claw", 0, 270, AngleUnit.DEGREES);
        } else {
            throw new NullPointerException("HardwareMap is null.");
        }
    }


    public void pickUp(){
        if(intakeState == Params.States.TO_INTAKE || intakeState == Params.States.DEPLOYED_2 || intakeState == Params.States.DEPLOYED_1){
            claw.turnToAngle(PARAMETERS.intakePosition);
            intakeState = Params.States.HAS_INTAKED;
        }
    }
    public void DEPLOY_1(){
        if(intakeState == Params.States.HAS_INTAKED){
            claw.turnToAngle(PARAMETERS.DEPLOY_1);
            intakeState = Params.States.DEPLOYED_1;
        }

    }
    public void DEPLOY_2(){
        if(intakeState == Params.States.DEPLOYED_1){
            claw.turnToAngle(PARAMETERS.DEPLOY_2);
            intakeState = Params.States.DEPLOYED_2;
        }
    }
    public void restartIntake(){
        if(intakeState == Params.States.DEPLOYED_2){
            claw.turnToAngle(PARAMETERS.intakePosition);
            intakeState = Params.States.TO_INTAKE;
        }
    }
    public Params.States getIntakeState(){
        return intakeState;
    }
}
