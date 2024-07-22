package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardWare.SpecialGamepad;

public class Intake implements Subsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;

    public static class Params{
        public double grab_position =43;
        public double pick_up_position = 18;
        public double DEPLOY_1 = 25;
        public double DEPLOY_2 = 18;
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


    public void PICKUP(){
        if(intakeState == Params.States.TO_INTAKE || intakeState == Params.States.DEPLOYED_2 || intakeState == Params.States.DEPLOYED_1){
            claw.turnToAngle(PARAMETERS.pick_up_position);
            intakeState = Params.States.TO_INTAKE;
        }
    }
    public void GRAB(){
        if(intakeState == Params.States.TO_INTAKE){
            claw.turnToAngle(PARAMETERS.grab_position);
            intakeState = Params.States.HAS_INTAKED;
        }
    }
    public void DEPLOY_1(){
        if(intakeState == Params.States.HAS_INTAKED){
            claw.turnToAngle(PARAMETERS.DEPLOY_1);
            intakeState = Params.States.DEPLOYED_1;
            myOpmode.gamepad1.setLedColor(255, 0, 0, 120000);//red light
        }

    }
    public void DEPLOY_2(){
        if(intakeState == Params.States.DEPLOYED_1){
            claw.turnToAngle(PARAMETERS.DEPLOY_2);
            intakeState = Params.States.TO_INTAKE;
            myOpmode.gamepad1.setLedColor(0, 255, 0, 120000);//green light
        }
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
