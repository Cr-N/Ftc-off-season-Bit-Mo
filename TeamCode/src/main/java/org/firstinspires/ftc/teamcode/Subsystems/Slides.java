package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides implements Subsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;

    public static class Params{
        public int startPosition = 0;
        public int moveBy = 100;
        public int intakePosition = 50;
        public int hangPosition = 1100;
        public int LEVEL_1 = 200;
        public int LEVEL_2 = 600;
        public int LEVEL_3 = 1000;
        public int LEVEL_4 = 1250;
        public int leftPos = startPosition;
        public int rightPos = startPosition;
        public double speedOfSlides = 0.5;
        public double manualControlSpeed = 0.2;
        public int MAXIMUM_POSITION = 1350;
        public enum State_of_slides{
            INTAKE_POSITION,
            HANG_POSITION,
            AT_LEVEL_1,
            AT_LEVEL_2,
            AT_LEVEL_3,
            AT_LEVEL_4,
        };
        public enum ControlState{
            LEVELS_MODE,
            MANUAL_MODE
        }
    }
    public static Params PARAMETERS = new Params();
    Params.ControlState controlState = Params.ControlState.LEVELS_MODE;
    Params.State_of_slides StateofSlides = Params.State_of_slides.INTAKE_POSITION;
    public DcMotorEx Slider_DR = null;
    public DcMotorEx Slider_ST = null;

    public Slides(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        if (this.hardwareMap != null) {
            Slider_DR = hardwareMap.get(DcMotorEx.class , "Slider_DR");
            Slider_ST = hardwareMap.get(DcMotorEx.class , "Slider_ST");
            Slider_DR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slider_ST.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slider_ST.setDirection(DcMotorSimple.Direction.REVERSE);
            Slider_ST.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slider_DR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            throw new NullPointerException("HardwareMap is null.");
        }
    }
    public void handleManualControlUP(){
        if(controlState == Params.ControlState.MANUAL_MODE)
        {
            if(Slider_DR.getCurrentPosition() < PARAMETERS.MAXIMUM_POSITION && Slider_ST.getCurrentPosition() < PARAMETERS.MAXIMUM_POSITION)
            {
                Slider_ST.setPower(PARAMETERS.manualControlSpeed);
                Slider_DR.setPower(PARAMETERS.manualControlSpeed);
            }
        }
    }
    public void handleManualControlDOWN(){
        if(controlState == Params.ControlState.MANUAL_MODE)
        {
            if(Slider_DR.getCurrentPosition() > PARAMETERS.startPosition && Slider_ST.getCurrentPosition() < PARAMETERS.startPosition)
            {
                Slider_ST.setPower(-PARAMETERS.manualControlSpeed);
                Slider_DR.setPower(-PARAMETERS.manualControlSpeed);
            }
        }
    }
    public void handleStopMotorsManualControl(){
        if(controlState == Params.ControlState.MANUAL_MODE){
            Slider_ST.setPower(0);
            Slider_DR.setPower(0);
        }
    }
    public void changeControlMode(){
        if(controlState == Params.ControlState.LEVELS_MODE){
            controlState = Params.ControlState.MANUAL_MODE;
            Slider_DR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            Slider_ST.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        else if(controlState == Params.ControlState.MANUAL_MODE){
            controlState = Params.ControlState.LEVELS_MODE;
            Slider_DR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }

    }
    public void MoveSliders(){
        Slider_ST.setTargetPosition(PARAMETERS.leftPos);
        Slider_DR.setTargetPosition(PARAMETERS.rightPos);

        Slider_DR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void handleLevelsUP(){
        if(controlState == Params.ControlState.LEVELS_MODE)
        {
            switch (StateofSlides) {
                case INTAKE_POSITION:
                    moveToLEVEL_1();
                    StateofSlides = Params.State_of_slides.AT_LEVEL_1;
                    break;
                case AT_LEVEL_1:
                    moveToLEVEL_2();
                    StateofSlides = Params.State_of_slides.AT_LEVEL_2;
                    break;
                case AT_LEVEL_2:
                    moveToLEVEL_3();
                    StateofSlides = Params.State_of_slides.AT_LEVEL_3;
                    break;
                case AT_LEVEL_3:
                    moveToLEVEL_4();
                    StateofSlides = Params.State_of_slides.AT_LEVEL_4;
                    break;

            }
        }
    }
    public void handleLevelsDOWN(){
        if(controlState == Params.ControlState.LEVELS_MODE)
        {
            switch (StateofSlides) {
                case AT_LEVEL_1:
                    moveToLEVEL_IntakePosition();
                    StateofSlides = Params.State_of_slides.INTAKE_POSITION;
                    break;
                case AT_LEVEL_2:
                    moveToLEVEL_1();
                    StateofSlides = Params.State_of_slides.AT_LEVEL_1;
                    break;
                case AT_LEVEL_3:
                    moveToLEVEL_2();
                    StateofSlides = Params.State_of_slides.AT_LEVEL_2;
                    break;
                case AT_LEVEL_4:
                    moveToLEVEL_3();
                    StateofSlides = Params.State_of_slides.AT_LEVEL_3;
                    break;
            }
        }
    }
    public void moveToLEVEL_IntakePosition(){
        Slider_ST.setTargetPosition(PARAMETERS.intakePosition);
        Slider_DR.setTargetPosition(PARAMETERS.intakePosition);

        updateCurrent_TARGETPositions();

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_1(){
        Slider_ST.setTargetPosition(PARAMETERS.LEVEL_1);
        Slider_DR.setTargetPosition(PARAMETERS.LEVEL_1);

        updateCurrent_TARGETPositions();

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_2(){
        Slider_ST.setTargetPosition(PARAMETERS.LEVEL_2);
        Slider_DR.setTargetPosition(PARAMETERS.LEVEL_2);

        updateCurrent_TARGETPositions();

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_3(){
        Slider_ST.setTargetPosition(PARAMETERS.LEVEL_3);
        Slider_DR.setTargetPosition(PARAMETERS.LEVEL_3);

        updateCurrent_TARGETPositions();

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_4(){
        Slider_ST.setTargetPosition(PARAMETERS.LEVEL_4);
        Slider_DR.setTargetPosition(PARAMETERS.LEVEL_4);

        updateCurrent_TARGETPositions();

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_HangPosition(){
        Slider_ST.setTargetPosition(PARAMETERS.hangPosition);
        Slider_DR.setTargetPosition(PARAMETERS.hangPosition);

        updateCurrent_TARGETPositions();

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void updateCurrent_TARGETPositions(){
        PARAMETERS.leftPos = Slider_ST.getTargetPosition();
        PARAMETERS.rightPos = Slider_DR.getTargetPosition();
    }
    public void updateCurrent_CURRENTPositions(){
        PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
        PARAMETERS.rightPos = Slider_DR.getCurrentPosition();
    }
}
