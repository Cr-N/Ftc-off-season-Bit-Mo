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
        public boolean SLIDES_ARE_UNLOCKED=true;
        public int startPosition = 0;
        public int intakePosition = 0;
        public int hangPosition = 1250;
        public int LEVEL_1 = 200;
        public int LEVEL_2 = 550;
        public int LEVEL_3 = 1100;
        public int LEVEL_4 = 1300;
        public int leftPos = startPosition;
        public int rightPos = startPosition;
        public double speedOfSlides = 0.5;
        public double manualControlSpeed = 0.5;
        public double   P_Stanga =3;
        public  double P_Dreapta=3;
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
    private DcMotorEx Slider_DR = null; // this was just public idk if it breaks it
    private DcMotorEx Slider_ST = null; // this was just public idk if it breaks it

    public Slides(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        if (this.hardwareMap != null) {
            Slider_DR = hardwareMap.get(DcMotorEx.class , "Slider_DR");
            Slider_ST = hardwareMap.get(DcMotorEx.class , "Slider_ST");

            Slider_ST.setPositionPIDFCoefficients(PARAMETERS.P_Stanga);
            Slider_DR.setPositionPIDFCoefficients(PARAMETERS.P_Dreapta);

            Slider_DR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Slider_ST.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            Slider_ST.setDirection(DcMotorSimple.Direction.REVERSE);

            Slider_ST.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Slider_DR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Slider_ST.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slider_DR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
            PARAMETERS.rightPos = Slider_DR.getCurrentPosition(); // remember to update these in the teleOp!!!!!!
        } else {
            throw new NullPointerException("HardwareMap is null.");
        }
    }
    public void Handle_Manual_Control_UP(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){
            if(controlState == Params.ControlState.MANUAL_MODE)
            {
                if(PARAMETERS.rightPos < PARAMETERS.LEVEL_4 && PARAMETERS.leftPos < PARAMETERS.LEVEL_4)
                {
                    Slider_ST.setPower(PARAMETERS.manualControlSpeed);
                    Slider_DR.setPower(PARAMETERS.manualControlSpeed);
                }
            }
        }
    }
    public void Handle_Manual_Control_DOWN(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){

            if(controlState == Params.ControlState.MANUAL_MODE)
            {
                if(PARAMETERS.rightPos > PARAMETERS.LEVEL_1 && PARAMETERS.leftPos > PARAMETERS.LEVEL_1)
                {
                    Slider_ST.setPower(-PARAMETERS.manualControlSpeed);
                    Slider_DR.setPower(-PARAMETERS.manualControlSpeed);
                }
            }
        }
    }
    public void Handle_Stop_Motors_Manual_Control(){
        if(controlState == Params.ControlState.MANUAL_MODE){
            Slider_ST.setPower(0);
            Slider_DR.setPower(0);
        }
    }
    public void Change_Control_Mode(){
        if(controlState == Params.ControlState.LEVELS_MODE){
            controlState = Params.ControlState.MANUAL_MODE;
            Slider_DR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            Slider_ST.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
        else if(controlState == Params.ControlState.MANUAL_MODE){
            controlState = Params.ControlState.LEVELS_MODE;
            Slider_DR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            if(PARAMETERS.leftPos < 400){
                Move_To_LEVEL_1();
            }
            if(PARAMETERS.leftPos >=400 && PARAMETERS.leftPos <800) {
                Move_To__LEVEL_2();
            }
            if(PARAMETERS.leftPos >=800 && PARAMETERS.leftPos < 1125){
                Move_To_LEVEL_3();
            }
            if(PARAMETERS.leftPos >= 1125){
                Move_To_LEVEL_4();
            }
        }

    }
    public void Change_CONTROL_STATE_To_LEVELS_MODE(){
        if(controlState == Params.ControlState.MANUAL_MODE){
            controlState = Params.ControlState.LEVELS_MODE;
            Slider_DR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            if(PARAMETERS.leftPos < 400){
                Move_To_LEVEL_1();
                StateofSlides = Params.State_of_slides.AT_LEVEL_1;
            }
            if(PARAMETERS.leftPos >=400 && PARAMETERS.leftPos <800) {
                Move_To__LEVEL_2();
                StateofSlides = Params.State_of_slides.AT_LEVEL_2;
            }
            if(PARAMETERS.leftPos >=800 && PARAMETERS.leftPos < 1125){
                Move_To_LEVEL_3();
                StateofSlides = Params.State_of_slides.AT_LEVEL_3;
            }
            if(PARAMETERS.leftPos >= 1125){
                Move_To_LEVEL_4();
                StateofSlides = Params.State_of_slides.AT_LEVEL_4;
            }
        }
    }
    public void Change_CONTROL_STATE_To_MANUAL_MODE(){
        if(controlState == Params.ControlState.LEVELS_MODE){
            controlState = Params.ControlState.MANUAL_MODE;
            Slider_DR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            Slider_ST.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }
    public void Move_Sliders(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){

            Slider_ST.setTargetPosition(PARAMETERS.leftPos);
            Slider_DR.setTargetPosition(PARAMETERS.rightPos);

            Slider_DR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            Slider_ST.setPower(PARAMETERS.speedOfSlides);
            Slider_DR.setPower(PARAMETERS.speedOfSlides);
        }

    }
    public void Move_To_LEVEL_INTAKE_POSITION(){

        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){

            Slider_ST.setTargetPosition(PARAMETERS.intakePosition);
            Slider_DR.setTargetPosition(PARAMETERS.intakePosition);

            Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Slider_ST.setPower(PARAMETERS.speedOfSlides);
            Slider_DR.setPower(PARAMETERS.speedOfSlides);

            StateofSlides = Params.State_of_slides.INTAKE_POSITION;
        }
    }
    public void Move_To_LEVEL_1(){

        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){

            StateofSlides = Params.State_of_slides.AT_LEVEL_1;

            Slider_ST.setTargetPosition(PARAMETERS.LEVEL_1);
            Slider_DR.setTargetPosition(PARAMETERS.LEVEL_1);

            Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Slider_ST.setPower(PARAMETERS.speedOfSlides);
            Slider_DR.setPower(PARAMETERS.speedOfSlides);

            StateofSlides = Params.State_of_slides.AT_LEVEL_1;
        }
    }
    public void Move_To__LEVEL_2(){

        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){

            Slider_ST.setTargetPosition(PARAMETERS.LEVEL_2);
            Slider_DR.setTargetPosition(PARAMETERS.LEVEL_2);

            Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Slider_ST.setPower(PARAMETERS.speedOfSlides);
            Slider_DR.setPower(PARAMETERS.speedOfSlides);

            StateofSlides = Params.State_of_slides.AT_LEVEL_2;
        }
    }
    public void Move_To_LEVEL_3(){

        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){

            Slider_ST.setTargetPosition(PARAMETERS.LEVEL_3);
            Slider_DR.setTargetPosition(PARAMETERS.LEVEL_3);

            Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Slider_ST.setPower(PARAMETERS.speedOfSlides);
            Slider_DR.setPower(PARAMETERS.speedOfSlides);

            StateofSlides = Params.State_of_slides.AT_LEVEL_3;
        }
    }
    public void Move_To_LEVEL_4(){

        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){

            Slider_ST.setTargetPosition(PARAMETERS.LEVEL_4);
            Slider_DR.setTargetPosition(PARAMETERS.LEVEL_4);

            Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Slider_ST.setPower(PARAMETERS.speedOfSlides);
            Slider_DR.setPower(PARAMETERS.speedOfSlides);

            StateofSlides = Params.State_of_slides.AT_LEVEL_4;
        }
    }
    public void Move_To_LEVEL_Hang_Position(){

        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){

            Slider_ST.setTargetPosition(PARAMETERS.hangPosition);
            Slider_DR.setTargetPosition(PARAMETERS.hangPosition);

            Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Slider_ST.setPower(PARAMETERS.speedOfSlides);
            Slider_DR.setPower(PARAMETERS.speedOfSlides);

            StateofSlides = Params.State_of_slides.AT_LEVEL_4;
        }

    }
    /*public void updateCurrent_TARGETPositions(){
        PARAMETERS.leftPos = Slider_ST.getTargetPosition();
        PARAMETERS.rightPos = Slider_DR.getTargetPosition();
    }
    public void updateCurrent_CURRENTPositions(){
        PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
        PARAMETERS.rightPos = Slider_DR.getCurrentPosition();
    }*/
    public Params.ControlState Get_Control_State() {
        return controlState;
    }
    public Params.State_of_slides Get_State_of_Slides() {
        return StateofSlides;
    }

    /**
     * This is VERY IMPORTANT, ALWAYS USE THIS
     */
    public void UPDATE_Left_Right_Positions(){
        PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
        PARAMETERS.rightPos = Slider_DR.getCurrentPosition();
    }
    public void LOCK_SLIDES(){
        PARAMETERS.SLIDES_ARE_UNLOCKED = false;
    }
    public void UNLOCK_SLIDES(){
        PARAMETERS.SLIDES_ARE_UNLOCKED = true;
    }
    public int Get_Left_Pos(){
        return PARAMETERS.leftPos;
    }
    public int Get_Right_Pos(){
        return PARAMETERS.rightPos;
    }
}
