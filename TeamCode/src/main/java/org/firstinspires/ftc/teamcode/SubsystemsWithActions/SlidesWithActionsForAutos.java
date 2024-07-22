package org.firstinspires.ftc.teamcode.SubsystemsWithActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
// This is the Implementation for Autonomous
public class SlidesWithActionsForAutos {

    public static class Params{
        public boolean SLIDES_ARE_UNLOCKED=true;
        public int startPosition = 0;
        public int intakePosition = 0;
        public int LEVEL_1 = 200;
        public int LEVEL_2 = 550;
        public int LEVEL_3 = 1100;
        public int LEVEL_4 = 1300;
        public int leftPos = startPosition;
        public int rightPos = startPosition;
        public double speedOfSlides = 0.5;
        public double leftSlideError=1;
        public double righSlideError=3  ;
        public double P_Stanga =3;
        public  double P_Dreapta=3;
        public enum State_of_slides{
            INTAKE_POSITION,
            AT_LEVEL_1,
            AT_LEVEL_2,
            AT_LEVEL_3,
            AT_LEVEL_4,
        };
    }
    public static Params PARAMETERS = new Params();
    Params.State_of_slides StateofSlides = Params.State_of_slides.INTAKE_POSITION;
    private DcMotorEx Slider_DR = null; // this was just public idk if it breaks it
    private DcMotorEx Slider_ST = null; // this was just public idk if it breaks it

    public SlidesWithActionsForAutos(HardwareMap hardwareMap) {

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

    }

    public class FORAUTO_Move_To_LEVEL_INTAKE_POSITION implements Action{

        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(initialized == false){
                Slider_ST.setTargetPosition(PARAMETERS.intakePosition);
                Slider_DR.setTargetPosition(PARAMETERS.intakePosition);

                Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slider_ST.setPower(PARAMETERS.speedOfSlides);
                Slider_DR.setPower(PARAMETERS.speedOfSlides);

                initialized = true;
            }

            PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
            PARAMETERS.rightPos = Slider_DR.getCurrentPosition();

            telemetryPacket.put("left slider pos ",PARAMETERS.leftPos);
            telemetryPacket.put("right slider pos ",PARAMETERS.rightPos);
            telemetryPacket.put("sliders state ", StateofSlides);

            if( (PARAMETERS.leftPos < PARAMETERS.intakePosition - PARAMETERS.leftSlideError && PARAMETERS.rightPos < PARAMETERS.intakePosition - PARAMETERS.righSlideError) || (PARAMETERS.leftPos > PARAMETERS.intakePosition + PARAMETERS.leftSlideError && PARAMETERS.rightPos > PARAMETERS.intakePosition + PARAMETERS.righSlideError) ){
                // return true reruns action
                return true;
            }
            else{
                StateofSlides = Params.State_of_slides.INTAKE_POSITION;
                return false;
            }
        }
    }

    public class FORAUTO_Move_To_LEVEL_1 implements Action{

        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                Slider_ST.setTargetPosition(PARAMETERS.LEVEL_1);
                Slider_DR.setTargetPosition(PARAMETERS.LEVEL_1);

                Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slider_ST.setPower(PARAMETERS.speedOfSlides);
                Slider_DR.setPower(PARAMETERS.speedOfSlides);

                initialized = true;
            }

            PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
            PARAMETERS.rightPos = Slider_DR.getCurrentPosition();

            telemetryPacket.put("left slider pos ",PARAMETERS.leftPos);
            telemetryPacket.put("right slider pos ",PARAMETERS.rightPos);
            telemetryPacket.put("sliders state ", StateofSlides);

            if( (PARAMETERS.leftPos < PARAMETERS.LEVEL_1 - PARAMETERS.leftSlideError && PARAMETERS.rightPos < PARAMETERS.LEVEL_1 - PARAMETERS.righSlideError) || (PARAMETERS.leftPos > PARAMETERS.LEVEL_1 + PARAMETERS.leftSlideError && PARAMETERS.rightPos > PARAMETERS.LEVEL_1 + PARAMETERS.righSlideError) ){
                return true;
            }
            else{
                StateofSlides = Params.State_of_slides.AT_LEVEL_1;
                return false;
            }
        }
    }

    public class FORAUTO_Move_To_LEVEL_2 implements Action{

        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                Slider_ST.setTargetPosition(PARAMETERS.LEVEL_2);
                Slider_DR.setTargetPosition(PARAMETERS.LEVEL_2);

                Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slider_ST.setPower(PARAMETERS.speedOfSlides);
                Slider_DR.setPower(PARAMETERS.speedOfSlides);

                initialized = true;
            }

            PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
            PARAMETERS.rightPos = Slider_DR.getCurrentPosition();

            telemetryPacket.put("left slider pos ",PARAMETERS.leftPos);
            telemetryPacket.put("right slider pos ",PARAMETERS.rightPos);
            telemetryPacket.put("slider states ", StateofSlides);

            if((PARAMETERS.leftPos < PARAMETERS.LEVEL_2 - PARAMETERS.leftSlideError && PARAMETERS.rightPos < PARAMETERS.LEVEL_2 - PARAMETERS.righSlideError) || (PARAMETERS.leftPos > PARAMETERS.LEVEL_2 + PARAMETERS.leftSlideError && PARAMETERS.rightPos > PARAMETERS.LEVEL_2 + PARAMETERS.righSlideError) ){
                return true;
            }
            else{
                StateofSlides = Params.State_of_slides.AT_LEVEL_2;
                return false;
            }
        }
    }

    public class FORAUTO_Move_To_LEVEL_3 implements Action{

        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                Slider_ST.setTargetPosition(PARAMETERS.LEVEL_3);
                Slider_DR.setTargetPosition(PARAMETERS.LEVEL_3);

                Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slider_ST.setPower(PARAMETERS.speedOfSlides);
                Slider_DR.setPower(PARAMETERS.speedOfSlides);

                initialized = true;
            }

            PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
            PARAMETERS.rightPos = Slider_DR.getCurrentPosition();

            telemetryPacket.put("left slider pos ",PARAMETERS.leftPos);
            telemetryPacket.put("right slider pos ",PARAMETERS.rightPos);
            telemetryPacket.put("slider states ", StateofSlides);

            if((PARAMETERS.leftPos < PARAMETERS.LEVEL_3 - PARAMETERS.leftSlideError && PARAMETERS.rightPos < PARAMETERS.LEVEL_3 - PARAMETERS.righSlideError) || (PARAMETERS.leftPos > PARAMETERS.LEVEL_3 + PARAMETERS.leftSlideError && PARAMETERS.rightPos > PARAMETERS.LEVEL_3 + PARAMETERS.righSlideError) ){
                return true;
            }
            else{
                StateofSlides = Params.State_of_slides.AT_LEVEL_3;
                return false;
            }
        }
    }

    public class FORAUTO_Move_To_LEVEL_4 implements Action{

        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){

                Slider_ST.setTargetPosition(PARAMETERS.LEVEL_4);
                Slider_DR.setTargetPosition(PARAMETERS.LEVEL_4);

                Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                Slider_ST.setPower(PARAMETERS.speedOfSlides);
                Slider_DR.setPower(PARAMETERS.speedOfSlides);

                initialized = true;
            }

            PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
            PARAMETERS.rightPos = Slider_DR.getCurrentPosition();

            telemetryPacket.put("left slider pos ",PARAMETERS.leftPos);
            telemetryPacket.put("right slider pos ",PARAMETERS.rightPos);
            telemetryPacket.put("slider states ", StateofSlides);

            if((PARAMETERS.leftPos < PARAMETERS.LEVEL_4 - PARAMETERS.leftSlideError && PARAMETERS.rightPos < PARAMETERS.LEVEL_4 - PARAMETERS.righSlideError) || (PARAMETERS.leftPos > PARAMETERS.LEVEL_4 + PARAMETERS.leftSlideError && PARAMETERS.rightPos > PARAMETERS.LEVEL_4 + PARAMETERS.righSlideError) ){
                return true;
            }
            else{
                StateofSlides = Params.State_of_slides.AT_LEVEL_4;
                return false;
            }
        }
    }

    public class UNLOCK_SLIDES implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            PARAMETERS.SLIDES_ARE_UNLOCKED = true;
            return false;
        }
    }

    public class LOCK_SLIDES implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            PARAMETERS.SLIDES_ARE_UNLOCKED = false;
            return false;
        }
    }

    public Action FORAUTO_Move_To_LEVEL_INTAKE_POSITION(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){
            return new FORAUTO_Move_To_LEVEL_INTAKE_POSITION();
        }
        else{
            throw new RuntimeException("Slides are locked! Unlock the slides before attempting to move them!!!!!!!");
        }
    }
    public Action FORAUTO_Move_To_LEVEL_1(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){
            return new FORAUTO_Move_To_LEVEL_1();
        }
        else{
            throw new RuntimeException("Slides are locked! Unlock the slides before attempting to move them!!!!!!!");
        }
    }
    public Action FORAUTO_Move_To_LEVEL_2(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){
            return new FORAUTO_Move_To_LEVEL_2();
        }
        else{
            throw new RuntimeException("Slides are locked! Unlock the slides before attempting to move them!!!!!!!");
        }
    }
    public Action FORAUTO_Move_To_LEVEL_3(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){
            return new FORAUTO_Move_To_LEVEL_3();
        }
        else{
            throw new RuntimeException("Slides are locked! Unlock the slides before attempting to move them!!!!!!!");
        }
    }
    public Action FORAUTO_Move_To_LEVEL_4(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true){
            return new FORAUTO_Move_To_LEVEL_4();
        }
        else{
            throw new RuntimeException("Slides are locked! Unlock the slides before attempting to move them!!!!!!!");
        }
    }
    public Params.State_of_slides getStateofSlides() {
        return StateofSlides;
    }
    public Action UNLOCK_SLIDES(){
        return new UNLOCK_SLIDES();
    }
    public Action LOCK_SLIDES(){
        return new LOCK_SLIDES();
    }
    public Action CHANGE_SLIDES_UNLOCK_STATE(){
        if(PARAMETERS.SLIDES_ARE_UNLOCKED == true)
            return new LOCK_SLIDES();
        else
            return new UNLOCK_SLIDES();

    }
}
