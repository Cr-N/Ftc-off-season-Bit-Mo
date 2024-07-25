package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@Config
@TeleOp
public class SliderTest extends LinearOpMode {

    public DcMotorEx Slider_DR = null;
    public DcMotorEx Slider_ST = null;
    public static class Params {
        public  int ManualMode = 0;
        public int startPosition = 0;
        public int moveBy = 100;
        public int LEVEL_1 = 200;
        public int LEVEL_2 = 600;
        public int LEVEL_3 = 1000;
        public int LEVEL_4 = 1250;
        public int intakePosition=0;
        public int nivel =0;
        public int leftPos = startPosition;
        public int rightPos = startPosition;
        public double speedOfSlides = 0.5;
        public double modify = 1; // 0.975
        public double   P_Stanga =3;
        public  double P_Dreapta=3;
    }
    boolean lastDpadUp = false , currentDpadUp,lastDpadDown=false,currentDpadDown , lastX = false,currentX,lastTriangle=false,currentTriangle,lastBack=false,currentBack;
    public static Params PARAMETERS = new Params();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx gm1 = new GamepadEx(gamepad1);
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

        waitForStart();

        while (opModeIsActive()) {
            {
                currentBack = gm1.isDown(GamepadKeys.Button.BACK);
                currentDpadDown = gm1.isDown(GamepadKeys.Button.DPAD_DOWN);
                currentDpadUp = gm1.isDown(GamepadKeys.Button.DPAD_UP);
                currentX = gm1.isDown(GamepadKeys.Button.A);
                currentTriangle = gm1.isDown(GamepadKeys.Button.Y);
            }


            PARAMETERS.leftPos = Slider_ST.getCurrentPosition();
            PARAMETERS.rightPos = Slider_DR.getCurrentPosition();

            if(currentBack && !lastBack)
            {
                if(PARAMETERS.ManualMode == 0)
                {
                    PARAMETERS.ManualMode = 1;
                    Slider_DR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Slider_ST.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                else if(PARAMETERS.ManualMode == 1)
                {
                    PARAMETERS.ManualMode = 0;
                    Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(PARAMETERS.leftPos < 400){
                        moveToLEVEL_1();
                        PARAMETERS.nivel=1;
                    }
                    if(PARAMETERS.leftPos >400 && PARAMETERS.leftPos <800) {
                        moveToLEVEL_2();
                        PARAMETERS.nivel = 2;
                    }
                    if(PARAMETERS.leftPos >800 && PARAMETERS.leftPos < 1125){
                        moveToLEVEL_3();
                        PARAMETERS.nivel=3;

                    }

                    if(PARAMETERS.leftPos > 1125){
                        moveToLEVEL_4();
                        PARAMETERS.nivel=4;
                    }

                }

            }
            if (currentDpadUp && !lastDpadUp && PARAMETERS.ManualMode==0) {

                switch (PARAMETERS.nivel) {
                    case 0:
                        moveToLEVEL_1();
                        break;
                    case 1:
                        moveToLEVEL_2();
                        break;
                    case 2:
                        moveToLEVEL_3();
                        break;
                    case 3:
                        moveToLEVEL_4();
                        break;
                }
                if(PARAMETERS.nivel <4)
                    PARAMETERS.nivel +=1;

            }
            if (currentDpadDown && !lastDpadDown && PARAMETERS.ManualMode == 0) {
                switch (PARAMETERS.nivel){
                    case 4:
                        moveToLEVEL_3();
                        break;
                    case 3:
                        moveToLEVEL_2();
                        break;
                    case 2:
                        moveToLEVEL_1();
                        break;
                    case 1:
                        moveToLEVEL_IntakePosition();
                        break;
                }
                if(PARAMETERS.nivel >=1)
                    PARAMETERS.nivel -=1;
            }
            if(currentX && PARAMETERS.ManualMode == 1 && PARAMETERS.leftPos > PARAMETERS.LEVEL_1 && PARAMETERS.rightPos > PARAMETERS.LEVEL_1 ) {
                Slider_ST.setPower(-PARAMETERS.speedOfSlides);
                Slider_DR.setPower(-PARAMETERS.speedOfSlides);
            }
            if(currentTriangle && PARAMETERS.ManualMode == 1 && PARAMETERS.leftPos < PARAMETERS.LEVEL_4 && PARAMETERS.rightPos < PARAMETERS.LEVEL_4){
                Slider_ST.setPower(PARAMETERS.speedOfSlides);
                Slider_DR.setPower(PARAMETERS.speedOfSlides);
            }
            if(!currentX && !currentTriangle && PARAMETERS.ManualMode==1 )
            {
                Slider_ST.setPower(0);
                Slider_DR.setPower(0);
            }


            telemetry.addData("LeftPos:", Slider_ST.getCurrentPosition());
            telemetry.addData("RightPos:", Slider_DR.getCurrentPosition());
            telemetry.addData("Diferenta Stanga - Dreapta: ",Slider_ST.getCurrentPosition() - Slider_DR.getCurrentPosition());
            telemetry.addData("Diferenta Dreapta - Stanga: ",Slider_DR.getCurrentPosition() - Slider_ST.getCurrentPosition());
            telemetry.update();

            {
                lastDpadUp = currentDpadUp;
                lastDpadDown = currentDpadDown;
                lastX = currentX;
                lastTriangle = currentTriangle;
                lastBack = currentBack;
            }

        }
    }
    public void MoveSliders(){
        Slider_ST.setTargetPosition(PARAMETERS.leftPos);
        Slider_DR.setTargetPosition((int) (PARAMETERS.rightPos*PARAMETERS.modify));

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_IntakePosition(){
        Slider_ST.setTargetPosition(PARAMETERS.intakePosition);
        Slider_DR.setTargetPosition(PARAMETERS.intakePosition);

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_1(){
        Slider_ST.setTargetPosition(PARAMETERS.LEVEL_1);
        Slider_DR.setTargetPosition(PARAMETERS.LEVEL_1);

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_2(){
        Slider_ST.setTargetPosition(PARAMETERS.LEVEL_2);
        Slider_DR.setTargetPosition(PARAMETERS.LEVEL_2);


        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_3(){
        Slider_ST.setTargetPosition(PARAMETERS.LEVEL_3);
        Slider_DR.setTargetPosition(PARAMETERS.LEVEL_3);


        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
    public void moveToLEVEL_4(){
        Slider_ST.setTargetPosition(PARAMETERS.LEVEL_4);
        Slider_DR.setTargetPosition(PARAMETERS.LEVEL_4);

        Slider_DR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slider_ST.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Slider_ST.setPower(PARAMETERS.speedOfSlides);
        Slider_DR.setPower(PARAMETERS.speedOfSlides);
    }
}
