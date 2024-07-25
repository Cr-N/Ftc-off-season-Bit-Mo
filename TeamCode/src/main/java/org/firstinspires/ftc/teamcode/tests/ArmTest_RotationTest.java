package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@Config
@TeleOp
public class ArmTest_RotationTest extends LinearOpMode {
    public static class Params{
        public double SET_CLAW_POSITION=19;
        public double SET_ARM_POSITION=30;
        public double SET_ROTATE_POSITION=120;
        public double moveBy=10;
        public double Arm_Intake_Position =0;
        public double Arm_Deploy_Position = 100;
        public double Arm_HangSafe_Position = 50;
        public double ArmPos=0;
        public double RotatePos=0;
        public double ClawPos =0;
        public double Rotate_Intake_Position = 0;
        public double Rotate_Deploy_Position =100;
        public double Rotate_HangSafe_Position=50;

    }
    public static Params PARAMETERS = new Params();

    public ServoEx Arm;
    public  ServoEx Rotate;
    public  ServoEx Claw;
    boolean lastDpadUp = false , currentDpadUp,lastDpadDown=false,currentDpadDown , lastX = false,currentX,lastTriangle=false,currentTriangle,lastBack=false,currentBack,lastRB=false,currentRB,lastLB=false,currentLB;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx gm1 = new GamepadEx(gamepad1);
        Arm = new SimpleServo(hardwareMap, "Arm", 0, 180, AngleUnit.DEGREES);
        Rotate = new SimpleServo(hardwareMap, "Rotate", 0, 180, AngleUnit.DEGREES);
        Claw = new SimpleServo(hardwareMap, "claw", 0, 270, AngleUnit.DEGREES);
        Arm.setInverted(false);
        Arm.turnToAngle(PARAMETERS.SET_ARM_POSITION);
        Rotate.turnToAngle(PARAMETERS.SET_ROTATE_POSITION);
        Claw.turnToAngle(PARAMETERS.SET_CLAW_POSITION);
        waitForStart();
        while(opModeIsActive()){
            {
                currentBack = gm1.isDown(GamepadKeys.Button.BACK);
                currentDpadDown = gm1.isDown(GamepadKeys.Button.DPAD_DOWN);
                currentDpadUp = gm1.isDown(GamepadKeys.Button.DPAD_UP);
                currentX = gm1.isDown(GamepadKeys.Button.A);
                currentTriangle = gm1.isDown(GamepadKeys.Button.Y);
                currentRB = gm1.isDown(GamepadKeys.Button.RIGHT_BUMPER);
                currentLB = gm1.isDown(GamepadKeys.Button.LEFT_BUMPER);
            }

            if(currentDpadUp && !lastDpadUp){
                PARAMETERS.ArmPos += PARAMETERS.moveBy;
                Arm.turnToAngle(PARAMETERS.ArmPos);
            }
            if(currentDpadDown && !lastDpadDown){
                PARAMETERS.ArmPos -= PARAMETERS.moveBy;
                Arm.turnToAngle(PARAMETERS.ArmPos);
            }
            if(currentTriangle && !lastTriangle){
                PARAMETERS.RotatePos += PARAMETERS.moveBy;
                Rotate.turnToAngle(PARAMETERS.RotatePos);
            }
            if(currentX && !lastX){
                PARAMETERS.RotatePos -= PARAMETERS.moveBy;
                Rotate.turnToAngle(PARAMETERS.RotatePos);
            }
            if(currentBack && !lastBack){
                Arm.turnToAngle(PARAMETERS.SET_ARM_POSITION);
                Rotate.turnToAngle(PARAMETERS.SET_ROTATE_POSITION);
            }
            if(currentLB && !lastLB){
                PARAMETERS.ClawPos += PARAMETERS.moveBy;
                Claw.turnToAngle(PARAMETERS.ClawPos);
            }
            if(currentRB && !lastRB){
                PARAMETERS.ClawPos -= PARAMETERS.moveBy;
                Claw.turnToAngle(PARAMETERS.ClawPos);
            }
            {
                lastDpadUp = currentDpadUp;
                lastDpadDown = currentDpadDown;
                lastX = currentX;
                lastTriangle = currentTriangle;
                lastBack = currentBack;
                lastLB = currentLB;
                lastRB = currentRB;
            }
            telemetry.addData("ArmPos: ",PARAMETERS.ArmPos);
            telemetry.addData("RotatePos: ",PARAMETERS.RotatePos);
            telemetry.addData("Move By: ",PARAMETERS.moveBy);
            telemetry.addData("Arm Angle: ",Arm.getAngle());
            telemetry.addData("Arm Position: ",Arm.getPosition());
            telemetry.addData("Rotate Angle: ",Rotate.getAngle());
            telemetry.addData("Rotate Position: ",Rotate.getPosition());
            telemetry.update();
        }
    }
    public void Rotate_To_IntakePosition(){
            Rotate.turnToAngle(PARAMETERS.Rotate_Intake_Position);
    }
    public void Rotate_To_DeployPosition(){
            Rotate.turnToAngle(PARAMETERS.Rotate_Deploy_Position);
    }
    public void Rotate_To_HangSafePosition(){
            Rotate.turnToAngle(PARAMETERS.Rotate_HangSafe_Position);
    }
    public void Arm_To_IntakePosition()
    {
            Arm.turnToAngle(PARAMETERS.Arm_Intake_Position);
    }
    public void Arm_To_DeployPosition()
    {
            Arm.turnToAngle(PARAMETERS.Arm_Deploy_Position);
    }
    public void Arm_To_HangSafePosition()
    {
            Arm.turnToAngle(PARAMETERS.Arm_HangSafe_Position);
    }
}

