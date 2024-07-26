package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Config
@TeleOp
public class Arm_Rotate_Claw_Testing extends LinearOpMode {
    public static class Params{
        public double Arm_Intake_Position =0;
        public double Arm_Deploy_Position = 100;
        public double Arm_HangSafe_Position = 50;
        public double Rotate_Intake_Position = 0;
        public double Rotate_Deploy_Position =100;
        public double Rotate_HangSafe_Position=50;
        public double currentArmPosition =0;
        public double currentRotatePosition=0;
        public double moveArmBy =10;
        public double moveRotateBy =10;
        public double moveClawBy = 10;
        public double currentClawPosition=0;

        /*public enum ArmStates{
            AT_INTAKE_POSITION,
            AT_HangSafe_POSITION,
            AT_DEPLOY_POSITION
        };
        public enum RotationState{
            AT_INTAKE_POSITION,
            AT_DEPLOY_POSITION,
            AT_HangSafe_Position
        }*/
    }
    public static Params PARAMETERS = new Params();
    public ServoEx Arm;
    public ServoEx Rotate;
    public ServoEx claw;
    public boolean lastDpadDown = false,currentDpadDown,lastDpadUp=false,currentDpadUp,lastY = false,currentY,lastA=false,currentA,lastLB=false,currentLB,lastRB=false,currentRB;
    @Override
    public void runOpMode() throws InterruptedException {
        Arm = new SimpleServo(hardwareMap, "Arm", 0, 180, AngleUnit.DEGREES);
        Rotate = new SimpleServo(hardwareMap, "Rotate", 0, 180, AngleUnit.DEGREES);
        claw = new SimpleServo(hardwareMap, "claw", 0, 270, AngleUnit.DEGREES);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx gm1 = new GamepadEx(gamepad1);

        waitForStart();
        while(opModeIsActive()){
            {
                currentDpadDown = gm1.isDown(GamepadKeys.Button.DPAD_DOWN);
                currentY = gm1.isDown(GamepadKeys.Button.Y);
                currentA = gm1.isDown(GamepadKeys.Button.A);
                currentDpadUp = gm1.isDown(GamepadKeys.Button.DPAD_UP);
                currentLB=gm1.isDown(GamepadKeys.Button.LEFT_BUMPER);
                currentRB=gm1.isDown(GamepadKeys.Button.RIGHT_BUMPER);
            }
            if(currentDpadUp && !lastDpadUp)
            {
                PARAMETERS.currentArmPosition += PARAMETERS.moveArmBy;
                Arm.turnToAngle(PARAMETERS.currentArmPosition);
            }
            if(currentDpadDown && !lastDpadDown)
            {
                PARAMETERS.currentArmPosition -= PARAMETERS.moveArmBy;
                Arm.turnToAngle(PARAMETERS.currentArmPosition);
            }
            if(currentY && !lastY)
            {
                PARAMETERS.currentRotatePosition += PARAMETERS.moveRotateBy;
                Rotate.turnToAngle(PARAMETERS.currentRotatePosition);
            }
            if(currentA && !lastA)
            {
                PARAMETERS.currentRotatePosition -= PARAMETERS.moveRotateBy;
                Rotate.turnToAngle(PARAMETERS.currentRotatePosition);
            }
            if(currentLB && !lastLB)
            {
                PARAMETERS.currentClawPosition -= PARAMETERS.moveClawBy;
                claw.turnToAngle(PARAMETERS.currentClawPosition);
            }
            if(currentRB && !lastRB)
            {
                PARAMETERS.currentClawPosition += PARAMETERS.moveClawBy;
                claw.turnToAngle(PARAMETERS.currentClawPosition);
            }
            telemetry.addData("Arm ANGLE: " , PARAMETERS.currentArmPosition);
            telemetry.addData("Arm POSITION: " , Arm.getPosition());
            telemetry.addLine(" ");
            telemetry.addData("Rotate ANGLE: ",PARAMETERS.currentRotatePosition);
            telemetry.addData("Rotate POSITION: ",Rotate.getPosition());
            telemetry.addLine(" ");
            telemetry.addData("Claw Angle: ", claw.getAngle());
            telemetry.addData("Claw position: ", claw.getPosition());
            telemetry.update();
            {
                lastA = currentA;
                lastY = currentY;
                lastDpadDown = currentDpadDown;
                lastDpadUp = currentDpadUp;
                lastLB = currentLB;
                lastRB = currentRB;
            }
        }

    }
}
