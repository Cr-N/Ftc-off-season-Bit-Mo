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
public class Arm_Rotate_Testing extends LinearOpMode {

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
    public boolean lastDpadDown = false,currentDpadDown,lastDpadUp=false,currentDpadUp,lastY = false,currentY,lastA=false,currentA;
    @Override
    public void runOpMode() throws InterruptedException {
        Arm = new SimpleServo(hardwareMap, "Arm", 0, 180, AngleUnit.DEGREES);
        Rotate = new SimpleServo(hardwareMap, "Rotate", 0, 180, AngleUnit.DEGREES);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx gm1 = new GamepadEx(gamepad1);

        waitForStart();
        while(opModeIsActive()){
            {
                currentDpadDown = gm1.isDown(GamepadKeys.Button.DPAD_DOWN);
                currentY = gm1.isDown(GamepadKeys.Button.Y);
                currentA = gm1.isDown(GamepadKeys.Button.A);
                currentDpadUp = gm1.isDown(GamepadKeys.Button.DPAD_UP);
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
            telemetry.addData("Arm ANGLE: " , PARAMETERS.currentArmPosition);
            telemetry.addData("Rotate ANGLE: ",PARAMETERS.currentRotatePosition);
            telemetry.addData("Arm POSITION: " , Arm.getPosition());
            telemetry.addData("Rotate POSITION: ",Rotate.getPosition());
            telemetry.update();
            {
                lastA = currentA;
                lastY = currentY;
                lastDpadDown = currentDpadDown;
                lastDpadUp = currentDpadUp;
            }
        }

    }
}
