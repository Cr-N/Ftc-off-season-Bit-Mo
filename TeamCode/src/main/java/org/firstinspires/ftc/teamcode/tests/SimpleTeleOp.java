package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardWare.Hardware;

@TeleOp(name = "Simple TeleOp")

public class SimpleTeleOp extends LinearOpMode {

    Hardware hardware = new Hardware(this);
    ElapsedTime runtime = new ElapsedTime();    // Use to determine when end game is starting.
    final double ENDGAME_TIME = 90.0;              // Wait this many seconds before rumble-alert for ENDGAME.
    boolean inEndgame = false;                 // Use to prevent multiple ENDGAME warning rumbles.

    Gamepad.RumbleEffect customRumbleEffect;    // Use to build a custom rumble sequence.

    @Override
    public void runOpMode() {
        GamepadEx gm1 = new GamepadEx(gamepad1);
        hardware.init();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .build();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double leftPower;
            double rightPower;
            double right_trigger = gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
            double left_trigger = gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);


            double drive = -gm1.getLeftY(); // left stick y
            double turn = gm1.getRightX(); // right stick x
            leftPower = Range.clip(drive + turn, -1.0, 1.0);
            rightPower = Range.clip(drive - turn, -1.0, 1.0);
            hardware.BL.setPower(leftPower);
            hardware.BR.setPower(rightPower);
            hardware.FL.setPower(leftPower);
            hardware.FR.setPower(rightPower);


            if (gamepad1.right_trigger > 0) {
                hardware.BL.setPower(-right_trigger);
                hardware.BR.setPower(right_trigger);
                hardware.FL.setPower(right_trigger);
                hardware.FR.setPower(-right_trigger);
            }
            if (gamepad1.left_trigger > 0) {
                hardware.BL.setPower(left_trigger);
                hardware.BR.setPower(-left_trigger);
                hardware.FL.setPower(-left_trigger);
                hardware.FR.setPower(left_trigger);
            }


            telemetry.addData("FL: ", hardware.FL.getCurrentPosition());
            telemetry.addData("FR: ", hardware.FR.getCurrentPosition());
            telemetry.addData("BL: ", hardware.BL.getCurrentPosition());
            telemetry.addData("BR: ", hardware.BR.getCurrentPosition());

            telemetry.addData("FL", hardware.FL.getPower());
            telemetry.addData("FR", hardware.FR.getPower());
            telemetry.addData("BL", hardware.BL.getPower());
            telemetry.addData("BR", hardware.BR.getPower());

            telemetry.update();

            if ((runtime.seconds() > ENDGAME_TIME) && !inEndgame)  {
                gamepad1.runRumbleEffect(customRumbleEffect);
                inEndgame =true;
            }
        }
    }
}