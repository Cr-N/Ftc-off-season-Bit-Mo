package org.firstinspires.ftc.teamcode.TeleOps;


import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardWare.HardwareMaps;

import org.firstinspires.ftc.teamcode.HardWare.HardwareMaps;

import kotlin.jvm.internal.Intrinsics;

@TeleOp(name="Field Centric testing ❌", group = "OpModes")
public class FieldCentric extends LinearOpMode {
    HardwareMap hwmap;
    GamepadEx gm1 = new GamepadEx(gamepad1);
    HardwareMaps hm = new HardwareMaps(hwmap);

    @Override
    public void runOpMode()
    {
        hm.imu.init();
        hm.imu.reset();

        double[] imuAngles = hm.imu.getAngles();
        
        waitForStart();

        while (opModeIsActive())
        {
            double botHeading = Math.toRadians(imuAngles[0]); // TODO: change according to what we use(order is yaw,pich,roll)
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                hm.imu.reset();
            }

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            hm.frontLeftMotor.set(frontLeftPower);
            hm.backLeftMotor.set(backLeftPower);
            hm.frontRightMotor.set(frontRightPower);
            hm.backRightMotor.set(backRightPower);
        }
    }

}
