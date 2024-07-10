package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
@TeleOp
public class ColorSensorSubsystemTest extends LinearOpMode {
    ColorSensor colorSensor;
    GamepadEx gm1;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = new ColorSensor(this);
        gm1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()){
            colorSensor.handle_pixel_feedback(); // -KJV John(3:16)"For God so loved the world that He gave His only Begotten Son, that whosoever beliveth in him shall not perish but have everlasting life"

            telemetry.addData("Distance STATE:  ", colorSensor.get_distance_state());
            telemetry.addData("Distance(CM):  ", colorSensor.get_current_distance_param());
            telemetry.addData("Min Detection Distance :  ", colorSensor.get_minimum_detection_distance_param());
            telemetry.update();
        }
    }
}
