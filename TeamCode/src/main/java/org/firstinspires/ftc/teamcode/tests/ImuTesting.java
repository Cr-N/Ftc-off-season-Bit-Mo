package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@Disabled
@TeleOp(name="IMU Telemetry", group="Sensor")
public class ImuTesting extends LinearOpMode {

    private IMU imu;

    @Override
    public void runOpMode() {
        // Define the orientation of the hub on the robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get IMU angles
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            // Telemetry data
            telemetry.addData("Yaw (Heading)", angles.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", angles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", angles.getRoll(AngleUnit.DEGREES));
            telemetry.update();

            // Optional: small delay to prevent spamming telemetry too fast
            sleep(100);
            if (gamepad1.cross)
                imu.resetYaw();
        }
    }
}
