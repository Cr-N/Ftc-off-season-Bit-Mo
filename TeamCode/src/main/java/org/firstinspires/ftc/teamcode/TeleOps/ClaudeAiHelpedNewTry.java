package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.HardWare.Hardware;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Revised Claude Helped ATag Drive", group = "Concept")
public class ClaudeAiHelpedNewTry extends LinearOpMode {
    // Constants
    private static final double DESIRED_DISTANCE = 12.0;
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;
    private static final double DISTANCE_TOLERANCE = 1.0;
    private static final double HEADING_TOLERANCE = 5.0;
    private static final double YAW_TOLERANCE = 5.0;
    private static final long AUTO_ALIGN_TIMEOUT = 10000; // 10 seconds

    // Hardware
    private Hardware robot;

    // Vision
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // State
    private boolean lastOptionsState = false;
    private boolean isAutoAlignMode = false;
    private boolean isCameraStreamActive = false;
    private ElapsedTime autoAlignTimer = new ElapsedTime();

    // PID variables
    private double integralSum = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() {
        try {
            initializeHardware();
            if (USE_WEBCAM) setManualExposure(6, 250);

            waitForStart();

            while (opModeIsActive()) {
                boolean currentOptionsState = gamepad1.options;

                if (currentOptionsState && !lastOptionsState) {
                    handleAprilTagDetection();
                }

                if (isAutoAlignMode) {
                    if (autoAlignTimer.milliseconds() > AUTO_ALIGN_TIMEOUT) {
                        exitAutoAlignMode("Auto-align timeout reached.");
                    } else {
                        driveToAprilTag();
                    }
                } else {
                    handleManualDrive();
                }

                if (gamepad1.b) {
                    exitAutoAlignMode("Manual exit triggered.");
                }

                lastOptionsState = currentOptionsState;
                updateTelemetry();
                sleep(10);
            }
        } catch (Exception e) {
            telemetry.addData("Error", "An exception occurred: " + e.getMessage());
            telemetry.update();
        } finally {
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }

    private void initializeHardware() {
        robot = new Hardware(hardwareMap);
        initAprilTag();
    }

    private void handleAprilTagDetection() {
        telemetry.addData("Status", "Searching for AprilTag...");
        telemetry.update();

        boolean targetFound = false;
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < 5000 && !targetFound && opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                }
            }
            sleep(20);
        }

        if (targetFound) {
            isAutoAlignMode = true;
            autoAlignTimer.reset();
            gamepad1.runRumbleEffect(createCustomRumbleEffect());
            startCameraStream();
            telemetry.addData("Status", "AprilTag detected! Entering auto-align mode.");
        } else {
            gamepad1.rumble(1.0, 1.0, 2000);
            telemetry.addData("Status", "No AprilTag detected. Remaining in manual mode.");
        }
        telemetry.update();
    }

    private void startCameraStream() {
        if (!isCameraStreamActive) {
            FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
            isCameraStreamActive = true;
        }
    }

    private Gamepad.RumbleEffect createCustomRumbleEffect() {
        return new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 1.0, 500)
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 1.0, 500)
                .build();
    }

    private void driveToAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        desiredTag = null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
                desiredTag = detection;
                break;
            }
        }

        if (desiredTag == null) {
            exitAutoAlignMode("AprilTag lost. Exiting auto-align mode.");
            return;
        }

        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        double drive = pidControl(rangeError, SPEED_GAIN, 0.001, 0.0001, MAX_AUTO_SPEED);
        double turn = pidControl(headingError, TURN_GAIN, 0.001, 0.0001, MAX_AUTO_TURN);
        double strafe = pidControl(-yawError, STRAFE_GAIN, 0.001, 0.0001, MAX_AUTO_STRAFE);

        moveRobot(drive, strafe, turn);

        if (Math.abs(rangeError) < DISTANCE_TOLERANCE &&
                Math.abs(headingError) < HEADING_TOLERANCE &&
                Math.abs(yawError) < YAW_TOLERANCE) {
            exitAutoAlignMode("Auto-align complete.");
        }

        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
    }

    private double pidControl(double error, double pGain, double iGain, double dGain, double maxOutput) {
        integralSum += error;
        double derivative = error - lastError;
        lastError = error;

        double output = (error * pGain) + (integralSum * iGain) + (derivative * dGain);
        return Range.clip(output, -maxOutput, maxOutput);
    }

    private void handleManualDrive() {
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double flPower = power * cos / max + turn;
        double frPower = power * sin / max - turn;
        double blPower = power * sin / max + turn;
        double brPower = power * cos / max - turn;

        if ((power + Math.abs(turn)) > 1) {
            flPower /= power + Math.abs(turn);
            frPower /= power + Math.abs(turn);
            blPower /= power + Math.abs(turn);
            brPower /= power + Math.abs(turn);
        }

        robot.FL.set(flPower);
        robot.FR.set(frPower);
        robot.BL.set(blPower);
        robot.BR.set(brPower);

        telemetry.addData("Manual", "FL %5.2f, FR %5.2f, BL %5.2f, BR %5.2f", flPower, frPower, blPower, brPower);
    }

    private void moveRobot(double x, double y, double yaw) {
        double flPower = x - y + yaw;
        double frPower = x + y - yaw;
        double blPower = x + y + yaw;
        double brPower = x - y - yaw;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
        ));

        robot.FL.set(flPower / max);
        robot.FR.set(frPower / max);
        robot.BL.set(blPower / max);
        robot.BR.set(brPower / max);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        visionPortal = builder.addProcessor(aprilTag).build();
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        sleep(20);

        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
        sleep(20);
    }

    private void updateTelemetry() {
        telemetry.addData("Mode", isAutoAlignMode ? "Auto-Align" : "Manual");
        telemetry.addData("Camera Stream", isCameraStreamActive ? "Active" : "Inactive");
        if (isAutoAlignMode && desiredTag != null) {
            telemetry.addData("Tag ID", desiredTag.id);
            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        }
        telemetry.update();
    }

    private void exitAutoAlignMode(String reason) {
        isAutoAlignMode = false;
        telemetry.addData("Status", "Exiting auto-align mode: " + reason);
        telemetry.update();
        sleep(1000); // Give time for the message to be read
    }
}