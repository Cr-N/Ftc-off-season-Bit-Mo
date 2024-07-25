/*package org.firstinspires.ftc.teamcode.TeleOps;

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
        waitForStart();

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
        robot = new Hardware(this);
        robot.init();
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

        robot.FL.setPower(flPower);
        robot.FR.setPower(frPower);
        robot.BL.setPower(blPower);
        robot.BR.setPower(brPower);

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

        robot.FL.setPower(flPower / max);
        robot.FR.setPower(frPower / max);
        robot.BL.setPower(blPower / max);
        robot.BR.setPower(brPower / max);
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
}*/
/*
////////////////THIS CODE WORKS!!!!!! (KINDA - it needs further adjusting for the allignment but the logic seems good)
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
        waitForStart();
        try {
            initializeHardware();
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
                //visionPortal.close();
            }
        }
    }

    private void initializeHardware() {
        robot = new Hardware(this);
        robot.init();
        initAprilTag();

        // Start the camera stream
        visionPortal.resumeStreaming();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        isCameraStreamActive = true;

        // Set manual exposure and gain
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }
    }

    private void handleAprilTagDetection() {
        telemetry.addData("Status", "Searching for AprilTag...");
        telemetry.update();

        boolean targetFound = false;
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < 5000 && !targetFound && opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (currentDetections.isEmpty()) {
                telemetry.addData("Detections", "No detections found.");
                telemetry.update();
                continue;
            }
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
            telemetry.addData("Status", "AprilTag detected! Entering auto-align mode.");
        } else {
            gamepad1.rumble(1.0, 1.0, 2000);
            telemetry.addData("Status", "No AprilTag detected. Remaining in manual mode.");
        }
        telemetry.update();
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
        if (currentDetections.isEmpty()) {
            exitAutoAlignMode("No detections found. Exiting auto-align mode.");
            return;
        }

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

        robot.FL.setPower(flPower);
        robot.FR.setPower(frPower);
        robot.BL.setPower(blPower);
        robot.BR.setPower(brPower);

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

        robot.FL.setPower(flPower / max);
        robot.FR.setPower(frPower / max);
        robot.BL.setPower(blPower / max);
        robot.BR.setPower(brPower / max);
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
*/
/*

////// THIS DID NOT FIX THE TURNING PROBLEM LOL

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
        waitForStart();
        try {
            initializeHardware();
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
                //visionPortal.close();
            }
        }
    }

    private void initializeHardware() {
        robot = new Hardware(this);
        robot.init();
        initAprilTag();

        // Start the camera stream
        visionPortal.resumeStreaming();
        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        isCameraStreamActive = true;

        // Set manual exposure and gain
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }
    }

    private void handleAprilTagDetection() {
        telemetry.addData("Status", "Searching for AprilTag...");
        telemetry.update();

        boolean targetFound = false;
        long startTime = System.currentTimeMillis();

        while (System.currentTimeMillis() - startTime < 5000 && !targetFound && opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (currentDetections.isEmpty()) {
                telemetry.addData("Detections", "No detections found.");
                telemetry.update();
                continue;
            }
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
            telemetry.addData("Status", "AprilTag detected! Entering auto-align mode.");
        } else {
            gamepad1.rumble(1.0, 1.0, 2000);
            telemetry.addData("Status", "No AprilTag detected. Remaining in manual mode.");
        }
        telemetry.update();
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
        if (currentDetections.isEmpty()) {
            exitAutoAlignMode("No detections found. Exiting auto-align mode.");
            return;
        }

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

        // Simplified movement logic to approach AprilTag
        double drive = pidControl(rangeError, SPEED_GAIN, 0.001, 0.0001, MAX_AUTO_SPEED);
        double turn = pidControl(headingError, TURN_GAIN, 0.001, 0.0001, MAX_AUTO_TURN);
        double strafe = pidControl(-yawError, STRAFE_GAIN, 0.001, 0.0001, MAX_AUTO_STRAFE);

        // Adjusted movement based on detection feedback
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

        robot.FL.setPower(flPower);
        robot.FR.setPower(frPower);
        robot.BL.setPower(blPower);
        robot.BR.setPower(brPower);
    }

    private void moveRobot(double drive, double strafe, double turn) {
        double frontLeftPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
        double frontRightPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
        double backLeftPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
        double backRightPower = Range.clip(drive + strafe - turn, -1.0, 1.0);

        robot.FL.setPower(frontLeftPower);
        robot.FR.setPower(frontRightPower);
        robot.BL.setPower(backLeftPower);
        robot.BR.setPower(backRightPower);
    }

    private void exitAutoAlignMode(String message) {
        isAutoAlignMode = false;
        desiredTag = null;
        telemetry.addData("Status", message);
        telemetry.update();
    }

    private void updateTelemetry() {
        if (desiredTag != null) {
            telemetry.addData("Detected Tag ID", desiredTag.id);
            telemetry.addData("Range", desiredTag.ftcPose.range);
            telemetry.addData("Bearing", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("Detected Tag ID", "None");
        }
        telemetry.addData("Auto Align Mode", isAutoAlignMode);
        telemetry.addData("Exposure Time", visionPortal.getCameraControl(ExposureControl.class).getExposure(TimeUnit.MILLISECONDS));
        telemetry.addData("Gain", visionPortal.getCameraControl(GainControl.class).getGain());
        telemetry.update();
    }

    private void setManualExposure(int gain, int exposureMs) {
        if (USE_WEBCAM) {
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            if (gainControl != null) {
                gainControl.setGain(gain);
            }
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null) {
                exposureControl.setExposure(exposureMs, TimeUnit.MILLISECONDS);
            }
        }
    }

    private void initAprilTag() {
        AprilTagProcessor.Builder aprilTagBuilder = new AprilTagProcessor.Builder();
        aprilTagBuilder.setDrawAxes(true);
        aprilTagBuilder.setDrawCubeProjection(true);
        aprilTagBuilder.setDrawTagOutline(true);
        aprilTagBuilder.setDrawTagID(true);
        aprilTagBuilder.setOutputUnits(DistanceUnit.CM , AngleUnit.DEGREES);
        aprilTag = aprilTagBuilder.build();

        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.setCamera(USE_WEBCAM ? hardwareMap.get(WebcamName.class, "Webcam 1") : null);
        visionPortalBuilder.addProcessor(aprilTag);
        visionPortal = visionPortalBuilder.build();
    }
}
*/
package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.teamcode.HardWare.Hardware;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Disabled
@TeleOp(name="Mars Rover: AprilTag Navigation", group = "Mars")
public class ClaudeAiHelpedNewTry extends LinearOpMode {
    // Constants
    private static final double DESIRED_DISTANCE = 12.0; // cm
    private static final double SPEED_GAIN = 0.02;
    private static final double STRAFE_GAIN = 0.015;
    private static final double TURN_GAIN = 0.01;
    private static final double MAX_AUTO_SPEED = 0.5;
    private static final double MAX_AUTO_STRAFE = 0.5;
    private static final double MAX_AUTO_TURN = 0.3;
    private static final double DISTANCE_TOLERANCE = 2.0; // cm
    private static final double HEADING_TOLERANCE = 2.0; // degrees
    private static final double YAW_TOLERANCE = 2.0; // degrees
    private static final long DETECTION_TIMEOUT = 5000; // ms
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1; // -1 means any tag

    // Hardware
    private Hardware robot;

    // Vision
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    // State
    private boolean isAutoAlignMode = false;
    private ElapsedTime detectionTimer = new ElapsedTime();
    private ElapsedTime telemetryTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        waitForStart();
        try {
            initialize();
            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.options && !isAutoAlignMode) {
                    handleAprilTagDetection();
                }

                if (isAutoAlignMode) {
                    driveToAprilTag();
                } else {
                    handleManualDrive();
                }

                updateTelemetry();
                sleep(20); // Short sleep to prevent CPU overuse
            }
        } catch (Exception e) {
            telemetry.addData("Error", "An exception occurred: " + e.getMessage());
            telemetry.update();
        } finally {
            shutdown();
        }
    }

    private void initialize() {
        telemetry.addData("Status", "Initializing...");
        telemetry.update();

        // Initialize robot hardware
        robot = new Hardware(this);
        robot.init();

        // Initialize AprilTag detection
        initAprilTag();

        // Set manual exposure for consistent detection
        if (USE_WEBCAM) {
            setManualExposure(6, 250);
        }

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    private void handleAprilTagDetection() {
        telemetry.addData("Status", "Searching for AprilTag...");
        telemetry.update();

        detectionTimer.reset();
        while (detectionTimer.milliseconds() < DETECTION_TIMEOUT && !isStopRequested()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
                    desiredTag = detection;
                    isAutoAlignMode = true;
                    gamepad1.runRumbleEffect(createCustomRumbleEffect());
                    telemetry.addData("Status", "AprilTag detected. Entering auto-align mode.");
                    telemetry.update();
                    return;
                }
            }
            sleep(20);
        }

        // If no tag is found after timeout
        if (!isAutoAlignMode) {
            gamepad1.rumble(1.0, 1.0, 2000);
            telemetry.addData("Status", "No AprilTag detected. Remaining in manual mode.");
            telemetry.update();
        }
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
            isAutoAlignMode = false;
            stopRobot();
            telemetry.addData("Status", "AprilTag lost. Exiting auto-align mode.");
            telemetry.update();
            return;
        }

        double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;

        // Calculate motor powers using the method from atagtesting
        double drive = -rangeError * SPEED_GAIN;
        double turn = -headingError * TURN_GAIN;
        double strafe = -yawError * STRAFE_GAIN;

        // Limit the motor powers to their respective maximums
        drive = Range.clip(drive, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        turn = Range.clip(turn, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        strafe = Range.clip(strafe, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        // Calculate wheel powers using mecanum drive kinematics
        double[] powers = calculateWheelPowers(drive, strafe, turn);

        // Apply the calculated powers to the motors
        robot.FL.setPower(powers[0]);
        robot.FR.setPower(powers[1]);
        robot.BL.setPower(powers[2]);
        robot.BR.setPower(powers[3]);

        // Check if we've reached the desired position
        if (Math.abs(rangeError) < DISTANCE_TOLERANCE &&
                Math.abs(headingError) < HEADING_TOLERANCE &&
                Math.abs(yawError) < YAW_TOLERANCE) {
            isAutoAlignMode = false;
            stopRobot();
            telemetry.addData("Status", "Desired position reached. Exiting auto-align mode.");
            telemetry.update();
        }
    }

    private double[] calculateWheelPowers(double drive, double strafe, double turn) {
        double[] wheelPowers = new double[4];
        wheelPowers[0] = drive + strafe + turn; // Front Left
        wheelPowers[1] = drive - strafe - turn; // Front Right
        wheelPowers[2] = drive - strafe + turn; // Back Left
        wheelPowers[3] = drive + strafe - turn; // Back Right

        // Normalize wheel powers
        double max = Math.max(Math.abs(wheelPowers[0]), Math.max(Math.abs(wheelPowers[1]),
                Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3]))));
        if (max > 1.0) {
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] /= max;
            }
        }

        return wheelPowers;
    }

    private void handleManualDrive() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double[] powers = calculateWheelPowers(drive, strafe, turn);

        robot.FL.setPower(powers[0]);
        robot.FR.setPower(powers[1]);
        robot.BL.setPower(powers[2]);
        robot.BR.setPower(powers[3]);
    }

    private void stopRobot() {
        robot.FL.setPower(0);
        robot.FR.setPower(0);
        robot.BL.setPower(0);
        robot.BR.setPower(0);
    }

    private void updateTelemetry() {
        // Limit telemetry updates to reduce CPU usage
        if (telemetryTimer.milliseconds() > 250) {
            telemetry.addData("Mode", isAutoAlignMode ? "Auto Align" : "Manual");
            if (desiredTag != null) {
                telemetry.addData("Detected Tag ID", desiredTag.id);
                telemetry.addData("Tag Range", String.format("%.2f cm", desiredTag.ftcPose.range));
                telemetry.addData("Tag Bearing", String.format("%.2f degrees", desiredTag.ftcPose.bearing));
                telemetry.addData("Tag Yaw", String.format("%.2f degrees", desiredTag.ftcPose.yaw));
            } else {
                telemetry.addData("Detected Tag ID", "None");
            }
            telemetry.update();
            telemetryTimer.reset();
        }
    }

    private void setManualExposure(int gain, int exposureMs) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMs, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        }
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private Gamepad.RumbleEffect createCustomRumbleEffect() {
        return new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  // Rumble both motors 100% for 500ms
                .addStep(0.0, 0.0, 300)  // Pause for 300ms
                .addStep(1.0, 1.0, 500)  // Rumble both motors 100% for 500ms
                .addStep(0.0, 0.0, 300)  // Pause for 300ms
                .addStep(1.0, 1.0, 500)  // Rumble both motors 100% for 500ms
                .build();
    }

    private void shutdown() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}