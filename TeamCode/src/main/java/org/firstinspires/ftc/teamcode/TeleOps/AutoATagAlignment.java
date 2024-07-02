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
import org.firstinspires.ftc.teamcode.HardWare.Hardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
// Total hour count: too many to count
@TeleOp(name="AprilTag Alignment TRY NR 3 or smth", group = "AprilTag")
public class AutoATagAlignment extends LinearOpMode {
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

    boolean lastOptions = false;
    boolean currentOptions;

    @Override
    public void runOpMode() {
        waitForStart();
        try {
            initialize();
            waitForStart();

            while (opModeIsActive()) {
                currentOptions = gamepad1.options;
                if (gamepad1.options && !lastOptions && !isAutoAlignMode) {
                    handleAprilTagDetection();
                }

                if (isAutoAlignMode) {
                    driveToAprilTag();
                } else {
                    handleManualDrive();
                }

                updateTelemetry();
                lastOptions = currentOptions;
                sleep(20); // Short sleep to prevent CPU overuse
            }
        } catch (Exception e) {
            telemetry.addData("Error", "An exception occurred: " + e.getMessage());
            telemetry.update();
        } finally {
            shutdownVisionProcessor();
        }
    }

    private void initialize() {

        telemetry.addData("Status", "Initializing...");
        telemetry.update();

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
        // Start processing image to detect AprilTag
        startVisionProcessor();

        telemetry.addData("Status", "Searching for AprilTag...");
        telemetry.update();
        // Resetting timer for Detection
        detectionTimer.reset();

        while (detectionTimer.milliseconds() < DETECTION_TIMEOUT && !isStopRequested()) {

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {

                if (detection.metadata != null && (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {

                    desiredTag = detection;
                    // Change state
                    isAutoAlignMode = true;
                    // Start Dashboard stream
                    FtcDashboard.getInstance().startCameraStream(visionPortal,30);

                    // Signal driver we got a detection
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
        // not checking for null bc we already did so
        double rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
        double headingError = desiredTag.ftcPose.bearing;
        double yawError = desiredTag.ftcPose.yaw;
        ///////////////////////////////////////////////////////// TODO: see if we actually need to check for the heading and yaw tolerance in the while loop below
        while ((Math.abs(rangeError) > DISTANCE_TOLERANCE) && (Math.abs(headingError) > HEADING_TOLERANCE) && (Math.abs(yawError) > YAW_TOLERANCE)  && !isStopRequested()){

            List<AprilTagDetection> loopingDetection = aprilTag.getDetections();
            desiredTag = null;
            for (AprilTagDetection detection : loopingDetection) {
                if (detection.metadata != null && (DESIRED_TAG_ID < 0 || detection.id == DESIRED_TAG_ID)) {
                    desiredTag = detection;
                    break;
                }
            }
            if(desiredTag != null){

                rangeError = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                headingError = desiredTag.ftcPose.bearing;
                yawError = desiredTag.ftcPose.yaw;

            }

            // Calculate motor powers using the method from atagtesting
            double drive,turn,strafe;

            // Limit the motor powers to their respective maximums
            drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            // Calculate wheel powers using mecanum drive kinematics
            double[] powers = calculateWheelPowers(drive, strafe, turn);

            // Apply the calculated powers to the motors
            robot.FL.setPower(powers[0]);
            robot.FR.setPower(powers[1]);
            robot.BL.setPower(powers[2]);
            robot.BR.setPower(powers[3]);

            // Check if we've reached the desired position
            if (Math.abs(rangeError) < DISTANCE_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE && Math.abs(yawError) < YAW_TOLERANCE) {
                isAutoAlignMode = false;
                stopRobot();
                telemetry.addData("Status", "Desired position reached. Exiting auto-align mode.");
                telemetry.update();
            }
        }
        // Check if we've reached the desired position
        if (Math.abs(rangeError) < DISTANCE_TOLERANCE && Math.abs(headingError) < HEADING_TOLERANCE && Math.abs(yawError) < YAW_TOLERANCE) {
            isAutoAlignMode = false;
            stopRobot();
            telemetry.addData("Status", "Desired position reached. Exiting auto-align mode.");
            telemetry.update();
        }
    }

    private double[] calculateWheelPowers(double drive, double strafe, double turn) {
        double[] wheelPowers = new double[4];

        wheelPowers[0] = drive - strafe - turn;  // Front Left (FL)

        wheelPowers[1] =  drive + strafe + turn; // Front Right (FR)

        wheelPowers[2] = drive + strafe - turn;  // Back Left (BL)

        wheelPowers[3] =  drive - strafe + turn; // Back Right (BR)
        double max = Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1]));
        max = Math.max(max, Math.abs(wheelPowers[2]));
        max = Math.max(max, Math.abs(wheelPowers[3]));

        if(max > 1.0){
            wheelPowers[0] /= max;
            wheelPowers[1] /= max;
            wheelPowers[2] /= max;
            wheelPowers[3] /= max;
        }

        return wheelPowers;
    }

    private void handleManualDrive() {

        shutdownVisionProcessor();

        double x,y,turn,theta,power,sin,cos,max,FLpow,FRpow,BLpow,BRpow;

        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;

        theta = Math.atan2(y,x);
        power = Math.hypot(x,y);

        sin = Math.sin(theta - Math.PI/4);
        cos = Math.cos(theta - Math.PI/4);
        max = Math.max( Math.abs(sin) , Math.abs(cos) );

        FLpow = power * cos/max + turn;
        FRpow = power * sin/max - turn;
        BLpow = power * sin/max + turn;
        BRpow = power * cos/max - turn;

        if((power + Math.abs(turn)) > 1){
            FLpow /= power + Math.abs(turn);
            FRpow /= power + Math.abs(turn);
            BLpow /= power + Math.abs(turn);
            BRpow /= power + Math.abs(turn);
        }

        robot.FL.setPower(FLpow);
        robot.FR.setPower(FRpow);
        robot.BL.setPower(BLpow);
        robot.BR.setPower(BRpow);
    }

    private void stopRobot() {
        if(!isStopRequested()){
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
            robot.BR.setPower(0);
        }
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

    private void shutdownVisionProcessor() {
        if (visionPortal != null) {
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }
    private void startVisionProcessor(){
        if(visionPortal != null){
            visionPortal.setProcessorEnabled(aprilTag,true);
        }
    }
}