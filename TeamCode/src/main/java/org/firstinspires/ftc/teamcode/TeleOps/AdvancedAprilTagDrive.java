package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Disabled
// To do: Make the driveRobot call at the end of the while OpMode is active
@TeleOp(name="Advanced Drive to AprilTag TRY NR 1", group = "Concept")
public class AdvancedAprilTagDrive extends LinearOpMode {
    final double DESIRED_DISTANCE = 12.0;
    final double SPEED_GAIN = 0.03;
    final double STRAFE_GAIN = 0.02;
    final double TURN_GAIN = 0.02;
    final double MAX_AUTO_SPEED = 0.8;
    final double MAX_AUTO_STRAFE = 0.8;
    final double MAX_AUTO_TURN = 0.5;

    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;
    private boolean cameraStreamActive = false;

    @Override
    public void runOpMode() {
        boolean targetFound = false;
        double drive = 0;
        double strafe = 0;
        double turn = 0;

        initAprilTag();
        leftFrontDrive = hardwareMap.get(DcMotor.class, "FL");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FR");
        leftBackDrive = hardwareMap.get(DcMotor.class, "BL");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BR");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        if (USE_WEBCAM) setManualExposure(6, 250);

        telemetry.addData("Camera preview on/off", "Press Options button to start");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.options && !cameraStreamActive) {
                FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
                cameraStreamActive = true;
                sleep(100);  // debounce delay
            }

            if (cameraStreamActive) {
                long startTime = System.currentTimeMillis();
                while (System.currentTimeMillis() - startTime < 5000) {
                    targetFound = false;
                    desiredTag = null;

                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                                targetFound = true;
                                desiredTag = detection;
                                break;
                            } else {
                                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                            }
                        } else {
                            telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                        }
                    }

                    if (targetFound) {
                        telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                        telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                        telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                        telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);

                        // Automatic alignment
                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double headingError = desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);


                        // Alert driver with custom rumble sequence
                        gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                                .addStep(1.0, 1.0, 500)
                                .addStep(0.0, 0.0, 250)
                                .addStep(1.0, 1.0, 500)
                                .build());
                        break;
                    }
                }

                if (!targetFound) {
                    telemetry.addData("Failure", "Could not find AprilTag");

                    // Rumble for 2 seconds at max power
                    gamepad1.runRumbleEffect(new Gamepad.RumbleEffect.Builder()
                            .addStep(1.0, 1.0, 2000)
                            .build());
                }
            } else {
                drive = -gamepad1.left_stick_y / 2.0;
                strafe = -gamepad1.left_stick_x / 2.0;
                turn = -gamepad1.right_stick_x / 3.0;
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                moveRobot(drive, strafe, turn);
            }

            telemetry.update();
            moveRobot(drive, strafe, turn);

        }
    }

    public void moveRobot(double x, double y, double yaw) {
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) {
            return;
        }

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        if (!isStopRequested()) {
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
    }
}
