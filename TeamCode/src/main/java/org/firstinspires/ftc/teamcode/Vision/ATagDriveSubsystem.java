/*
package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardWare.DriveBase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class ATagDriveSubsystem {
    HardwareMap hardwareMap;
    LinearOpMode myOpmode = null;

    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 6.0; //  this is how close the camera should get to the target (inches)
    DriveBase driveBase;

    final double SPEED_GAIN = 0.08;          //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.08;         //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;           //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.6;       //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.6;      //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;        //  Clip the turn speed to this max value (adjust for your robot)
    private static int DESIRED_TAG_ID = -1;  // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;       // Used to manage the video source.
    private AprilTagProcessor aprilTag;      // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;   // Used to hold the data for a detected AprilTag
    boolean targetFound = false;    // Set to true when an AprilTag target is detected
    double drive = 0;        // Desired forward power/speed (-1 to +1)
    double strafe = 0;        // Desired strafe power/speed (-1 to +1)
    double turn = 0;        // Desired turning power/speed (-1 to +1)

    public ATagDriveSubsystem(LinearOpMode opMode) {
        myOpmode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        driveBase = new DriveBase(myOpmode);
    }

    public void init_AprilTag_Drive(int Tag_To_Drive_To) {
        DESIRED_TAG_ID = Tag_To_Drive_To;
        initAprilTag();
    }

    public void get_Detections() {

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
                    // This tag is in the library, but we do not want to track it right now.
                    myOpmode.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                myOpmode.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            myOpmode.telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
            myOpmode.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            myOpmode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            myOpmode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            myOpmode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            myOpmode.telemetry.addData("\n>", "Drive using joysticks to find valid target\n");
        }
    }

    public void Apply_Powers() {

        if (targetFound) {
            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
            double headingError = desiredTag.ftcPose.bearing;
            double yawError = desiredTag.ftcPose.yaw;

            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            myOpmode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        }
        else{
            myOpmode.telemetry.addLine("Tag not found, not applying powers.");
        }
        myOpmode.telemetry.update();

        // Apply desired axes motions to the drivetrain.
        moveRobot(drive, strafe, turn);
    }


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
/*
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        driveBase.FL.set(leftFrontPower);
        driveBase.FR.set(rightFrontPower);
        driveBase.BL.set(leftBackPower);
        driveBase.BR.set(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
   /* private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                .build();

        aprilTag.setDecimation(3);  // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
                                    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
    }

}
*/