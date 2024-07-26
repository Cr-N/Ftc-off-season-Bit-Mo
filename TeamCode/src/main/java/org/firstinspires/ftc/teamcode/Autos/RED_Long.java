package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.MasterWithActionsClass;
import org.firstinspires.ftc.teamcode.Vision.HeightFilterRed3Box;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class RED_Long extends LinearOpMode{

    double LEFT_SPILNE_1_X = -51;
    double LEFT_SPLINE_1_Y = -37;
    double LEFT_SPLINE_1_HEADING = Math.PI/2;
    double LEFT_SPLINE_1_TANGENT = 2;
    double LEFT_WAIT_SECONDS_1 = 0.5;
    double LEFT_STRAFE_1_VECTOR_X = -46.5;
    double LEFT_STRAFE_1_VECTOR_Y = -43;
    double LEFT_STRAFE_1_HEADING = Math.PI/2+Math.PI/6;
    double LEFT_SPLINE_2_X = -35.4;
    double LEFT_SPLINE_2_Y = -62;
    double LEFT_SPLINE_2_HEADING = Math.PI;
    double LEFT_SPLINE_2_TANGENT = 0;
    double LEFT_STRAFE_2_VECTOR_X = 5;
    double LEFT_STRAFE_2_VECTOR_Y = -58.9;
    double LEFT_SPLINE_3_X = 48.4;
    double LEFT_SPLINE_3_Y = -21; // -22.4
    double LEFT_SPLINE_3_TANGENT = 0;
    double LEFT_WAIT_SECONDS_2 = 2.5;
    double LEFT_PARK_X = 50;
    double LEFT_PARK_Y = -60;

    double MIDDLE_SPILNE_1_X = -39;
    double MIDDLE_SPLINE_1_Y = -36.4;
    double MIDDLE_SPLINE_1_HEADING = Math.PI/2;
    double MIDDLE_SPLINE_1_TANGENT = 2;
    double MIDDLE_WAIT_SECONDS_1 = 0.5;
    double MIDDLE_STRAFE_1_VECTOR_X = -46.5;
    double MIDDLE_STRAFE_1_VECTOR_Y = -43;
    double MIDDLE_STRAFE_1_HEADING = Math.PI/2+Math.PI/6;
    double MIDDLE_SPLINE_2_X = -35.4;
    double MIDDLE_SPLINE_2_Y = -62;
    double MIDDLE_SPLINE_2_HEADING = Math.PI;
    double MIDDLE_SPLINE_2_TANGENT = 0;
    double MIDDLE_STRAFE_2_VECTOR_X = 5;
    double MIDDLE_STRAFE_2_VECTOR_Y = -60;
    double MIDDLE_SPLINE_3_X = 48.4;
    double MIDDLE_SPLINE_3_Y = -33;
    double MIDDLE_SPLINE_3_TANGENT = 0;
    double MIDDLE_WAIT_SECONDS_2 = 2;
    double MIDDLE_PARK_X = 50;
    double MIDDLE_PARK_Y = -60;

    double RIGHT_STRAFE_1_X=-40;
    double RIGHT_STRAFE_1_Y=-45;
    double RIGHT_SPILNE_1_X = -37;
    double RIGHT_SPLINE_1_Y = -36;
    double RIGHT_SPLINE_1_HEADING = Math.toRadians(20);
    double RIGHT_SPLINE_1_TANGENT = 0;
    double RIGHT_WAIT_SECONDS_1 = 0.5;
    double RIGHT_STRAFE_1_VECTOR_X = -46.5;
    double RIGHT_STRAFE_1_VECTOR_Y = -43;
    double RIGHT_STRAFE_1_HEADING = Math.PI/2+Math.PI/6;
    double RIGHT_SPLINE_2_X = -35.4;
    double RIGHT_SPLINE_2_Y = -62;
    double RIGHT_SPLINE_2_HEADING = Math.PI;
    double RIGHT_SPLINE_2_TANGENT = 0;
    double RIGHT_STRAFE_2_VECTOR_X = 5;
    double RIGHT_STRAFE_2_VECTOR_Y = -59.9;
    double RIGHT_SPLINE_3_X = 48.4;
    double RIGHT_SPLINE_3_Y = -39; // -22.4
    double RIGHT_SPLINE_3_TANGENT = 0;
    double RIGHT_WAIT_SECONDS_2 = 2;
    double RIGHT_PARK_X = 50;
    double RIGHT_PARK_Y = -60;


    MasterWithActionsClass master;
    HeightFilterRed3Box REDvisionProcessor;
    MecanumDrive drive;
    Action LEFT;
    Action MIDDLE;
    Action RIGHT;
    VisionPortal visionPortal;
    enum CASE {
        LEFT,
        MIDDLE,
        RIGHT,
        NONE
    };
    //HeightFilterRed3Box.Selected CASE;
    @Override
    public void runOpMode() throws InterruptedException {
        CASE detectionCase = CASE.NONE;
        master = new MasterWithActionsClass(this);
        REDvisionProcessor = new HeightFilterRed3Box();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), REDvisionProcessor);
        drive = new MecanumDrive(hardwareMap,new Pose2d(-35.2, -63.2, Math.PI/2));
        // left case trajectory
        {
            LEFT = drive.actionBuilder(drive.pose)
                    //Spike mark LEFT
                    .splineToLinearHeading(new Pose2d(LEFT_SPILNE_1_X,LEFT_SPLINE_1_Y,LEFT_SPLINE_1_HEADING),LEFT_SPLINE_1_TANGENT)
                    .afterTime(1.3, master.intake.DEPLOY_1())
                    .waitSeconds(LEFT_WAIT_SECONDS_2)

                    // Go to leaving spot
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(LEFT_SPLINE_2_X,LEFT_SPLINE_2_Y,LEFT_SPLINE_2_HEADING),LEFT_SPLINE_2_TANGENT)

                    // Go forward a little(we cross from long to short)
                    .strafeToConstantHeading(new Vector2d(LEFT_STRAFE_2_VECTOR_X,LEFT_STRAFE_2_VECTOR_Y))
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Spline to Backboard on the LEFT side
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(LEFT_SPLINE_3_X,LEFT_SPLINE_3_Y),LEFT_SPLINE_3_TANGENT)
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Deploy Yellow
                    .afterTime(0.8,master.Score_Yellow_MIDDLE_SHORT())
                    .waitSeconds(LEFT_WAIT_SECONDS_2)

                    //Park in the corner
                    //.strafeTo(new Vector2d(PARK_X,PARK_Y))
                    .splineToConstantHeading(new Vector2d(LEFT_PARK_X,LEFT_PARK_Y),LEFT_SPLINE_3_TANGENT)

                    .build();

        }
        // middle case trajectory
        {
            MIDDLE = drive.actionBuilder(drive.pose)
                    //Spike mark MIDDLE
                    .splineToLinearHeading(new Pose2d(MIDDLE_SPILNE_1_X,MIDDLE_SPLINE_1_Y,MIDDLE_SPLINE_1_HEADING),MIDDLE_SPLINE_1_TANGENT)
                    .afterTime(1,master.intake.DEPLOY_1())
                    .waitSeconds(MIDDLE_WAIT_SECONDS_2)

                    //Go back a little
                    //.strafeToSplineHeading(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y),STRAFE_1_HEADING)
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Go to leaving spot
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(MIDDLE_SPLINE_2_X,MIDDLE_SPLINE_2_Y,MIDDLE_SPLINE_2_HEADING),MIDDLE_SPLINE_2_TANGENT)
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Go forward a little(we cross from long to short)
                    .strafeToConstantHeading(new Vector2d(MIDDLE_STRAFE_2_VECTOR_X,MIDDLE_STRAFE_2_VECTOR_Y))
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Spline to Backboard on the LEFT side
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(MIDDLE_SPLINE_3_X,MIDDLE_SPLINE_3_Y),MIDDLE_SPLINE_3_TANGENT)
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Deploy Yellow
                    .afterTime(0.5,master.Score_Yellow())
                    .waitSeconds(MIDDLE_WAIT_SECONDS_2)

                    //Park in the corner
                    //.strafeTo(new Vector2d(PARK_X,PARK_Y))
                    .splineToConstantHeading(new Vector2d(MIDDLE_PARK_X,MIDDLE_PARK_Y),MIDDLE_SPLINE_3_TANGENT)

                    .build();
        }
        // right case trajectory
        {
            RIGHT = drive.actionBuilder(drive.pose)
                    //Spike mark LEFT
                    .strafeToConstantHeading(new Vector2d(RIGHT_STRAFE_1_X,RIGHT_STRAFE_1_Y))
                    .splineToLinearHeading(new Pose2d(RIGHT_SPILNE_1_X,RIGHT_SPLINE_1_Y,RIGHT_SPLINE_1_HEADING),RIGHT_SPLINE_1_TANGENT)
                    // .splineTo(new Vector2d(SPILNE_1_X,SPLINE_1_Y),SPLINE_1_TANGENT)
                    .afterTime(1,master.intake.DEPLOY_1())
                    .waitSeconds(RIGHT_WAIT_SECONDS_2)

                    //Go back a little
                    //.strafeToSplineHeading(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y),STRAFE_1_HEADING)
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Go to leaving spot
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(RIGHT_SPLINE_2_X,RIGHT_SPLINE_2_Y,RIGHT_SPLINE_2_HEADING),RIGHT_SPLINE_2_TANGENT)
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Go forward a little(we cross from long to short)
                    .strafeToConstantHeading(new Vector2d(RIGHT_STRAFE_2_VECTOR_X,RIGHT_STRAFE_2_VECTOR_Y))
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Spline to Backboard on the RIGHT side
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(RIGHT_SPLINE_3_X,RIGHT_SPLINE_3_Y),RIGHT_SPLINE_3_TANGENT)
                    //.waitSeconds(WAIT_SECONDS_2)

                    // Deploy Yellow
                    .afterTime(0.5,master.Score_Yellow())
                    .waitSeconds(RIGHT_WAIT_SECONDS_2)

                    //Park in the corner
                    //.strafeTo(new Vector2d(PARK_X,PARK_Y))
                    .splineToConstantHeading(new Vector2d(RIGHT_PARK_X,RIGHT_PARK_Y),RIGHT_SPLINE_3_TANGENT)


                    .build();
        }
        Actions.runBlocking(
                master.Prep_For_Purple()
        );
        while (!opModeIsActive() && !isStopRequested()){
            switch (REDvisionProcessor.getSelection()){
                case LEFT:
                    telemetry.addLine("LEFT");
                    telemetry.update();
                    detectionCase = CASE.LEFT;
                    break;
                case RIGHT:
                    telemetry.addLine("RIGHT");
                    telemetry.update();
                    detectionCase = CASE.RIGHT;
                    break;
                case MIDDLE:
                    telemetry.addLine("MIDDLE");
                    telemetry.update();
                    detectionCase = CASE.MIDDLE;
                    break;
                case NONE:
                    telemetry.addLine("NONE AKA RANDOM");
                    telemetry.update();
                    detectionCase = CASE.NONE;
                    break;
            }
        }
        waitForStart();

        if(isStopRequested())return;
        switch (detectionCase){
            case LEFT:
                Actions.runBlocking(
                        LEFT
                );
                break;
            case MIDDLE:
                Actions.runBlocking(
                        MIDDLE
                );
                break;
            case RIGHT:
                Actions.runBlocking(
                        RIGHT
                );
                break;
            case NONE:
                Actions.runBlocking(
                        RIGHT
                );
                break;
        }
    }
}