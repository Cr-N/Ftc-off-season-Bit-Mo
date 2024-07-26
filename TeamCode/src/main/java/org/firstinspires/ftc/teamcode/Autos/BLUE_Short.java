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
import org.firstinspires.ftc.teamcode.Vision.HeightFilterBlue3Box;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class BLUE_Short extends LinearOpMode{
    double LEFT_STRAFE_1_X= 28;
    double LEFT_STRAFE_1_Y= 52;
    double LEFT_SPLINE_1_X = 41;
    double LEFT_SPLINE_1_Y = 30;
    double LEFT_SPLINE_1_HEADING = Math.PI;
    double LEFT_SPLINE_1_TANGENT = 1;
    double LEFT_WAIT_SECONDS_1 = 1;
    double LEFT_SPLINE_2_X = 58.7;
    double LEFT_SPLINE_2_Y = 43;
    double LEFT_SPLINE_2_TANGENT = 0;
    double LEFT_PARK_X = 55;
    double LEFT_PARK_Y = 60;
    double LEFT_PARK_TANGENT = 0;
    double LEFT_WAIT_SECONDS_2 = 5;


    double MIDDLE_LINE_TO_Y_1_Y = 36;
    double MIDDLE_WAIT_SECONDS_1 = 2.5;
    double MIDDLE_LINE_TO_Y_2_Y = 35.7;
    double MIDDLE_SPLINE_1_X = 59.4;
    double MIDDLE_SPLINE_1_Y = 21.7;
    double MIDDLE_SPLINE_1_HEADING = Math.PI;
    double MIDDLE_SPLINE_1_TANGENT = 0;
    double MIDDLE_WAIT_SECONDS_2 = 2;
    double MIDDLE_WAIT_SECONDS_3 = 2;
    double MIDDLE_PARK_X = 58;
    double MIDDLE_PARK_Y = 57;
    double MIDDLE_PARK_TANGENT = 0;


    double RIGHT_LINE_TO_Y_1_Y = 46;
    double RIGHT_SPLINE_1_X = 15;
    double RIGHT_SPLINE_1_Y = 38;
    double RIGHT_SPLINE_1_HEADING = 2.09+(Math.PI/2);
    double RIGHT_SPLINE_1_TANGENT = 1;
    double RIGHT_WAIT_SECONDS_1 = 1;
    double RIGHT_STRAFE_1_VECTOR_X = 15;
    double RIGHT_STRAFE_1_VECTOR_Y = 39;
    double RIGHT_SPLINE_2_X = 53;
    double RIGHT_SPLINE_2_Y = 18;
    double RIGHT_SPLINE_2_HEADING = Math.PI;
    double RIGHT_SPLINE_2_TANGENT = 0;
    double RIGHT_WAIT_SECONDS_2 = 3.5;
    double RIGHT_PARK_X = 53;
    double RIGHT_PARK_Y = 58;
    double RIGHT_PARK_TANGENT = 0;

    MasterWithActionsClass master;
    HeightFilterBlue3Box BLUEvisionProcessor;
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
    //HeightFilterBlue3Box.Selected CASE;
    @Override
    public void runOpMode() throws InterruptedException {
        CASE detectionCase = CASE.NONE;
        master = new MasterWithActionsClass(this);
        BLUEvisionProcessor = new HeightFilterBlue3Box();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), BLUEvisionProcessor);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, 63.2, 3*Math.PI/2));
        // left case trajectory
        {
            LEFT = drive.actionBuilder(drive.pose)
                    // SPIKE MARK RIGHT
                    .strafeTo(new Vector2d(LEFT_STRAFE_1_X, LEFT_STRAFE_1_Y))
                    .splineToLinearHeading(new Pose2d(LEFT_SPLINE_1_X, LEFT_SPLINE_1_Y, LEFT_SPLINE_1_HEADING), LEFT_SPLINE_1_TANGENT)
                    .afterTime(2,master.intake.DEPLOY_1())
                    .waitSeconds(LEFT_WAIT_SECONDS_1)

                    // Spline to Backdrop
                    .setReversed(true)
                    .splineToConstantHeading(new Vector2d(LEFT_SPLINE_2_X,LEFT_SPLINE_2_Y),LEFT_SPLINE_2_TANGENT)
                    .afterTime(0.1,master.Score_Yellow_LEFT_BLUE_SHORT())
                    .waitSeconds(LEFT_WAIT_SECONDS_2)

                    // Park
                    .splineToConstantHeading(new Vector2d(LEFT_PARK_X,LEFT_PARK_Y),LEFT_PARK_TANGENT)
                    .build();

        }
        // middle case trajectory
        {
            MIDDLE = drive.actionBuilder(drive.pose)
                    .lineToY(MIDDLE_LINE_TO_Y_1_Y)
                    .afterTime(0,master.intake.DEPLOY_1())
                    .waitSeconds(MIDDLE_WAIT_SECONDS_1)

                    .lineToYConstantHeading(MIDDLE_LINE_TO_Y_2_Y)

                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(MIDDLE_SPLINE_1_X,MIDDLE_SPLINE_1_Y,MIDDLE_SPLINE_1_HEADING),MIDDLE_SPLINE_1_TANGENT)

                    .afterTime(0.1,master.Score_Yellow_MIDDLE_SHORT())
                    .waitSeconds(MIDDLE_WAIT_SECONDS_2)

                    .afterTime(0.1,master.Prep_For_TeleOp())

                    .splineToConstantHeading(new Vector2d(MIDDLE_PARK_X,MIDDLE_PARK_Y),MIDDLE_PARK_TANGENT)
                    .build();
        }
        // right case trajectory
        {
            RIGHT = drive.actionBuilder(drive.pose)
                    // SPIKE MARK **LEFT**
                    .lineToY(RIGHT_LINE_TO_Y_1_Y)

                    .splineToLinearHeading(new Pose2d(RIGHT_SPLINE_1_X,RIGHT_SPLINE_1_Y,RIGHT_SPLINE_1_HEADING),RIGHT_SPLINE_1_TANGENT)
                    .afterTime(0.6 ,master.intake.DEPLOY_1())
                    .waitSeconds(RIGHT_WAIT_SECONDS_1)

                    // Mergi putin in spate ca sa nu dai in pixelul MOV
                    //.strafeTo(new Vector2d(RIGHT_STRAFE_1_VECTOR_X,RIGHT_STRAFE_1_VECTOR_Y))

                    // Spline catre Backdrop 1
                    .setReversed(true)
                    .splineToLinearHeading(new Pose2d(RIGHT_SPLINE_2_X,RIGHT_SPLINE_2_Y,RIGHT_SPLINE_2_HEADING),RIGHT_SPLINE_2_TANGENT)
                    .afterTime(0.1,master.Score_Yellow())
                    .waitSeconds(RIGHT_WAIT_SECONDS_2)

                    // Parcare
                    .splineToConstantHeading(new Vector2d(RIGHT_PARK_X,RIGHT_PARK_Y),RIGHT_PARK_TANGENT)
                    .build();
        }
        Actions.runBlocking(
                master.Prep_For_Purple()
        );

        while (!opModeIsActive() && !isStopRequested()){
            switch (BLUEvisionProcessor.getSelection()){
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
                        LEFT
                );
                break;
        }
    }
}