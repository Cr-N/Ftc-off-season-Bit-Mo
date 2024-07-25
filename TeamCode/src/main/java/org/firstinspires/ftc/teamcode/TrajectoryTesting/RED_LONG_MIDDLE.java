package org.firstinspires.ftc.teamcode.TrajectoryTesting;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous
public class RED_LONG_MIDDLE extends LinearOpMode {

    // parametize this one with public static double ... like the one we just did
    public static double LINE_TO_Y_1_Y = -36;
    public static double SPILNE_1_X = -51;
    public static double SPLINE_1_Y = -37;
    public static double SPLINE_1_HEADING = Math.PI/2;
    public static double SPLINE_1_TANGENT = 2;
    public static double WAIT_SECONDS_1 = 0.5;
    public static double STRAFE_1_VECTOR_X = -46.5;
    public static double STRAFE_1_VECTOR_Y = -43;
    public static double STRAFE_1_HEADING = Math.PI/2+Math.PI/6;
    public static double SPLINE_2_X = -35.4;
    public static double SPLINE_2_Y = -62;
    public static double SPLINE_2_HEADING = Math.PI;
    public static double SPLINE_2_TANGENT = 0;
    public static double STRAFE_2_VECTOR_X = 5;
    public static double STRAFE_2_VECTOR_Y = -58.9;
    public static double SPLINE_3_X = 48.4;
    public static double SPLINE_3_Y = -27;
    public static double SPLINE_3_TANGENT = 0;
    public static double WAIT_SECONDS_2 = 2;
    public static double PARK_X = 50;
    public static double PARK_Y = -60;

    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap,new Pose2d(-35.2, -63.2, Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
                //Spike mark LEFT
                .lineToY(LINE_TO_Y_1_Y)
                .waitSeconds(WAIT_SECONDS_2)

                //Go back a little
                //.strafeToSplineHeading(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y),STRAFE_1_HEADING)
                //.waitSeconds(WAIT_SECONDS_2)

                // Go to leaving spot
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(SPLINE_2_X,SPLINE_2_Y,SPLINE_2_HEADING),SPLINE_2_TANGENT)
                //.waitSeconds(WAIT_SECONDS_2)

                // Go forward a little(we cross from long to short)
                .strafeToConstantHeading(new Vector2d(STRAFE_2_VECTOR_X,STRAFE_2_VECTOR_Y))
                //.waitSeconds(WAIT_SECONDS_2)

                // Spline to Backboard on the LEFT side
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(SPLINE_3_X,SPLINE_3_Y),SPLINE_3_TANGENT)
                //.waitSeconds(WAIT_SECONDS_2)

                // Deploy Yellow
                .waitSeconds(WAIT_SECONDS_2)

                //Park in the corner
                //.strafeTo(new Vector2d(PARK_X,PARK_Y))
                .splineToConstantHeading(new Vector2d(PARK_X,PARK_Y),SPLINE_3_TANGENT)


                .build();
        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                traj
        );
    }
}