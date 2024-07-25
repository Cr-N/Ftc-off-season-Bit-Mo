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
public class RED_LONG_RIGHT extends LinearOpMode {

    double SPILNE_1_X = -37;
    double SPLINE_1_Y = -36;
    double SPLINE_1_HEADING = Math.toRadians(20);
    double SPLINE_1_TANGENT = 0;
    double WAIT_SECONDS_1 = 0.5;
    double STRAFE_1_VECTOR_X = -46.5;
    double STRAFE_1_VECTOR_Y = -43;
    double STRAFE_1_HEADING = Math.PI/2+Math.PI/6;
    double SPLINE_2_X = -35.4;
    double SPLINE_2_Y = -62;
    double SPLINE_2_HEADING = Math.PI;
    double SPLINE_2_TANGENT = 0;
    double STRAFE_2_VECTOR_X = 5;
    double STRAFE_2_VECTOR_Y = -59.9;
    double SPLINE_3_X = 48.4;
    double SPLINE_3_Y = -39; // -22.4
    double SPLINE_3_TANGENT = 0;
    double WAIT_SECONDS_2 = 2;
    double PARK_X = 50;
    double PARK_Y = -60;
    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap,new Pose2d(-35.2, -63.2, Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
                //Spike mark LEFT
                .strafeToConstantHeading(new Vector2d(-40,-45))
                .splineToLinearHeading(new Pose2d(SPILNE_1_X,SPLINE_1_Y,SPLINE_1_HEADING),SPLINE_1_TANGENT)
                // .splineTo(new Vector2d(SPILNE_1_X,SPLINE_1_Y),SPLINE_1_TANGENT)
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

                // Spline to Backboard on the RIGHT side
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