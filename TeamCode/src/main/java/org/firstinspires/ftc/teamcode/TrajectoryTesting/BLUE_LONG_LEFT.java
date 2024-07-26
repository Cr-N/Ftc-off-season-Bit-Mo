package org.firstinspires.ftc.teamcode.TrajectoryTesting;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.MasterWithActionsClass;
@Disabled
@Config
@Autonomous
public class BLUE_LONG_LEFT extends LinearOpMode {
    MasterWithActionsClass master;
    double LEFT_STRAFE_1_X=-40;
    double LEFT_STRAFE_1_Y=45;
    double LEFT_SPILNE_1_X = -37;
    double LEFT_SPLINE_1_Y = 36;
    double LEFT_SPLINE_1_HEADING = Math.toRadians(270+60);
    double LEFT_SPLINE_1_TANGENT = 0;
    double LEFT_WAIT_SECONDS_1 = 0.5;
    double LEFT_STRAFE_1_VECTOR_X = -46.5;
    double LEFT_STRAFE_1_VECTOR_Y = -43;
    double LEFT_STRAFE_1_HEADING = Math.PI/2+Math.PI/6;
    double LEFT_SPLINE_2_X = -35.4;
    double LEFT_SPLINE_2_Y = 60;
    double LEFT_SPLINE_2_HEADING = Math.PI;
    double LEFT_SPLINE_2_TANGENT = 0;
    double LEFT_STRAFE_2_VECTOR_X = 5;
    double LEFT_STRAFE_2_VECTOR_Y = 63;
    double LEFT_SPLINE_3_X = 45.6;
    double LEFT_SPLINE_3_Y = 40.6; // -22.4
    double LEFT_SPLINE_3_TANGENT = 0;
    double LEFT_WAIT_SECONDS_2 = 2;
    double LEFT_PARK_X = 55;
    double LEFT_PARK_Y = 64;

    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        drive = new MecanumDrive(hardwareMap,new Pose2d(-35.2, 63.2, 3*Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
                //Spike mark LEFT
                .strafeToConstantHeading(new Vector2d(LEFT_STRAFE_1_X,LEFT_STRAFE_1_Y))
                .splineToLinearHeading(new Pose2d(LEFT_SPILNE_1_X,LEFT_SPLINE_1_Y,LEFT_SPLINE_1_HEADING),LEFT_SPLINE_1_TANGENT)
                // .splineTo(new Vector2d(SPILNE_1_X,SPLINE_1_Y),SPLINE_1_TANGENT)
                .afterTime(0.5,master.intake.DEPLOY_1())
                .waitSeconds(LEFT_WAIT_SECONDS_2)

                //Go back a little
                //.strafeToSplineHeading(new Vector2d(STRAFE_1_VECTOR_X,STRAFE_1_VECTOR_Y),STRAFE_1_HEADING)
                //.waitSeconds(WAIT_SECONDS_2)

                // Go to leaving spot
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(LEFT_SPLINE_2_X,LEFT_SPLINE_2_Y,LEFT_SPLINE_2_HEADING),LEFT_SPLINE_2_TANGENT)
                //.waitSeconds(WAIT_SECONDS_2)

                // Go forward a little(we cross from long to short)
                .strafeToConstantHeading(new Vector2d(LEFT_STRAFE_2_VECTOR_X,LEFT_STRAFE_2_VECTOR_Y))
                //.waitSeconds(WAIT_SECONDS_2)

                // Spline to Backboard on the RIGHT side
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(LEFT_SPLINE_3_X,LEFT_SPLINE_3_Y),LEFT_SPLINE_3_TANGENT)
                //.waitSeconds(WAIT_SECONDS_2)

                // Deploy Yellow
                .afterTime(0.5,master.Score_Yellow())
                .waitSeconds(LEFT_WAIT_SECONDS_2)

                //Park in the corner
                //.strafeTo(new Vector2d(PARK_X,PARK_Y))
                .splineToConstantHeading(new Vector2d(LEFT_PARK_X,LEFT_PARK_Y),LEFT_SPLINE_3_TANGENT)


                .build();
        Actions.runBlocking(
                master.Prep_For_Purple()
        );
        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                traj
        );
    }
}