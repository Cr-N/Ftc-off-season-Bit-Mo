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
public class RED_LONG_LEFT extends LinearOpMode {
    MasterWithActionsClass master;
    // parametize this one with public static double ... like the one we just did
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
   double LEFT_WAIT_SECONDS_2 = 2;
   double LEFT_PARK_X = 50;
   double LEFT_PARK_Y = -60;
    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        drive = new MecanumDrive(hardwareMap,new Pose2d(-35.2, -63.2, Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
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
                .afterTime(0.8,master.Score_Yellow())
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
