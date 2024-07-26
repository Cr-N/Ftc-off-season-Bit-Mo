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
public class BLUE_LONG_MIDDLE extends LinearOpMode {
    MasterWithActionsClass master;
    // parametize this one with public static double ... like the one we just did
    double MIDDLE_SPILNE_1_X = -39;
    double MIDDLE_SPLINE_1_Y =37.6;
    double MIDDLE_SPLINE_1_HEADING = 3*Math.PI/2; // PI/2
    double MIDDLE_SPLINE_1_TANGENT = -2;
    double MIDDLE_WAIT_SECONDS_1 = 2;
    double MIDDLE_STRAFE_1_VECTOR_X = -46.5;
    double MIDDLE_STRAFE_1_VECTOR_Y = -43;
    double MIDDLE_STRAFE_1_HEADING = Math.PI/2+Math.PI/6;
    double MIDDLE_SPLINE_2_X = -35.4;
    double MIDDLE_SPLINE_2_Y = 65;
    double MIDDLE_SPLINE_2_HEADING = Math.PI; // PI
    double MIDDLE_SPLINE_2_TANGENT = -0.1;
    double MIDDLE_STRAFE_2_VECTOR_X = 5;
    double MIDDLE_STRAFE_2_VECTOR_Y = 63;
    double MIDDLE_SPLINE_3_X = 48.4;
    double MIDDLE_SPLINE_3_Y = 35.4;
    double MIDDLE_SPLINE_3_TANGENT = 0;
    double MIDDLE_WAIT_SECONDS_2 = 3;
    double MIDDLE_PARK_X = 53;
    double MIDDLE_PARK_Y = 66;
    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        drive = new MecanumDrive(hardwareMap,new Pose2d(-35.2, 63.2, 3*Math.PI/2));
        traj = drive.actionBuilder(drive.pose)
                //Spike mark MIDDLE
                .splineToLinearHeading(new Pose2d(MIDDLE_SPILNE_1_X,MIDDLE_SPLINE_1_Y,MIDDLE_SPLINE_1_HEADING),MIDDLE_SPLINE_1_TANGENT)
                .afterTime(1,master.intake.DEPLOY_1())
                .waitSeconds(MIDDLE_WAIT_SECONDS_1)

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
                .afterTime(0.1,master.Score_Yellow_MIDDLE_SHORT())
                .waitSeconds(MIDDLE_WAIT_SECONDS_2)

                //Park in the corner
                //.strafeTo(new Vector2d(PARK_X,PARK_Y))
                .splineToConstantHeading(new Vector2d(MIDDLE_PARK_X,MIDDLE_PARK_Y),MIDDLE_SPLINE_3_TANGENT)
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
