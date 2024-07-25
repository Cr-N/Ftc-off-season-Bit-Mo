package org.firstinspires.ftc.teamcode.TrajectoryTesting;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.MasterWithActionsClass;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.SlidesWithActionsForAutos;

@Config
@Autonomous
public class RED_SHORT_RIGHT extends LinearOpMode {

    double RIGHT_STRAFE_1_X= 28;
    double RIGHT_STRAFE_1_Y= -52;
    double RIGHT_SPLINE_1_X = 39.4;;
    double RIGHT_SPLINE_1_Y = -30;
    double RIGHT_SPLINE_1_HEADING = Math.PI;
    double RIGHT_SPLINE_1_TANGENT = 1;
    double RIGHT_WAIT_SECONDS_1 = 1;
    double RIGHT_SPLINE_2_X = 57.5;
    double RIGHT_SPLINE_2_Y = -41;
    double RIGHT_SPLINE_2_TANGENT = 0;
    double RIGHT_PARK_X = 55;
    double RIGHT_PARK_Y = -60;
    double RIGHT_PARK_TANGENT = 0;
    double RIGHT_WAIT_SECONDS_2 = 3;


    SlidesWithActionsForAutos slides;
    Action traj;
    MecanumDrive drive;
    MasterWithActionsClass master;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new SlidesWithActionsForAutos(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        master = new MasterWithActionsClass(this);
        traj = drive.actionBuilder(drive.pose)

                // SPIKE MARK RIGHT
                .strafeTo(new Vector2d(RIGHT_STRAFE_1_X, RIGHT_STRAFE_1_Y))
                .splineToLinearHeading(new Pose2d(RIGHT_SPLINE_1_X, RIGHT_SPLINE_1_Y, RIGHT_SPLINE_1_HEADING), RIGHT_SPLINE_1_TANGENT)
                .afterTime(2,master.intake.DEPLOY_1())
                .waitSeconds(RIGHT_WAIT_SECONDS_1)

                // Strafe to Backdrop
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(RIGHT_SPLINE_2_X,RIGHT_SPLINE_2_Y),RIGHT_SPLINE_2_TANGENT)
                .afterTime(0,master.Score_Yellow())
                .waitSeconds(RIGHT_WAIT_SECONDS_2)

                // Park
                .splineToConstantHeading(new Vector2d(RIGHT_PARK_X,RIGHT_PARK_Y),RIGHT_PARK_TANGENT)
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
