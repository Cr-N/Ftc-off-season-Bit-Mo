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
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.SlidesWithActionsForAutos;
@Disabled
@Config
@Autonomous
public class BLUE_SHORT_LEFT extends LinearOpMode {

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


    SlidesWithActionsForAutos slides;
    Action traj;
    MecanumDrive drive;
    MasterWithActionsClass master;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new SlidesWithActionsForAutos(hardwareMap);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, 63.2, 3*Math.PI/2));
        master = new MasterWithActionsClass(this);
        traj = drive.actionBuilder(drive.pose)

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
