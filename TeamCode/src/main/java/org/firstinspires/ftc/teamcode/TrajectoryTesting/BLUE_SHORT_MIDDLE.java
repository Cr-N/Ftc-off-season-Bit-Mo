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
public class BLUE_SHORT_MIDDLE extends LinearOpMode {
    MasterWithActionsClass master;
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

    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, 63.2, 3*Math.PI/2));
        traj = drive.actionBuilder(drive.pose)

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
