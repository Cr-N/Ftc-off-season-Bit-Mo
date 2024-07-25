package org.firstinspires.ftc.teamcode.TrajectoryTesting;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.MasterWithActionsClass;

@Config
@Autonomous
public class RED_SHORT_MIDDLE_STACK_TEST extends LinearOpMode {
    MasterWithActionsClass master;
    public static double LINE_TO_Y_1_Y = -36;
    public static double WAIT_SECONDS_1 = 1;
    public static double LINE_TO_Y_2_Y = -37.7;

    public static double SPLINE_1_X = 59.4;
    public static double SPLINE_1_Y = -29;
    public static double SPLINE_1_HEADING = Math.PI;
    public static double SPLINE_1_TANGENT = 0;

    public static double WAIT_SECONDS_2 = 1.5;
    public static double WAIT_SECONDS_3 = 2;
    public static double PARK_X = 50;
    public static double PARK_Y = -60;
    public static double PARK_TANGENT = 0;

    Action traj;
    MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        traj = drive.actionBuilder(drive.pose)

                .lineToY(LINE_TO_Y_1_Y)
                .afterTime(0.1,master.intake.DEPLOY_1())
                .waitSeconds(WAIT_SECONDS_1)

                .lineToYConstantHeading(LINE_TO_Y_2_Y)

                .setReversed(true)
                .splineToLinearHeading(new Pose2d(SPLINE_1_X,SPLINE_1_Y,SPLINE_1_HEADING),SPLINE_1_TANGENT)

                .afterTime(0.1,master.Score_Yellow())
                .waitSeconds(WAIT_SECONDS_2)

                .afterTime(0.1,master.Prep_For_TeleOp())

                //.splineToConstantHeading(new Vector2d(PARK_X,PARK_Y),PARK_TANGENT)
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
