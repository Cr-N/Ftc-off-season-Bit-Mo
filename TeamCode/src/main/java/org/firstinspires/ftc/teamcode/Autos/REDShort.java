package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubsystemsWithActions.MasterWithActionsClass;
import org.firstinspires.ftc.teamcode.Vision.Red3BoxVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
// All in one approach /////////////////////////

/*
 * ┏┓┓ ┓   ┳┳┓┏┓┓┏┏┓┳┳┓┏┓┳┓┏┳┓  ┳┏┓  ┏┓┏┓┏┓┏┓┳┓┏┓┏┳┓┏┓┳┓  ┏┓┏┓┏┓┳┓┏┓┏┓┏┓┓┏
 * ┣┫┃ ┃   ┃┃┃┃┃┃┃┣ ┃┃┃┣ ┃┃ ┃   ┃┗┓  ┗┓┣ ┃┃┣ ┣┫┣┫ ┃ ┣ ┃┃  ┣┫┃┃┃┃┣┫┃┃┣┫┃ ┣┫
 * ┛┗┗┛┗┛  ┛ ┗┗┛┗┛┗┛┛ ┗┗┛┛┗ ┻   ┻┗┛  ┗┛┗┛┣┛┗┛┛┗┛┗ ┻ ┗┛┻┛  ┛┗┣┛┣┛┛┗┗┛┛┗┗┛┛┗
 */
public class REDShort extends LinearOpMode{
        public static double LEFT_LINE_TO_Y_1_Y = -40.2;
        public static double LEFT_SPLINE_1_X = 6;
        public static double LEFT_SPLINE_1_Y = -33.2;
        public static double LEFT_SPLINE_1_HEADING = 2.09;
        public static double LEFT_SPLINE_1_TANGENT = 1;
        public static double LEFT_WAIT_SECONDS_1 = 2;
        public static double LEFT_STRAFE_1_VECTOR_X = 13;
        public static double LEFT_STRAFE_1_VECTOR_Y = -36;
        public static double LEFT_SPLINE_2_X = 50;
        public static double LEFT_SPLINE_2_Y = -26;
        public static double LEFT_SPLINE_2_HEADING = Math.PI;
        public static double LEFT_SPLINE_2_TANGENT = 0;
        public static double LEFT_WAIT_SECONDS_2 = 3;
        public static double LEFT_STRAFE_2_VECTOR_X = 46;
        public static double LEFT_STRAFE_2_VECTOR_Y = -60;


        public static double MIDDLE_LINE_TO_Y_1_Y = -35;
        public static double MIDDLE_WAIT_SECONDS_1 = 1;
        public static double MIDDLE_LINE_TO_Y_2_Y = -37.7;
        public static double MIDDLE_SPLINE_1_X = 52;
        public static double MIDDLE_SPLINE_1_Y = -30;
        public static double MIDDLE_SPLINE_1_HEADING = Math.PI;
        public static double MIDDLE_SPLINE_1_TANGENT = 0;
        public static double MIDDLE_STRAFE_1_VECTOR_X = 50;
        public static double MIDDLE_STRAFE_1_VECTOR_Y = -60;
        public static double MIDDLE_WAIT_SECONDS_2 = 2;


    MasterWithActionsClass master;
    Red3BoxVisionProcessor REDvisionProcessor;
    MecanumDrive drive;
    Action LEFT;
    Action MIDDLE;
    Action RIGHT;
    VisionPortal visionPortal;
    Red3BoxVisionProcessor.Selected CASE;
    @Override
    public void runOpMode() throws InterruptedException {
        master = new MasterWithActionsClass(this);
        REDvisionProcessor = new Red3BoxVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), REDvisionProcessor);
        drive = new MecanumDrive(hardwareMap,new Pose2d(11.5, -63.2, Math.PI/2));
        // left case trajectory
        {
            LEFT = drive.actionBuilder(drive.pose)
                    // SPIKE MARK **LEFT**
                    .lineToY(LEFT_LINE_TO_Y_1_Y)

                    .splineToLinearHeading(new Pose2d(LEFT_SPLINE_1_X,LEFT_SPLINE_1_Y,LEFT_SPLINE_1_HEADING),LEFT_SPLINE_1_TANGENT)

                    .waitSeconds(LEFT_WAIT_SECONDS_1)

                    // Mergi putin in spate ca sa nu dai in pixelul MOV
                    .strafeTo(new Vector2d(LEFT_STRAFE_1_VECTOR_X,LEFT_STRAFE_1_VECTOR_Y))

                    // Spline catre Backdrop 1
                    .splineToLinearHeading(new Pose2d(LEFT_SPLINE_2_X,LEFT_SPLINE_2_Y,LEFT_SPLINE_2_HEADING),LEFT_SPLINE_2_TANGENT)

                    .waitSeconds(LEFT_WAIT_SECONDS_2)

                    // Parcare
                    .strafeTo(new Vector2d(LEFT_STRAFE_2_VECTOR_X,LEFT_STRAFE_2_VECTOR_Y))
                    .build();

        }
        // middle case trajectory
        {
            MIDDLE = drive.actionBuilder(drive.pose)
                    .afterTime(0.1,master.Prep_For_Purple())
                    .waitSeconds(5)
                    // SPIKE MARK **MIDDLE**
                    .lineToY(MIDDLE_LINE_TO_Y_1_Y)

                    .waitSeconds(MIDDLE_WAIT_SECONDS_1)

                    .lineToYConstantHeading(MIDDLE_LINE_TO_Y_2_Y)

                    // Spline catre Backdrop 1
                    .splineToLinearHeading(new Pose2d(MIDDLE_SPLINE_1_X,MIDDLE_SPLINE_1_Y,MIDDLE_SPLINE_1_HEADING),MIDDLE_SPLINE_1_TANGENT)
                    //.afterTime(0.2,master.Score_Yellow())
                    .waitSeconds(MIDDLE_WAIT_SECONDS_2)

                    .strafeTo(new Vector2d(MIDDLE_STRAFE_1_VECTOR_X,MIDDLE_STRAFE_1_VECTOR_Y))
                    //.stopAndAdd(master.Prep_For_TeleOp())
                    .build();
        }
        // right case trajectory
        {
            RIGHT = drive.actionBuilder(drive.pose)
                    .build();
        }
        while (opModeInInit() && !isStopRequested()){

            switch (REDvisionProcessor.getSelection()){
                case LEFT:

                    telemetry.addLine("       __   __ _____ ");
                    telemetry.addLine(" |    |    |     |   ");
                    telemetry.addLine(" |    |__  |__   |   ");
                    telemetry.addLine(" |    |    |     |   ");
                    telemetry.addLine(" |__  |__  |     |   ");
                    telemetry.update();
                    CASE = Red3BoxVisionProcessor.Selected.LEFT;
                    break;
                case RIGHT:
                    telemetry.addLine("       ***");
                    telemetry.addLine(" ****  ***  *********  **   ** *******");
                    telemetry.addLine(" *  *   *   *          **   **   **");
                    telemetry.addLine(" ****   *   *   *****  *******   **");
                    telemetry.addLine(" **     *   *      **  **   **   **");
                    telemetry.addLine(" *  *   *   *********  **   **   **");
                    telemetry.update();
                    CASE = Red3BoxVisionProcessor.Selected.LEFT;

                    break;
                case MIDDLE:
                    telemetry.addLine("  *       *    *   *     *     *      *     **** ");
                    telemetry.addLine("  * *   * *        * *   * *   *      *     *    ");
                    telemetry.addLine("  *   *   *    *   *  *  *  *  *      *     **** ");
                    telemetry.addLine("  *       *    *   * *   * *   *      *     *    ");
                    telemetry.addLine("  *       *    *   *     *     *****  ***** **** ");
                    telemetry.update();

                    CASE = Red3BoxVisionProcessor.Selected.LEFT;

                    break;
                case NONE:
                    telemetry.addLine("*     *  *****  *       *  **** ");
                    telemetry.addLine("* *   *  *   *  * *     *  *    ");
                    telemetry.addLine("*   * *  *   *  *   *   *  **** ");
                    telemetry.addLine("*    **  *   *  *     * *  *    ");
                    telemetry.addLine("*     *  *****  *       *  **** ");
                    telemetry.update();
                    CASE = Red3BoxVisionProcessor.Selected.RIGHT;
                    break;
            }
        }
        if(isStopRequested())return;
        switch (CASE){
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
        }
    }
}