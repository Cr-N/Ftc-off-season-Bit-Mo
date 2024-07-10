package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardWare.DriveBase;
import org.firstinspires.ftc.teamcode.HardWare.SpecialGamepad;
import org.firstinspires.ftc.teamcode.Subsystems.MasterClass;

@TeleOp
public class Drive_Arm_Rotate extends LinearOpMode {
    MasterClass master;

    private MecanumDrive drive;
    DriveBase driveBase;
    GamepadEx gm1;
    SpecialGamepad gamepadButtonCheck;



    @Override
    public void runOpMode() throws InterruptedException {
        // Initializations of Subsystems, HardWare, GamepadEx, MecanumDrive,ButtonChecker
        {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            master = new MasterClass(this);
            driveBase = new DriveBase(this);
            driveBase.init();
            gm1 = new GamepadEx(gamepad1);
            drive = new MecanumDrive(false,driveBase.FL,driveBase.FR,driveBase.BL,driveBase.BR);
            gamepadButtonCheck = new SpecialGamepad(this);

        }

        waitForStart();

        while (opModeIsActive()){
            drive.driveRobotCentric(gm1.getLeftX(), gm1.getLeftY(), gm1.getRightX(), false); // x,y,turn
            gamepadButtonCheck.updateCurrentStates();
            master.Slides_subsystem.updateLeftRightPos(); //TODO: Move this to where we change the mode After testing
            master.Color_sensor_subsystem.handle_pixel_feedback();
            // Control Manual Slidere
            {
                if (gm1.isDown(GamepadKeys.Button.DPAD_UP)) {
                    master.Slides_subsystem.handleManualControlUP();
                }
                if (gm1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    master.Slides_subsystem.handleManualControlDOWN();
                }
                if (!gm1.isDown(GamepadKeys.Button.DPAD_UP)  && !gm1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    master.Slides_subsystem.handleStopMotorsManualControl();
                }
            }

            // Control pe nivele Slidere
            {
                if (gamepadButtonCheck.isPressed_Button_Dpad_Up()) {
                    master.handle_slide_levels_UP();
                }
                if (gamepadButtonCheck.isPressed_Button_Dpad_Down()) {
                    master.handle_slide_levels_DOWN();
                }
            }

            // Changing Control States
            if(gamepadButtonCheck.isPressed_Button_START()){
                // normally, we would call Sliders_Subsystem.updateLeftRightPos(); ONLY HERE because we don't need to always call for the current position of the motors, only when changing MODES but since this is a test, we will call it at the start of the TeleOp to show the position difference in the Telemetry below
                //Sliders_Subsystem.updateLeftRightPos(); // We get the current position only when we really need it that would be for checkin for the levels adjustments so that it does not feel weird for the driver
                master.Slides_subsystem.changeControlMode();
            }
            // Move Systems to Intake Position
            if(gamepadButtonCheck.isPressed_Button_A()) {
                master.handleLoweringArm();
            }

            // Move Rotate + Arm to intake position
            if(gamepadButtonCheck.isPressed_Button_Y()){
                master.Arm_Subsystem.Arm_To_DeployPosition();
                master.Rotate_subsystem.Rotate_To_Deploy_Position();
                master.Slides_subsystem.UNLOCK_SLIDES();
            }
            if(gamepadButtonCheck.isPressed_Button_Left_Bumper()){
                master.Claw_subsystem.handleIntaking();
            }

            gamepadButtonCheck.updateLastStates();
            telemetry.addLine("//////////////////////SLIDES/////////////////////");
            telemetry.addData("Control State of Slides: ",master.Slides_subsystem.getControlState());
            telemetry.addData("Level State of Slides: ",master.Slides_subsystem.getStateofSlides());
            telemetry.addData("Left Pos: ",master.Slides_subsystem.getLeftPos());
            telemetry.addData("Right Pos: ",master.Slides_subsystem.getRightPos());
            telemetry.addData("Left - Right : ",master.Slides_subsystem.getLeftPos() - master.Slides_subsystem.getRightPos());
            telemetry.addData("Right - Left : ",master.Slides_subsystem.getRightPos() - master.Slides_subsystem.getLeftPos());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addLine(" ");
            telemetry.addLine("///////////////////////ARM///////////////////////");
            telemetry.addData("State of Arm: ",master.Arm_Subsystem.getArmState());
            telemetry.addData("Angle of Arm: ",master.Arm_Subsystem.getCurrent_Arm_Angle());
            telemetry.addData("Position of Arm: ",master.Arm_Subsystem.getCurrent_Arm_Position());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addLine(" ");
            telemetry.addLine("/////////////////////ROTATE//////////////////////");
            telemetry.addData("Rotate State: ",master.Rotate_subsystem.getStateOfRotate());
            telemetry.addData("Rotate Angle: ",master.Rotate_subsystem.getCurrent_Rotate_Arngle());
            telemetry.addData("Rotate Position: ",master.Rotate_subsystem.getCurrent_Rotate_Position());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addLine("///////////////COLOR SENSOR//////////////////////");
            telemetry.addData("Distance STATE:  ", master.Color_sensor_subsystem.get_distance_state());
            telemetry.addData("Distance(CM):  ", master.Color_sensor_subsystem.get_current_distance_param());
            telemetry.addData("Min Detection Distance :  ", master.Color_sensor_subsystem.get_minimum_detection_distance_param());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.update();

        }
    }
}
