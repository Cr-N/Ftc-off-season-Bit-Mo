package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardWare.DriveBase;
import org.firstinspires.ftc.teamcode.HardWare.SpecialGamepad;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Rotate;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
public class Drive_Arm_Rotate extends LinearOpMode {
    Slides Sliders_Subsystem;
    Arm Arm_Subsystem;
    Rotate Rotate_Subsystem;
    private MecanumDrive drive;
    DriveBase driveBase;
    GamepadEx gm1;
    ColorSensor colorSensor;
    SpecialGamepad gamepadButtonCheck;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializations of Subsystems, HardWare, GamepadEx, MecanumDrive,ButtonChecker
        {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            Arm_Subsystem = new Arm(this);
            Rotate_Subsystem = new Rotate(this);
            driveBase = new DriveBase(this);
            driveBase.init();
            gm1 = new GamepadEx(gamepad1);
            drive = new MecanumDrive(false,driveBase.FL,driveBase.FR,driveBase.BL,driveBase.BR);
            gamepadButtonCheck = new SpecialGamepad(this);
            Sliders_Subsystem = new Slides(this);
            colorSensor = new ColorSensor(this);
        }
        waitForStart();
        while (opModeIsActive()){
            drive.driveRobotCentric(gm1.getLeftX(), gm1.getLeftY(), gm1.getRightX(), false); // x,y,turn
            {
                gamepadButtonCheck.updateCurrentStates();
                Sliders_Subsystem.updateLeftRightPos(); //TODO: Move this to where we change the mode After testing
                colorSensor.updateSensorData();
                colorSensor.handleColorDetections();
                colorSensor.handleDistanceDetections();
                colorSensor.handle_Driver_Feedback_For_Pixels();
            }
            // Control Manual Slidere
            {
                if (gamepadButtonCheck.isPressed_Button_Dpad_Up() && Sliders_Subsystem.getControlState() == Slides.Params.ControlState.MANUAL_MODE) {
                    Sliders_Subsystem.handleManualControlUP();
                }
                if (gamepadButtonCheck.isPressed_Button_Dpad_Down() && Sliders_Subsystem.getControlState() == Slides.Params.ControlState.MANUAL_MODE) {
                    Sliders_Subsystem.handleManualControlDOWN();
                }
                if (!gamepadButtonCheck.isPressed_Button_Dpad_Up() && !gamepadButtonCheck.isPressed_Button_Dpad_Down() && Sliders_Subsystem.getControlState() == Slides.Params.ControlState.MANUAL_MODE) {
                    Sliders_Subsystem.handleStopMotorsManualControl();
                }
            }
            // Control pe nivele Slidere
            {
                if (gamepadButtonCheck.isPressed_Button_Dpad_Up() && Sliders_Subsystem.getControlState() == Slides.Params.ControlState.LEVELS_MODE) {
                    Sliders_Subsystem.handleLevelsUP();
                }
                if (gamepadButtonCheck.isPressed_Button_Dpad_Down() && Sliders_Subsystem.getControlState() == Slides.Params.ControlState.LEVELS_MODE) {
                    Sliders_Subsystem.handleLevelsDOWN();
                }
            }
            // Changing Control States
            if(gamepadButtonCheck.isPressed_Button_START()){
                // normally, we would call Sliders_Subsystem.updateLeftRightPos(); ONLY HERE because we don't need to always call for the current position of the motors, only when changing MODES but since this is a test, we will call it at the start of the TeleOp to show the position difference in the Telemetry below
                //Sliders_Subsystem.updateLeftRightPos(); // We get the current position only when we really need it that would be for checkin for the levels adjustments so that it does not feel weird for the driver
                Sliders_Subsystem.changeControlMode();
            }
            // Move Systems to Intake Position
            if(gamepadButtonCheck.isPressed_Button_A()){
                Rotate_Subsystem.Rotate_To_Pick_Up_Position();
                Sliders_Subsystem.moveToLEVEL_IntakePosition();
                Arm_Subsystem.Arm_To_Pick_Up_Position();
            }
            // Move Rotate + Arm to intake position
            if(gamepadButtonCheck.isPressed_Button_Y()){
                Rotate_Subsystem.Rotate_To_Deploy_Position();
                Arm_Subsystem.Arm_To_DeployPosition();
            }
            gamepadButtonCheck.updateLastStates();
            telemetry.addLine("//////////////////////SLIDES/////////////////////");
            telemetry.addData("Control State of Slides: ",Sliders_Subsystem.getControlState());
            telemetry.addData("Level State of Slides: ",Sliders_Subsystem.getStateofSlides());
            telemetry.addData("Left Pos: ",Sliders_Subsystem.getLeftPos());
            telemetry.addData("Right Pos: ",Sliders_Subsystem.getRightPos());
            telemetry.addData("Left - Right : ",Sliders_Subsystem.getLeftPos() - Sliders_Subsystem.getRightPos());
            telemetry.addData("Right - Left : ",Sliders_Subsystem.getRightPos() - Sliders_Subsystem.getLeftPos());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addLine("\n");
            telemetry.addLine("///////////////////////ARM///////////////////////");
            telemetry.addData("State of Arm: ",Arm_Subsystem.getArmState());
            telemetry.addData("Angle of Arm: ",Arm_Subsystem.getCurrent_Arm_Angle());
            telemetry.addData("Position of Arm: ",Arm_Subsystem.getCurrent_Arm_Position());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addLine("\n");
            telemetry.addLine("/////////////////////ROTATE//////////////////////");
            telemetry.addData("Rotate State: ",Rotate_Subsystem.getStateOfRotate());
            telemetry.addData("Rotate Angle: ",Rotate_Subsystem.getStateOfRotate());
            telemetry.addData("Rotate Position: ",Rotate_Subsystem.getStateOfRotate());
            telemetry.addLine("/////////////////////////////////////////////////");


        }
    }
}
