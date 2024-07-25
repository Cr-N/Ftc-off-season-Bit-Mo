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
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp(name="TeleOp special :)", group = "TELEOP")
public class TeleOp_Foarte_Bun extends LinearOpMode {
    MasterClass master;

    private MecanumDrive drive;
    private MecanumDrive driveInverted;
    DriveBase driveBase;
    GamepadEx gm1;
    SpecialGamepad gamepadButtonCheck;

    boolean INVERTED_CONTROLS;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializations of Subsystems, HardWare, GamepadEx, MecanumDrive,ButtonChecker
        {
            INVERTED_CONTROLS = false;
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            master = new MasterClass(this);
            driveBase = new DriveBase(this);
            driveBase.init();
            gm1 = new GamepadEx(gamepad1);
            drive = new MecanumDrive(false,driveBase.FL,driveBase.FR,driveBase.BL,driveBase.BR);
            driveInverted = new MecanumDrive(false,driveBase.BL,driveBase.BR,driveBase.FL,driveBase.FR);
            gamepadButtonCheck = new SpecialGamepad(this);

        }

        waitForStart();

        while (opModeIsActive()){
            if(INVERTED_CONTROLS == false)
            {
                drive.driveRobotCentric(gm1.getLeftX(), gm1.getLeftY(), gm1.getRightX(), true); // x,y,turn
            }
            else
            {
                driveInverted.driveRobotCentric(gm1.getLeftX(), -gm1.getLeftY(), gm1.getRightX(), true); // everything is inverted but the crab. Maybe for eotation too?
            }
            if(gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0){
                    driveBase.BL.set(gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                    driveBase.BR.set(-gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                    driveBase.FL.set(-gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
                    driveBase.FR.set(gm1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            }
            if(gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)>0){
                driveBase.BL.set(-gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
                driveBase.BR.set(gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
                driveBase.FL.set(gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
                driveBase.FR.set(-gm1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            }
            gamepadButtonCheck.updateCurrentStates();
            master.Slides_subsystem.UPDATE_Left_Right_Positions(); //TODO: Move this to where we change the mode After testing
            master.Color_sensor_subsystem.Handle_Pixel_Feedback();
            // Control Manual Slidere
            {
                if (gm1.isDown(GamepadKeys.Button.DPAD_UP)) {
                    master.Slides_subsystem.Handle_Manual_Control_UP();
                }
                if (gm1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    master.Slides_subsystem.Handle_Manual_Control_DOWN();
                }
                if (!gm1.isDown(GamepadKeys.Button.DPAD_UP)  && !gm1.isDown(GamepadKeys.Button.DPAD_DOWN)) {
                    master.Slides_subsystem.Handle_Stop_Motors_Manual_Control();
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
                master.Slides_subsystem.Change_Control_Mode();
            }
            if(gamepadButtonCheck.isPressed_Button_TouchPad()){
                if(INVERTED_CONTROLS == true)
                    INVERTED_CONTROLS = false;
                else
                    INVERTED_CONTROLS = true;
            }
            // Move Systems to Intake Position
            if(gamepadButtonCheck.isPressed_Button_A()) {
                master.handleLoweringArm();
            }

            // Move Rotate + Arm to intake position
            if(gamepadButtonCheck.isPressed_Button_Y()){
                master.Handle_Arm_Up();
                INVERTED_CONTROLS = true;
            }
            if(gamepadButtonCheck.isPressed_Button_Left_Bumper()){
                master.Claw_subsystem.Handle_Intaking();
            }
            if(gamepadButtonCheck.isPressed_Button_X()){
                if(master.Slides_subsystem.Get_Control_State() == Slides.Params.ControlState.MANUAL_MODE){
                    master.Slides_subsystem.Change_CONTROL_STATE_To_LEVELS_MODE();
                }
                master.Arm_Subsystem.Arm_To_Intermediary_Position();
                master.Rotate_subsystem.Rotate_To_Pick_Up_Position();

                if(master.Slides_subsystem.Get_Left_Pos() > 25 && master.Slides_subsystem.Get_Right_Pos() > 25){
                    master.Slides_subsystem.Move_To_LEVEL_INTAKE_POSITION();
                }
            }
            gamepadButtonCheck.updateLastStates();
            telemetry.addLine("//////////////////////SLIDES/////////////////////");
            telemetry.addData("Control State of Slides: ",master.Slides_subsystem.Get_Control_State());
            telemetry.addData("Level State of Slides: ",master.Slides_subsystem.Get_State_of_Slides());
            telemetry.addData("Left Pos: ",master.Slides_subsystem.Get_Left_Pos());
            telemetry.addData("Right Pos: ",master.Slides_subsystem.Get_Right_Pos());
            telemetry.addData("Left - Right : ",master.Slides_subsystem.Get_Left_Pos() - master.Slides_subsystem.Get_Right_Pos());
            telemetry.addData("Right - Left : ",master.Slides_subsystem.Get_Right_Pos() - master.Slides_subsystem.Get_Left_Pos());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addLine(" ");
            telemetry.addLine("///////////////////////ARM///////////////////////");
            telemetry.addData("State of Arm: ",master.Arm_Subsystem.Get_Arm_State());
            telemetry.addData("Angle of Arm: ",master.Arm_Subsystem.Get_Current_Arm_Angle());
            telemetry.addData("Position of Arm: ",master.Arm_Subsystem.Get_Current_Arm_Position());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addLine(" ");
            telemetry.addLine("/////////////////////ROTATE//////////////////////");
            telemetry.addData("Rotate State: ",master.Rotate_subsystem.Get_State_Of_Rotate());
            telemetry.addData("Rotate Angle: ",master.Rotate_subsystem.Get_Current_Rotate_Angle());
            telemetry.addData("Rotate Position: ",master.Rotate_subsystem.Get_Current_Rotate_Position());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addLine("///////////////COLOR SENSOR//////////////////////");
            telemetry.addData("Distance STATE:  ", master.Color_sensor_subsystem.Get_Distance_State());
            telemetry.addData("Distance(CM):  ", master.Color_sensor_subsystem.Get_Current_Distance_Param());
            telemetry.addData("Min Detection Distance :  ", master.Color_sensor_subsystem.Get_Minimum_Detection_Distance_Parameter());
            telemetry.addLine("/////////////////////////////////////////////////");
            telemetry.addData("INVERTED? ",INVERTED_CONTROLS);
            telemetry.update();

        }
    }
}
