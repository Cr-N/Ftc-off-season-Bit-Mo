/*package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardWare.SpecialGamepad;
import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
public class SlidersSubsystemTest extends LinearOpMode {
    Slides slides;
    GamepadEx gm1;
    SpecialGamepad specialGamepad;
    @Override
    public void runOpMode() throws InterruptedException {

        slides = new Slides(this);
        specialGamepad = new SpecialGamepad(this);
        gm1 = new GamepadEx(gamepad1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        slides.updateLeftRightPos();
        telemetry.addLine("Initialized");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            slides.updateLeftRightPos();
            specialGamepad.updateCurrentStates();

            if (specialGamepad.isPressed_Button_Dpad_Up()) {
                slides.handleLevelsUP();
            }
            if (specialGamepad.isPressed_Button_Dpad_Down()) {
                slides.handleLevelsDOWN();
            }
            if(specialGamepad.isPressed_Button_BACK()){
                slides.changeControlMode();
            }

            if(gamepad1.dpad_up){
                slides.handleManualControlUP();
            }
            if(gamepad1.dpad_down){
                slides.handleManualControlDOWN();
            }
            if(!gamepad1.dpad_up && !gamepad1.dpad_down){
                slides.handleStopMotorsManualControl();
            }
            specialGamepad.updateLastStates();
        }
        telemetry.addData("LeftPos: ",slides.getLeftPos());
        telemetry.addData("RightPos: ",slides.getRightPos());
        telemetry.update();

    }
}
*/