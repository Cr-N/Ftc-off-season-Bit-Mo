package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Slides;

@TeleOp
public class SlidersSubsystemTest extends LinearOpMode {
    Slides slides;
    boolean lastDpadUp = false , currentDpadUp, lastDpadDown = false , currentDpadDown,lastShareButton = false , currentShareButton;
    @Override
    public void runOpMode() throws InterruptedException {
        slides = new Slides(this);
        GamepadEx gm1 = new GamepadEx(gamepad1);
        waitForStart();
        while(opModeIsActive()){

            // States for button presses
            {
                currentDpadDown = gm1.isDown(GamepadKeys.Button.DPAD_DOWN);
                currentDpadUp = gm1.isDown(GamepadKeys.Button.DPAD_UP);
                currentShareButton = gm1.isDown(GamepadKeys.Button.BACK);
            }
            // LEVELS control checks
            if (currentDpadUp && !lastDpadUp) {
                slides.handleLevelsUP();
            }
            if (currentDpadDown && !lastDpadDown) {
                slides.handleLevelsDOWN();

            }
            if(currentShareButton && !lastShareButton){
                slides.changeControlMode();
            }

            if(currentDpadUp){
                slides.handleManualControlUP();
            }
            if(currentDpadDown){
                slides.handleManualControlDOWN();
            }
            if(gamepad1.atRest()){
                slides.handleStopMotorsManualControl();
            }
            // States for Buttons Presses
            {
                lastDpadDown = currentDpadDown;
                lastDpadUp = currentDpadUp;
                lastShareButton = currentShareButton;
            }

        }

    }
}
