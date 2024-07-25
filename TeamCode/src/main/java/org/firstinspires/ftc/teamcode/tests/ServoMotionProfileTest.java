package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardWare.ServoMotionProfile;
import org.firstinspires.ftc.teamcode.HardWare.SpecialGamepad;
@Disabled
@Config
@TeleOp(name = "Servo Motion Profile Test")
public class ServoMotionProfileTest extends LinearOpMode {

    public SimpleServo myServo;
    public ServoMotionProfile servoMotionProfile;
    public SpecialGamepad specialGamepad;
    public static double pos1 = 0;
    public static double pos2 = 0.11;
    public static double pos3 = 0.22;
    public static double pos4 = 0.33;
    public static double pos5 = 0.44;
    public static double pos6 = 0.55;
    public static double pos7 = 0.66;
    public static double pos8 = 0.77;
    public static double pos9 = 0.88;
    public static double pos10 = 1;
    public static double MAXVEL = 1.0;
    public static double MAXACCEL = 0.5;
    public static int level=1;
    public static double low =0;
    public static double high=1;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        specialGamepad = new SpecialGamepad(this);
        // Initialize hardware
        //myServo = hardwareMap.get(Servo.class, "myServo");
        myServo = new SimpleServo(hardwareMap,"myServo",0,270,AngleUnit.DEGREES);

        // Create motion profile
        servoMotionProfile = new ServoMotionProfile(myServo, MAXVEL, MAXACCEL);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while(opModeIsActive()){

            specialGamepad.updateCurrentStates();
            if(specialGamepad.isPressed_Button_Dpad_Up()){
                if(level!=10){
                    level=level+1;
                    handleLevels();
                }
            }
            if(specialGamepad.isPressed_Button_Dpad_Down()){
                if(level!=1){
                    level=level-1;
                    handleLevels();
                }
            }
            if(specialGamepad.isPressed_Button_Left_Bumper()){
                servoMotionProfile.moveToPosition(low);
                level = 1;
            }
            if(specialGamepad.isPressed_Button_Right_Bumper()){
                servoMotionProfile.moveToPosition(high);
                level = 10;
            }
            specialGamepad.updateLastStates();
            telemetry.addData("angle ",myServo.getAngle());
            telemetry.addData("position ",myServo.getPosition());
            telemetry.addData("level ", level);
            telemetry.update();

        }
    }
    public void handleLevels(){
        switch (level){
            case 1:
                servoMotionProfile.moveToPosition(pos1);
                break;
            case 2:
                servoMotionProfile.moveToPosition(pos2);
                break;
            case 3:
                servoMotionProfile.moveToPosition(pos3);
                break;
            case 4:
                servoMotionProfile.moveToPosition(pos4);
                break;
            case 5:
                servoMotionProfile.moveToPosition(pos5);
                break;
            case 6:
                servoMotionProfile.moveToPosition(pos6);
                break;
            case 7:
                servoMotionProfile.moveToPosition(pos7);
                break;
            case 8:
                servoMotionProfile.moveToPosition(pos8);
                break;
            case 9:
                servoMotionProfile.moveToPosition(pos9);
                break;
            case 10:
                servoMotionProfile.moveToPosition(pos10);
                break;
        }
    }
}
