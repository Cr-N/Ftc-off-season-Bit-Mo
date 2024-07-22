package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp
public class TestServoActions extends LinearOpMode {

    SimpleServo servo;
    public static double servoError =0.5;
    public static double low =100;
    public static double high =160;


    @Override
    public void runOpMode() throws InterruptedException {

        servo = new SimpleServo(hardwareMap,"myServo",0,180,AngleUnit.DEGREES);
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        MoveToLow(),
//                        new SleepAction(2),

                        MoveToHigh(),
//                        new SleepAction(2),

                        MoveToLow(),
//                        new SleepAction(2),

                        MoveToHigh()
//                        new SleepAction(2)

                    )
        );
    }


    public class MoveToLow implements Action {
        private boolean initialized=false;
        private double servoAngle;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(initialized == false){
                servo.turnToAngle(low);
                initialized = true;
            }

            servoAngle = servo.getAngle();

            telemetryPacket.put("angle: ", servoAngle);

            telemetry.addLine("Running to LOW");
            telemetry.update();

            if((servoAngle <= low && servoAngle >=low - servoError) || (servoAngle >= low && servoAngle <=low + servoError) || (servoAngle == low)){
                return false;
            }
            else {
                return true;
            }
        }
    }

    public class MoveToHigh implements Action {
        private boolean initialized;
        private double servoAngle;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(initialized == false){
                servo.turnToAngle(high);
                initialized = true;
            }

            servoAngle = servo.getAngle();

            telemetryPacket.put("angle: ", servoAngle);

            telemetry.addData("angle:  ",servoAngle);
            telemetry.addLine("Running to HIGH");
            telemetry.update();

            if((servoAngle <= high && servoAngle >=high - servoError) || (servoAngle >= high && servoAngle <=high + servoError) || (servoAngle == high)){
                // return false STOPS ACTION
                return false;
            }
            else {
                return true;
            }
        }
    }
    public Action MoveToLow() {
        return new MoveToLow();
    }
    public Action MoveToHigh() {
        return new MoveToHigh();
    }
}
