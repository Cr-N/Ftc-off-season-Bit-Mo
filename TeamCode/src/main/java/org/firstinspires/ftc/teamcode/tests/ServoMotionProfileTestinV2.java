package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@Config
@TeleOp(name = "SmoothServoControl")
public class ServoMotionProfileTestinV2 extends LinearOpMode {

    public SimpleServo servo;
   // public static double MAX_VELOCITY = 0.25; // Maximum change in position per update
    //public static double MAX_ACCELERATION = 0.7; // Maximum change in velocity per update
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //servo = hardwareMap.get(Servo.class, "myServo");
        servo = new SimpleServo(hardwareMap, "myServo", 0, 180, AngleUnit.DEGREES);
        waitForStart();
        while (opModeIsActive()){

            servo.turnToAngle(180);
            telemetry.addData("Target Position", 1);
            telemetry.addData("Current Position", servo.getPosition());
            telemetry.update();

            sleep(5000);
            servo.turnToAngle(0);
            telemetry.addData("Target Position", 0);
            telemetry.addData("Current Position", servo.getPosition());
            telemetry.update();
            sleep(5000);
            //moveToPosition(servo,0);
            //moveToPosition(servo,1);
        }

    }

    /*private void moveToPosition(SimpleServo servo, double targetPosition) {
        double currentPosition = servo.getPosition();
        double direction = Math.signum(targetPosition - currentPosition);

        double velocity = 0;
        double position = currentPosition;
        double previousTime = getRuntime();

        while (opModeIsActive() && Math.abs(position - targetPosition) > 0.01) {
            double currentTime = getRuntime();
            double deltaTime = currentTime - previousTime;

            double distanceRemaining = Math.abs(targetPosition - position);
            double decelerationDistance = (velocity * velocity) / (2 * MAX_ACCELERATION);

            if (distanceRemaining <= decelerationDistance) {
                velocity -= MAX_ACCELERATION * deltaTime;
            } else if (velocity < MAX_VELOCITY) {
                velocity += MAX_ACCELERATION * deltaTime;
            } else {
                velocity = MAX_VELOCITY;
            }

            velocity = Math.max(velocity, 0);
            position += direction * velocity * deltaTime;
            servo.setPosition(position);

            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("Current Position", position);
            telemetry.addData("Velocity", velocity);
            telemetry.update();

            previousTime = currentTime;
        }

        servo.setPosition(targetPosition);
    }*/
}
