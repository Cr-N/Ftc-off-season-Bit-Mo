package org.firstinspires.ftc.teamcode.HardWare;

import com.arcrobotics.ftclib.hardware.SimpleServo;

public class ServoMotionProfile {

    private SimpleServo servo;
    private double maxVelocity;
    private double maxAcceleration;
    private double targetPosition;
    private double startPosition;

    public ServoMotionProfile(SimpleServo servo, double maxVelocity, double maxAcceleration) {
        this.servo = servo;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void moveToPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        this.startPosition = servo.getPosition();

        // Calculate motion profile parameters
        double distance = targetPosition - startPosition;
        double tAccel = maxVelocity / maxAcceleration;
        double dAccel = 0.5 * maxAcceleration * tAccel * tAccel;

        double tTotal, tConst;
        if (2 * dAccel >= Math.abs(distance)) {
            // Triangular profile
            tAccel = Math.sqrt(Math.abs(distance) / maxAcceleration);
            tConst = 0;
            tTotal = 2 * tAccel;
        } else {
            // Trapezoidal profile
            double dConst = Math.abs(distance) - 2 * dAccel;
            tConst = dConst / maxVelocity;
            tTotal = 2 * tAccel + tConst;
        }

        // Determine the direction
        int direction = distance > 0 ? 1 : -1;

        long startTime = System.currentTimeMillis();
        double currentTime = 0;

        while (currentTime < tTotal) {
            currentTime = (System.currentTimeMillis() - startTime) / 1000.0; // Convert to seconds
            double elapsedTime = currentTime;

            double currentPosition;
            if (elapsedTime < tAccel) {
                // Acceleration phase
                currentPosition = startPosition + 0.5 * maxAcceleration * elapsedTime * elapsedTime * direction;
            } else if (elapsedTime < tAccel + tConst) {
                // Constant velocity phase
                currentPosition = startPosition + (0.5 * maxAcceleration * tAccel * tAccel + maxVelocity * (elapsedTime - tAccel)) * direction;
            } else {
                // Deceleration phase
                double elapsedTimeDecel = elapsedTime - tAccel - tConst;
                currentPosition = startPosition + (0.5 * maxAcceleration * tAccel * tAccel + maxVelocity * tConst + (maxVelocity * elapsedTimeDecel - 0.5 * maxAcceleration * elapsedTimeDecel * elapsedTimeDecel)) * direction;
            }

            servo.setPosition(currentPosition);
            try {
                Thread.sleep(10); // Sleep for 10ms
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        // Ensure the servo reaches the final position
        servo.setPosition(targetPosition);;
    }
}
