package org.firstinspires.ftc.teamcode.Vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
@TeleOp(name = "TEST Detectie", group = "Vision")
public class TestVisionProcessorsOpMode extends OpMode {
    private HeightFilterBlue3Box visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        visionProcessor = new HeightFilterBlue3Box(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        visionPortal.resumeStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("Identified", visionProcessor.getSelection());
    }
}