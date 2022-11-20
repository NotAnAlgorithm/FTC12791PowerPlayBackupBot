package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystem.Webcam;
import org.firstinspires.ftc.teamcode.subsystem.vision.SignalDetectionPipeline;

@TeleOp(group = "test")
public class SignalDetectionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SignalDetectionPipeline pipeline = new SignalDetectionPipeline();
        Webcam webcam = new Webcam(hardwareMap, pipeline);

        sleep(500);

        waitForStart();

        while (opModeIsActive()) {
            sleep(50);
            if (pipeline.average == null) continue;
            telemetry.addData("color", "%f %f %f", pipeline.average.val[0], pipeline.average.val[1], pipeline.average.val[2]);
            telemetry.addData("detection", pipeline.position);
            telemetry.update();
        }
    }
}
