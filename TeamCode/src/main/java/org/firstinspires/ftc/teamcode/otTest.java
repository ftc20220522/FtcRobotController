package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class otTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.servo.get("servo1");
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (servo.getPosition() > 0.7) {
                servo.setPosition(0.1);
                TimeUnit.MILLISECONDS.sleep(350);
            } else {
                servo.setPosition(0.75);
                TimeUnit.MILLISECONDS.sleep(350);
            }
        }
    }
}
