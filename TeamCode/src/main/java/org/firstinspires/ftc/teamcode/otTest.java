package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class otTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Servo servoLOT = hardwareMap.servo.get("servo2");
        Servo servoROT = hardwareMap.servo.get("servo1");
        servoROT.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            //Top of D Pad - servoROT = "servo1" & servoLOT = "servo2"
            if (gamepad2.dpad_up) {
                if (servoROT.getPosition() > 0.7 && servoLOT.getPosition() > 0.7) {
                    servoROT.setPosition(0.1);
                    servoLOT.setPosition(0.1);
                    TimeUnit.MILLISECONDS.sleep(350);
                } else {
                    servoROT.setPosition(0.75);
                    servoLOT.setPosition(0.75);
                    TimeUnit.MILLISECONDS.sleep(350);
                }
            }
            //Left of D Pad - servoLOT = "servo2"
            if (gamepad2.dpad_left) {
                if (servoLOT.getPosition() > 0.7) {
                    servoLOT.setPosition(0.1);
                    TimeUnit.MILLISECONDS.sleep(350);
                } else {
                    servoLOT.setPosition(0.75);
                    TimeUnit.MILLISECONDS.sleep(350);
                }
            }
            //Right of D Pad - servoROT = "servo1"
            if (gamepad2.dpad_right) {
                if (servoROT.getPosition() > 0.7) {
                    servoROT.setPosition(0.1);
                    TimeUnit.MILLISECONDS.sleep(350);
                } else {
                    servoROT.setPosition(0.75);
                    TimeUnit.MILLISECONDS.sleep(350);
                }
            }
            telemetry.addData("servo pos", servoLOT.getPosition());
            telemetry.addData("servo pos", servoROT.getPosition());
            telemetry.update();
        }
    }
}
