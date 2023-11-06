package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class servoTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Servo servoL = hardwareMap.servo.get("servo1");
        servoL.setDirection(Servo.Direction.REVERSE);
        Servo servoR = hardwareMap.servo.get("servo2");
        servoR.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (gamepad1.a) {
                if (servoL.getPosition()>0.7) {
                    servoL.setPosition(0.1);
                    servoR.setPosition(0.75);
                    TimeUnit.MILLISECONDS.sleep(350);
                } else {
                    servoL.setPosition(0.75);
                    servoR.setPosition(0.1);
                    TimeUnit.MILLISECONDS.sleep(350);
                }
            }
            telemetry.addData("servo pos.", servoL.getPosition());
            telemetry.update();
        }
    }
}
