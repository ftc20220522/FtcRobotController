package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class servoLimitTest extends LinearOpMode{
    //.2->.5
    //.57&.75 HOT - Hook
    //0.83->0.55 TOT - Top
    //0-0.21 BOT - Bottom
    //0.1-0.2
    //Flap: close 0.51, open 0,66
    public void runOpMode() throws InterruptedException {
        Servo servoBOT = hardwareMap.servo.get("servo3");
        Servo servoTOT = hardwareMap.servo.get("servo2");
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
//            servo1.setPosition(0);
            //.52 Out - 67 in
            if (gamepad1.a) {
                servoBOT.setPosition(servoBOT.getPosition()+0.05);
                TimeUnit.MILLISECONDS.sleep(350);
            }
            if (gamepad1.b) {
                servoBOT.setPosition(servoBOT.getPosition()-0.05);
                TimeUnit.MILLISECONDS.sleep(350);
            }
            if (gamepad1.x) {
                servoBOT.setPosition(servoBOT.getPosition()-0.01);
                TimeUnit.MILLISECONDS.sleep(350);
            }
            if (gamepad1.y) {
                servoBOT.setPosition(servoBOT.getPosition()+0.01);
                TimeUnit.MILLISECONDS.sleep(350);
            }
            if (gamepad1.dpad_down) {
                servoTOT.setPosition(servoTOT.getPosition()+0.05);
                TimeUnit.MILLISECONDS.sleep(350);
            }
            if (gamepad1.dpad_right) {
                servoTOT.setPosition(servoTOT.getPosition()-0.05);
                TimeUnit.MILLISECONDS.sleep(350);
            }
            if (gamepad1.dpad_left) {
                servoTOT.setPosition(servoTOT.getPosition()-0.01);
                TimeUnit.MILLISECONDS.sleep(350);
            }
            if (gamepad1.dpad_up) {
                servoTOT.setPosition(servoTOT.getPosition()+0.01);
                TimeUnit.MILLISECONDS.sleep(350);
            }
            telemetry.addData("servo pos.", servoBOT.getPosition());
            telemetry.addData("servo pos.", servoTOT.getPosition());
            telemetry.update();
        }
    }
}