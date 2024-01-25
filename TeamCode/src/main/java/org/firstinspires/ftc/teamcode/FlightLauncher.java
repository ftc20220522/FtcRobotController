package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(group = "FINALCODE")
public class FlightLauncher extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoLauncher = hardwareMap.servo.get("servo5");
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servoLauncher.setPosition(1);
            }
        }
    }
}