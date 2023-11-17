package org.firstinspires.ftc.teamcode;

import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;


@TeleOp(group = "FINALCODE")
public class RO_Meet1 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        //Center Odometery Wheel in Motor Port 0 (motor1 encoder)
        //Right Odometery Wheel in Motor Port 1 (motor2 encoder)
        //Left Odometery Wheel in Motor Port 2 (motor3 encoder)

        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor4");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor1");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor2");
        DcMotor motorIntake = hardwareMap.dcMotor.get("motor5");

        DcMotorEx motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motor6");
        motorSlideLeft.setDirection(DcMotorEx.Direction.REVERSE);
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorEx motorSlideRight= hardwareMap.get(DcMotorEx.class, "motor7");
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Servo servoLOT = hardwareMap.servo.get("servo2");
        Servo servoROT = hardwareMap.servo.get("servo1");
        servoROT.setDirection(Servo.Direction.REVERSE);
        Servo servoUpperOT =  hardwareMap.servo.get("servo3");
        Servo servoLowerOT =  hardwareMap.servo.get("servo4");
        Servo servoLauncher = hardwareMap.servo.get("servo5");


        /*
         Reverse the right side motors
         Reverse left motors if you are using NeveRests
         motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
         motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        */

        waitForStart();
        if (isStopRequested()) return;

        double y;
        double x;
        double rx;
        int rightPosition = 0;
        int leftPosition = 0;
        int rightPrevposition = 0;
        int leftPrevposition = 0;
        boolean a = false;

        while (opModeIsActive()) {


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // servo2.setDirection(Servo.Direction.REVERSE);
            if (gamepad1.right_trigger > 0) {
                y = -gamepad1.left_stick_y; // Remember, this is reversed!
                x = gamepad1.left_stick_x; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
            } else if (gamepad1.left_trigger > 0) {
                y = 0.25 * -gamepad1.left_stick_y; // Remember, this is reversed!
                x = 0.25 * gamepad1.left_stick_x; // Counteract imperfect strafing
                rx = 0.35 * gamepad1.right_stick_x;
            } else {
                y = -0.5 * gamepad1.left_stick_y; // Remember, this is reversed!
                x = 0.5 * gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
                rx = 0.65 * gamepad1.right_stick_x;
            }

            /*
             Denominator is the largest motor power (absolute value) or 1
             This ensures all the powers maintain the same ratio, but only when
             at least one is out of the range [-1, 1]
            */
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            motorFrontLeft.setPower(-frontLeftPower);
            motorBackLeft.setPower(-backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


//            telemetry.addData("odometer middle pos", motorBackRight.getCurrentPosition());
//            telemetry.addData("odometer right pos", motorFrontRight.getCurrentPosition());
//            telemetry.addData("odometer left pos", motorBackLeft.getCurrentPosition());
//            telemetry.update();

            //Viper Slide Preset
            if(gamepad2.x){
                rightPosition = 1200;
                leftPosition = 1200;
            }
            if(gamepad2.y) {
                rightPosition =  2023;
                leftPosition = 2023;
            }
            if(gamepad2.b) {
                rightPosition = 2850;
                leftPosition = 2850;
            }
            if(gamepad2.a) {
                rightPosition = 50;
                leftPosition = 50;
            }
            if (gamepad2.left_stick_y != 0) {
                motorSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorSlideRight.setVelocity(signum(gamepad2.left_stick_y)*2000);
                motorSlideLeft.setVelocity(signum(gamepad2.left_stick_y)*2000);
                rightPosition = motorSlideRight.getCurrentPosition();
                leftPosition = motorSlideLeft.getCurrentPosition();
                rightPrevposition = motorSlideRight.getCurrentPosition();
                leftPrevposition = motorSlideLeft.getCurrentPosition();
                a = true;
            } else if (a) {
                motorSlideRight.setVelocity(0);
                motorSlideLeft.setVelocity(0);
                a = false;
            }
//
            if (rightPrevposition != rightPosition && leftPrevposition != leftPosition && gamepad2.left_stick_y == 0) {
                motorSlideRight.setTargetPosition(rightPosition);
                motorSlideLeft.setTargetPosition(leftPosition);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightPrevposition = rightPosition;
                leftPrevposition = leftPosition;
                motorSlideRight.setVelocity(4000);
                motorSlideLeft.setVelocity(4000);
            }

            //Intake - motorIntake = "motor7"
            if (gamepad2.left_trigger > 0) {
                motorIntake.setPower(gamepad2.left_trigger);
            } else {
                motorIntake.setPower(0);
            }

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

            //Flight Launcher - servoLauncher = "servo5"
            if (gamepad1.a) {
                servoLauncher.setPosition(1);
            }

        }
    }
}