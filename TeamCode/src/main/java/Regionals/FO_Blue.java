package Regionals;

import static java.lang.Math.signum;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "FINALCODE")
public class FO_Blue extends OpMode {

    //Center Odometery Wheel in Motor Port 0 (motor1 encoder)
    //Right Odometery Wheel in Motor Port 1 (motor2 encoder)
    //Left Odometery Wheel in Motor Port 2 (motor3 encoder)

//    DcMotor motorBackRight;
//    DcMotor motorFrontRight;
//    DcMotor motorBackLeft;
//    DcMotor motorFrontLeft;
    DcMotor motorIntake;
    DcMotor motorLauncher;
    DcMotorEx motorSlideLeft;
    DcMotorEx motorSlideRight;
    DcMotor temp;

    Servo servoClamp;
    Servo servoHOT;
    Servo servoFOT;
    Servo servoTOT;
    Servo servoBOT;
    Servo servoFL;
//    IMU imu;
    SampleMecanumDrive drive;
    Pose2d startPose;
//    CRServo servoInt;

    double y;
    double x;
    double rx;
    int position = 60;
    int prevposition = 0;
    boolean a = false;
    boolean pull = false;
    boolean intrun = false;
    int speed = 4000;
    long start;
    long end = 0;
    boolean settime = false;

    public enum ArmState {
        Bottom,
        RotateUp,
        Drop,
        Drop2,
        DownCheck,
        RotateDown,
    }
    public enum HookState {
        In,
        Out,
    }
    public enum RigState {
        Bottom,
        Extend,
        Rigged,
    }
    public enum LaunchState {
        PlaneIn,
        PlaneSent,
    }
    ElapsedTime liftTimer = new ElapsedTime();
    ElapsedTime hookTimer = new ElapsedTime();
    ElapsedTime rigTimer = new ElapsedTime();
    ElapsedTime launchTimer = new ElapsedTime();
    ArmState armState = ArmState.Bottom;
    HookState hState = HookState.Out;
    RigState rState = RigState.Bottom;
    LaunchState lState = LaunchState.PlaneIn;

    public void init() {
        //New BOT Up - 0.47, Down - 0.7
//        motorBackRight = hardwareMap.dcMotor.get("motor8");
//        motorFrontRight = hardwareMap.dcMotor.get("motor7");
//        motorBackLeft = hardwareMap.dcMotor.get("motor2");
//        motorFrontLeft = hardwareMap.dcMotor.get("motor3");
        motorIntake = hardwareMap.dcMotor.get("motor5");
        motorLauncher = hardwareMap.dcMotor.get("motor4");
        temp = hardwareMap.dcMotor.get("motor7");
        motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motor1");
        motorSlideRight = hardwareMap.get(DcMotorEx.class, "motor6");
        servoClamp = hardwareMap.servo.get("servo1");
        servoHOT = hardwareMap.servo.get("servo5"); //Hook ot
        servoFOT = hardwareMap.servo.get("servo4"); // flap ot
        servoTOT = hardwareMap.servo.get("servo2"); // top ot
        servoBOT = hardwareMap.servo.get("servo3"); // bottom ot
        servoFL = hardwareMap.servo.get("servo6");
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSlideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
//                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

        // Initialize SampleMecanumDrive
        drive = new SampleMecanumDrive(hardwareMap);

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details

        startPose = new Pose2d(0, 0, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        liftTimer.reset();
        hookTimer.reset();
    }

    public void start() {
        servoClamp.setPosition(0.5);
        motorSlideLeft.setTargetPosition(78);
        motorSlideRight.setTargetPosition(78);
        servoTOT.setPosition(0.83);
        servoBOT.setPosition(0.7);
        servoFOT.setPosition(0.51);
        servoHOT.setPosition(0.52);
//        imu.resetYaw();
    }

    public void loop() {
        switch (lState) {
            case PlaneIn:
                if (launchTimer.milliseconds() > 1500) {
                    motorLauncher.setPower(0);
                    servoFL.setPosition(0.69);
                    if (gamepad2.back) {
                        servoFL.setPosition(0.39);
                        launchTimer.reset();
                        lState = LaunchState.PlaneSent;
                    }
                }
                break;
            case PlaneSent:
                if (launchTimer.milliseconds()>350) {
                    motorLauncher.setPower(1);
                    launchTimer.reset();
                    lState = LaunchState.PlaneIn;
                }
                break;
        }
        telemetry.addData("lState:",lState);
        switch (rState) {
            case Bottom:
                if (gamepad1.start) {
                    motorSlideRight.setTargetPosition(2560);
                    motorSlideLeft.setTargetPosition(2560);
                    motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorSlideRight.setVelocity(1500);
                    motorSlideLeft.setVelocity(1500);
                    position = 1500;
                    prevposition = position;
                    rigTimer.reset();
                    rState = RigState.Extend;
                }
                break;
            case Extend:
                if (rigTimer.milliseconds()>1500) {
//                    motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                    motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    if (gamepad1.start) {
//                        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        motorSlideRight.setTargetPosition(50);
                        motorSlideLeft.setTargetPosition(50);
                        motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorSlideRight.setVelocity(1500);
                        motorSlideLeft.setVelocity(1500);
                        position = 50;
                        prevposition = position;
                        rigTimer.reset();
                        rState = RigState.Rigged;
                    }
                }
                break;
            case Rigged:
                if (rigTimer.milliseconds()>500) {
                    if (gamepad1.start) {
                        motorSlideRight.setTargetPosition(1500);
                        motorSlideLeft.setTargetPosition(1500);
                        motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorSlideRight.setVelocity(1500);
                        motorSlideLeft.setVelocity(1500);
                        position = 1500;
                        prevposition = position;
                        rigTimer.reset();
                        rState = RigState.Extend;
                    }
                }
        }

        switch (hState) {
            case Out:
                if (hookTimer.milliseconds() > 300) {
                    if (gamepad2.right_bumper && armState == ArmState.Bottom) {
                        servoHOT.setPosition(0.67);
                        hookTimer.reset();
                        hState = HookState.In;
                    }
                    break;
                }
            case In:
                if (hookTimer.milliseconds() > 300) {
                    if (gamepad2.right_bumper && armState == ArmState.Bottom) {
                        servoHOT.setPosition(0.52);
                        hookTimer.reset();
                        hState = HookState.Out;
                    }
                    break;
                }
//            default:
//                // should never be reached, as armState should never be null
//                hState = HookState.Out;
        }
        telemetry.addData("hState:", hState);


        switch (armState) {
            case Bottom:
                if (gamepad2.left_bumper) {
                    if (motorSlideRight.getCurrentPosition() < 700) {
                        servoHOT.setPosition(0.67);
                        hState = HookState.In;
                        motorSlideRight.setTargetPosition(1000);
                        motorSlideLeft.setTargetPosition(1000);
                        motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        motorSlideRight.setVelocity(1000);
                        motorSlideLeft.setVelocity(1000);
                        position = 500;
                        prevposition = position;
                    }
                    liftTimer.reset();
                    armState = ArmState.RotateUp;
                } else if (gamepad1.left_bumper) {
                    servoBOT.setPosition(0.67);
                } else {
                    if (liftTimer.milliseconds()>1000) {
                        servoBOT.setPosition(0.695);
                    }
                }
                break;
            case RotateUp:
                if (motorSlideRight.getCurrentPosition()>480) {
                    servoTOT.setPosition(0.54);
                    servoBOT.setPosition(0.47);
                    liftTimer.reset();
                    armState = ArmState.Drop;
                }
                break;
            case Drop:
                if (liftTimer.seconds() >= 1) {
                    if (gamepad2.dpad_down) {
                        servoFOT.setPosition(0.66);
                        liftTimer.reset();
                        armState = ArmState.Drop2;
                    } else if (gamepad2.dpad_up) {
                        servoFOT.setPosition(0.66);
                        servoHOT.setPosition(0.52);
                        hState = HookState.Out;
                    } else if (gamepad2.left_bumper) {
                        servoFOT.setPosition(0.51);
                        servoTOT.setPosition(0.83);
                        servoBOT.setPosition(0.68);
                        liftTimer.reset();
                        armState = ArmState.DownCheck;
                    }
                }
                break;
            case Drop2:
                if (liftTimer.milliseconds() > 350) {
                    if (gamepad2.dpad_down) {
                        servoHOT.setPosition(0.52);
                        hState = HookState.Out;
                    } else if (gamepad2.left_bumper) {
                        servoFOT.setPosition(0.51);
                        servoTOT.setPosition(0.83);
                        servoBOT.setPosition(0.68);
                        liftTimer.reset();
                        armState = ArmState.DownCheck;
                    }
                }
                break;
            case DownCheck:
                if (motorSlideRight.getCurrentPosition()<900) {
                    motorSlideRight.setTargetPosition(950);
                    motorSlideLeft.setTargetPosition(950);
                    motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorSlideRight.setVelocity(2500);
                    motorSlideLeft.setVelocity(2500);
                    position = 950;
                    prevposition = position;
                    liftTimer.reset();
                    armState = ArmState.RotateDown;
                } else {
                    armState = ArmState.RotateDown;
                }
            case RotateDown:
                if (liftTimer.milliseconds()>1000) {
                    motorSlideRight.setTargetPosition(55);
                    motorSlideLeft.setTargetPosition(55);
                    motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorSlideRight.setVelocity(1000);
                    motorSlideLeft.setVelocity(1000);
                    position = 55;
                    prevposition = position;
                    liftTimer.reset();
                    armState = ArmState.Bottom;
                }
                break;
            default:
                // should never be reached, as armState should never be null
                armState = ArmState.Bottom;
        }
        telemetry.addData("ArmState:", armState);



        if (gamepad1.right_trigger > 0) {
            y = gamepad1.left_stick_y; // Remember, this is reversed!
            x = gamepad1.left_stick_x; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;
        } else if (gamepad1.left_trigger > 0) {
            y = 0.25 * gamepad1.left_stick_y; // Remember, this is reversed!
            x = 0.25 * gamepad1.left_stick_x; // Counteract imperfect strafing
            rx = 0.35 * gamepad1.right_stick_x;
        } else {
            y = 0.5 * gamepad1.left_stick_y; // Remember, this is reversed!
            x = 0.5 * gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = 0.65 * gamepad1.right_stick_x;
        }

//        if (gamepad1.back) {
//            startPose = new Pose2d(0, 0, Math.toRadians(90));
//            drive.setPoseEstimate(startPose);
////            imu = hardwareMap.get(IMU.class, "imu");
////            // Adjust the orientation parameters to match your robot
////            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
////                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
////                    RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
////            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
////            imu.initialize(parameters);
////            imu.resetYaw();
//        }
        if (gamepad1.dpad_right) {
            startPose = new Pose2d(0, 0, Math.toRadians(270));
            drive.setPoseEstimate(startPose);
        } else if (gamepad1.dpad_left) {
            startPose = new Pose2d(0, 0, Math.toRadians(90));
            drive.setPoseEstimate(startPose);
        } else if (gamepad1.dpad_up) {
            startPose = new Pose2d(0, 0, Math.toRadians(0));
            drive.setPoseEstimate(startPose);
        }

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading

        Vector2d input = new Vector2d(
                -y,
                -x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rx
                )
        );

        // Update everything. Odometry. Etc.
        drive.update();

        // Print pose to telemetry
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

//        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)-Math.toRadians(90);
//
//        // Rotate the movement direction counter to the bot's rotation
//        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//        rotX = rotX * 1.1;  // Counteract imperfect strafing
//
//        // Denominator is the largest motor power (absolute value) or 1
//        // This ensures all the powers maintain the same ratio,
//        // but only if at least one is out of the range [-1, 1]
//        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//        double frontLeftPower = (rotY + rotX + rx) / denominator;
//        double backLeftPower = (rotY - rotX + rx) / denominator;
//        double frontRightPower = (rotY - rotX - rx) / denominator;
//        double backRightPower = (rotY + rotX - rx) / denominator;
//
//        motorFrontLeft.setPower(frontLeftPower);
//        motorBackLeft.setPower(backLeftPower);
//        motorFrontRight.setPower(frontRightPower);
//        motorBackRight.setPower(backRightPower);


//            telemetry.addData("odometer middle pos", motorBackRight.getCurrentPosition());
//            telemetry.addData("odometer right pos", motorFrontRight.getCurrentPosition());
//            telemetry.addData("odometer left pos", motorBackLeft.getCurrentPosition());
//            telemetry.update();

        //Viper Slide Preset
        if (gamepad2.x) {
            speed=4000;
            position = 1000;
            armState = ArmState.RotateUp;
        }
        if (gamepad2.y) {
            speed=4000;
            position = 1750;
            armState = ArmState.RotateUp;

        }
        if (gamepad2.b) {
            speed=4000;
            position = 2250;
            armState = ArmState.RotateUp;
        }
        if (gamepad2.a) {
            speed=4000;
            position = 55;
            armState = ArmState.RotateUp;
        }

        if (gamepad2.left_stick_y != 0) {
            motorSlideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorSlideRight.setVelocity(-signum(gamepad2.left_stick_y)*1900);
            motorSlideLeft.setVelocity(-signum(gamepad2.left_stick_y)*2000);
            position = motorSlideLeft.getCurrentPosition();
            prevposition = position;
            a = true;
        } else if (a) {
            motorSlideRight.setVelocity(0);
            motorSlideLeft.setVelocity(0);
            motorSlideRight.setTargetPosition(motorSlideLeft.getCurrentPosition());
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            position = motorSlideLeft.getCurrentPosition();
            prevposition = position;
            a = false;
        }
        if (prevposition != position && gamepad2.left_stick_y == 0) {
            motorSlideRight.setTargetPosition(position);
            motorSlideLeft.setTargetPosition(position);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(speed);
            motorSlideLeft.setVelocity(speed);
            prevposition=position;
        }
//        telemetry.addData("heading", Math.toDegrees(botHeading));
        telemetry.addData("position", position);
        telemetry.addData("right", motorSlideRight.getCurrentPosition());
        telemetry.addData("positionReal", motorSlideRight.getTargetPosition());
        telemetry.addData("lefd", motorSlideLeft.getCurrentPosition());
        telemetry.addData("leftOdometry", temp.getCurrentPosition());
        telemetry.addData("rightOdometry", temp.getCurrentPosition());
        telemetry.addData("midOdometry", temp.getCurrentPosition());

        if (gamepad1.left_bumper) {
            motorIntake.setPower(1);
            servoHOT.setPosition(0.52);
            hState = HookState.Out;
        } else if (gamepad1.right_bumper) {
            motorIntake.setPower(-1);
        } else {
            motorIntake.setPower(0);
        }

        if (gamepad2.back) {
            motorLauncher.setPower(1);
        } else {
            motorLauncher.setPower(0);
        }
    }
}
