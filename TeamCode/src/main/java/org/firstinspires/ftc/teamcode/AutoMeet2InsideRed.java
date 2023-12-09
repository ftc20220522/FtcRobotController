package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;
@Autonomous(name="AutoMeet2InsideRed")
public class AutoMeet2InsideRed extends LinearOpMode{
    private final int READ_PERIOD = 1;

    private HuskyLens huskyLens;


    String mode = "TAG";

    String pos;

    int location=1;

    HuskyLens.Block[] blocks;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo servoROT = hardwareMap.servo.get("servo2");
        servoROT.setDirection(Servo.Direction.REVERSE);
        Servo servoLOT = hardwareMap.servo.get("servo3");
        Servo servoTOT = hardwareMap.servo.get("servo5");
        servoTOT.setDirection(Servo.Direction.REVERSE);
        Servo servoBOT = hardwareMap.servo.get("servo4");
        DcMotorEx motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motor6");
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotorEx motorSlideRight= hardwareMap.get(DcMotorEx.class, "motor7");
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        TrajectorySequence toRight = drive.trajectorySequenceBuilder(startPose)
                .forward(20)
                .strafeRight(10)
                .build();
        TrajectorySequence toMid = drive.trajectorySequenceBuilder(startPose)
//                .strafeLeft(4.5)
                .forward(26)
                .build();
        TrajectorySequence toLeft = drive.trajectorySequenceBuilder(startPose)
//                .strafeLeft(4.5)
                .forward(26.5)
                .turn(Math.toRadians(90))
                .forward(3)
                .build();
        TrajectorySequence back = drive.trajectorySequenceBuilder(toMid.end())
                .back(18)
                .strafeRight(25)
                .build();
        TrajectorySequence back2 = drive.trajectorySequenceBuilder(toRight.end())
                .back(4)
                .strafeRight(16)
                .forward(25)
                .build();
        TrajectorySequence back3 = drive.trajectorySequenceBuilder(toLeft.end())
                .back(4)
                .strafeLeft(16)
                .back(20)
                .build();
        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);

        /*
         * Immediately expire so that the first time through we'll do the read.
         */
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         */
        waitForStart();
        servoTOT.setPosition(0.89);
        servoBOT.setPosition(0.9);
        servoROT.setPosition(0.1);
        long start = System.currentTimeMillis();
        long end = start + 2000;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        while (opModeIsActive() && System.currentTimeMillis() < end) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            telemetry.update();
            blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            if (blocks.length>0) {
                telemetry.addData("Block", blocks[0].toString());
                if (blocks[0].x <= 100) {
                    telemetry.addData("Pos:", "Left");
                    telemetry.update();
                    location=2;
                } else if (blocks[0].x > 100 && blocks[0].x <= 200) {
                    telemetry.addData("Pos:", "Middle");
                    telemetry.update();
                    location=3;
                } else if (blocks[0].x > 200) {
                    telemetry.addData("Pos:", "Right");
                    telemetry.update();
                }
                break;
            }
        }
        if (location == 2) {
            drive.followTrajectorySequence(toMid);
            motorSlideRight.setTargetPosition(420);
            motorSlideLeft.setTargetPosition(420);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(3400);
            motorSlideLeft.setVelocity(3400);
            sleep(1000);
            servoTOT.setPosition(0.876);
            servoBOT.setPosition(0.82);
            sleep(1000);
            servoROT.setPosition(0.8);
            sleep(500);
            drive.followTrajectorySequence(back);
            servoTOT.setPosition(0.89);
            servoBOT.setPosition(0.9);
            servoROT.setPosition(0.1);
            sleep(500);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(3400);
            motorSlideLeft.setVelocity(3400);
            sleep(2500);
        } else if (location==3){
            drive.followTrajectorySequence(toRight);
            motorSlideRight.setTargetPosition(420);
            motorSlideLeft.setTargetPosition(420);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(3400);
            motorSlideLeft.setVelocity(3400);
            sleep(1000);
            servoTOT.setPosition(0.876);
            servoBOT.setPosition(0.82);
            sleep(1000);
            servoROT.setPosition(0.8);
            sleep(500);
            drive.followTrajectorySequence(back2);
            servoTOT.setPosition(0.89);
            servoBOT.setPosition(0.9);
            servoROT.setPosition(0.1);
            sleep(500);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(3400);
            motorSlideLeft.setVelocity(3400);
            sleep(2500);
        } else if (location == 1) {
            drive.followTrajectorySequence(toLeft);
            motorSlideRight.setTargetPosition(420);
            motorSlideLeft.setTargetPosition(420);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(3400);
            motorSlideLeft.setVelocity(3400);
            sleep(1000);
            servoTOT.setPosition(0.876);
            servoBOT.setPosition(0.82);
            sleep(1000);
            servoROT.setPosition(0.8);
            sleep(500);
            drive.followTrajectorySequence(back3);
            servoTOT.setPosition(0.89);
            servoBOT.setPosition(0.9);
            servoROT.setPosition(0.1);
            sleep(500);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(3400);
            motorSlideLeft.setVelocity(3400);
            sleep(2500);
        }
    }

}
