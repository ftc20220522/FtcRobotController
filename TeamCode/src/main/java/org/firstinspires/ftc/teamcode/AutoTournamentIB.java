package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;

@Autonomous(name="AutoTournamentIB")
public class AutoTournamentIB extends LinearOpMode {
    private final int READ_PERIOD = 2;

    private HuskyLens huskyLens;


    String mode = "TAG";

    String pos;

    int location = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor8");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor7");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor2");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorIntake = hardwareMap.dcMotor.get("motor4");
        DcMotorEx motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motor1");
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorEx motorSlideRight = hardwareMap.get(DcMotorEx.class, "motor6");
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor temp = hardwareMap.dcMotor.get("motor7");
        Servo servoLauncher = hardwareMap.servo.get("servo1");
        Servo servoHOT = hardwareMap.servo.get("servo5"); //Hook ot
        Servo servoFOT = hardwareMap.servo.get("servo4"); // flap ot
        Servo servoTOT = hardwareMap.servo.get("servo2"); // top ot
        Servo servoBOT = hardwareMap.servo.get("servo3"); // bottom ot
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Pose2d startPose = new Pose2d(0, 0, 180);
        drive.setPoseEstimate(startPose);

        //Middle Movement
        TrajectorySequence startM = drive.trajectorySequenceBuilder(startPose)
                .back(29)
                .build();
        TrajectorySequence backM = drive.trajectorySequenceBuilder(startM.end())
                .forward(7)
                //Clockwise is positive (right)
                //Counterclockwise is negative (left)
                .build();
        TrajectorySequence moveM = drive.trajectorySequenceBuilder(backM.end())
                .turn(Math.toRadians(-90))
                .forward(32)
                .strafeRight(5)
                .build();
        TrajectorySequence leftM = drive.trajectorySequenceBuilder(moveM.end())
                .forward(8)
                .build();
        TrajectorySequence backwardM = drive.trajectorySequenceBuilder(leftM.end())
                .back(8)
                .strafeLeft(28)
                .build();


        //Right Movement
        TrajectorySequence startR = drive.trajectorySequenceBuilder(startPose)
                .back(29.5)
                .turn(Math.toRadians(-90))
                .back(6)
                .build();
        TrajectorySequence backR = drive.trajectorySequenceBuilder(startR.end())
                .forward(6)
                .build();
        TrajectorySequence moveR = drive.trajectorySequenceBuilder(backR.end())
                .forward(35.5)
                .strafeRight(7)
                .build();
        TrajectorySequence leftR = drive.trajectorySequenceBuilder(moveR.end())
                .forward(3)
                .build();
        TrajectorySequence backwardR = drive.trajectorySequenceBuilder(leftR.end())
                .back(8)
                .strafeLeft(36)
                .build();


        //Left Movement
        TrajectorySequence startL = drive.trajectorySequenceBuilder(startPose)
                .back(28.5)
                //Negative turns counterclockwise
                .turn(Math.toRadians(90))
                .back(10)
                .build();
        TrajectorySequence backL = drive.trajectorySequenceBuilder(startL.end())
                .forward(9)
                .build();
        TrajectorySequence moveL = drive.trajectorySequenceBuilder(backL.end())
                .strafeLeft(28)
                .turn(Math.toRadians(-180))
                .forward(87)
                .build();
        TrajectorySequence leftL = drive.trajectorySequenceBuilder(moveL.end())
                .strafeLeft(40)
                .forward(3)
                .build();
        TrajectorySequence backwardL = drive.trajectorySequenceBuilder(leftL.end())
                .back(8)
                .strafeRight(15)
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
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        waitForStart();
        servoLauncher.setPosition(0.2);
        motorSlideLeft.setTargetPosition(0);
        motorSlideRight.setTargetPosition(0);
        servoTOT.setPosition(0.83);
        servoBOT.setPosition(0.23);
        servoFOT.setPosition(0.1);
        servoHOT.setPosition(0.67);
        long start = System.currentTimeMillis();
        long end = start + 2000;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            telemetry.update();
            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());
                if (blocks[i].x <= 100) {
                    telemetry.addData("Pos:", "Left");
                    telemetry.update();
                    location = 3;
                } else if (blocks[i].x > 100 && blocks[i].x <= 200) {
                    telemetry.addData("Pos:", "Middle");
                    telemetry.update();
                    location = 2;
                } else if (blocks[i].x > 200) {
                    telemetry.addData("Pos:", "Right");
                    telemetry.update();
                    location = 1;
                }
            }
            if (blocks.length == 0) {
                location = 1;
            }
            if (location != 0) {
                break;
            }
        }


            if (location == 2) {
                drive.followTrajectorySequence(startM);
                motorIntake.setPower(-0.45);
                drive.followTrajectorySequence(backM);
                motorIntake.setPower(0);
                drive.followTrajectorySequence(moveM);

                //END PART DO NOT CHANGE
                motorSlideRight.setTargetPosition(1000);
                motorSlideLeft.setTargetPosition(1000);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                sleep(250);
                servoTOT.setPosition(0.54);
                servoBOT.setPosition(0);
                sleep(700);
                motorSlideRight.setTargetPosition(25);
                motorSlideLeft.setTargetPosition(25);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                drive.followTrajectorySequence(leftM);
                sleep(300);
                servoFOT.setPosition(0.72);
                sleep(200);
                drive.followTrajectorySequence(backwardM);
                motorSlideRight.setTargetPosition(1000);
                motorSlideLeft.setTargetPosition(1000);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                servoTOT.setPosition(0.83);
                servoBOT.setPosition(0.23);
                servoFOT.setPosition(0.57);
                servoHOT.setPosition(0.52);
                sleep(2000);
                motorSlideRight.setTargetPosition(0);
                motorSlideLeft.setTargetPosition(0);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                sleep(1000);
            } else if (location == 1) {
                drive.followTrajectorySequence(startR);
                motorIntake.setPower(-0.45);
                drive.followTrajectorySequence(backR);
                motorIntake.setPower(0);
                drive.followTrajectorySequence(moveR);

                //END PART DO NOT CHANGE
                motorSlideRight.setTargetPosition(1000);
                motorSlideLeft.setTargetPosition(1000);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                sleep(250);
                servoTOT.setPosition(0.54);
                servoBOT.setPosition(0);
                sleep(1200);
                motorSlideRight.setTargetPosition(25);
                motorSlideLeft.setTargetPosition(25);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                drive.followTrajectorySequence(leftR);
                sleep(300);
                servoFOT.setPosition(0.72);
                sleep(200);
                drive.followTrajectorySequence(backwardR);
                motorSlideRight.setTargetPosition(1000);
                motorSlideLeft.setTargetPosition(1000);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                servoTOT.setPosition(0.83);
                servoBOT.setPosition(0.23);
                servoFOT.setPosition(0.57);
                servoHOT.setPosition(0.52);
                sleep(2000);
                motorSlideRight.setTargetPosition(0);
                motorSlideLeft.setTargetPosition(0);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                sleep(1000);
            } else if (location == 3) {
                drive.followTrajectorySequence(startL);
//                motorIntake.setPower(-0.45);
                drive.followTrajectorySequence(backL);
//                motorIntake.setPower(0);
                drive.followTrajectorySequence(moveL);

                //END PART DO NOT CHANGE
                motorSlideRight.setTargetPosition(1000);
                motorSlideLeft.setTargetPosition(1000);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                sleep(250);
                servoTOT.setPosition(0.54);
                servoBOT.setPosition(0);
                sleep(1200);
                motorSlideRight.setTargetPosition(100);
                motorSlideLeft.setTargetPosition(100);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                sleep(3400);
                drive.followTrajectorySequence(leftL);
                sleep(300);
                servoFOT.setPosition(0.72);
                sleep(200);
                drive.followTrajectorySequence(backwardL);
                motorSlideRight.setTargetPosition(1000);
                motorSlideLeft.setTargetPosition(1000);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                servoTOT.setPosition(0.83);
                servoBOT.setPosition(0.23);
                servoFOT.setPosition(0.57);
                servoHOT.setPosition(0.52);
                sleep(2000);
                motorSlideRight.setTargetPosition(0);
                motorSlideLeft.setTargetPosition(0);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                sleep(1000);
            }
        }

    }

