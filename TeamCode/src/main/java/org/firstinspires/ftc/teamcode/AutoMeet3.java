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

@Autonomous(name="AutoMeet3OutsideBlue")
public class AutoMeet3 extends LinearOpMode{
    private final int READ_PERIOD = 2;

    private HuskyLens huskyLens;


    String mode = "TAG";

    String pos;

    int location=2;

    HuskyLens.Block[] blocks;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor8");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor7");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor2");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorIntake = hardwareMap.dcMotor.get("motor4");
        DcMotor motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motor1");
        DcMotor motorSlideRight = hardwareMap.get(DcMotorEx.class, "motor6");
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
                .back(34)
                .build();
        TrajectorySequence backM = drive.trajectorySequenceBuilder(startM.end())
                .forward(10)
                //Clockwise is positive (left)
                //Counterclockwise is negative (right)
                .build();
        TrajectorySequence endM = drive.trajectorySequenceBuilder(backM.end())
                .forward(3)
                .strafeLeft(20)
                .back(31)
                .strafeRight(115)
                .forward(15)
                .build();



        //Right Movement
        TrajectorySequence startR = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .strafeLeft(11)
                .build();
        TrajectorySequence backR = drive.trajectorySequenceBuilder(startR.end())
                .forward(10)
                .build();
        TrajectorySequence endR = drive.trajectorySequenceBuilder(backR.end())
                .strafeRight(15)
                .back(35)
                .strafeRight(95)
                .forward(13.5)
                .build();



        //Left Movement
        TrajectorySequence startL = drive.trajectorySequenceBuilder(startPose)
                .back(28.5)
                //Positive turns counterclockwise
                .turn(Math.toRadians(90))
                .back(10)
                .build();
        TrajectorySequence backL = drive.trajectorySequenceBuilder(startL.end())
                .forward(9)
                .build();
        TrajectorySequence endL = drive.trajectorySequenceBuilder(backL.end())
                .forward(4)
                .strafeLeft(20)
                .back(50)
                .strafeRight(100)
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
        servoLauncher.setPosition(0.2);
        motorSlideLeft.setTargetPosition(0);
        motorSlideRight.setTargetPosition(0);
        servoTOT.setPosition(0.83);
        servoBOT.setPosition(0.23);
        servoFOT.setPosition(0.1);
        servoHOT.setPosition(0.57);
        long start = System.currentTimeMillis();
        long end = start + 2000;
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        while (opModeIsActive() && System.currentTimeMillis() < end) {

            rateLimit.reset();

            telemetry.update();
            blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            if (blocks.length>0) {
                telemetry.addData("Block", blocks[0].toString());
                if (blocks[0].x <= 100) {
                    telemetry.addData("Pos:", "Left");
                    telemetry.update();
                    location=3;
                } else if (blocks[0].x > 100 && blocks[0].x <= 200) {
                    telemetry.addData("Pos:", "Middle");
                    telemetry.update();
                    location=2;
                } else if (blocks[0].x > 200) {
                    telemetry.addData("Pos:", "Right");
                    telemetry.update();
                    location=1;
                }
                break;
            }
        }
        if (location == 2) {
            drive.followTrajectorySequence(startM);
            motorIntake.setPower(-0.38);
            drive.followTrajectorySequence(backM);
            motorIntake.setPower(0);
            drive.followTrajectorySequence(endM);
//
//
//            motorSlideRight.setTargetPosition(420);
//            motorSlideLeft.setTargetPosition(420);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(3400);
//            motorSlideLeft.setVelocity(3400);
//            sleep(1000);
//            servoTOT.setPosition(0.876);
//            servoBOT.setPosition(0.82);
//            sleep(1000);
//            servoROT.setPosition(0.8);
//            sleep(500);
//            drive.followTrajectorySequence(back);
//            servoTOT.setPosition(0.89);
//            servoBOT.setPosition(0.9);
//            servoROT.setPosition(0.1);
//            sleep(500);
//            motorSlideRight.setTargetPosition(0);
//            motorSlideLeft.setTargetPosition(0);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(3400);
//            motorSlideLeft.setVelocity(3400);
//            sleep(2500);
        } else if (location==1){
            drive.followTrajectorySequence(startR);
            motorIntake.setPower(-0.45);
            drive.followTrajectorySequence(backR);
            motorIntake.setPower(0);
            drive.followTrajectorySequence(endR);
//            motorSlideRight.setTargetPosition(420);
//            motorSlideLeft.setTargetPosition(420);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(3400);
//            motorSlideLeft.setVelocity(3400);
//            sleep(1000);
//            servoTOT.setPosition(0.876);
//            servoBOT.setPosition(0.82);
//            sleep(1000);
//            servoROT.setPosition(0.8);
//            sleep(500);
//            drive.followTrajectorySequence(back2);
//            servoTOT.setPosition(0.89);
//            servoBOT.setPosition(0.9);
//            servoROT.setPosition(0.1);
//            sleep(500);
//            motorSlideRight.setTargetPosition(0);
//            motorSlideLeft.setTargetPosition(0);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(3400);
//            motorSlideLeft.setVelocity(3400);
//            sleep(2500);
        } else if (location == 3) {
            drive.followTrajectorySequence(startL);
            motorIntake.setPower(-0.38);
            drive.followTrajectorySequence(backL);
            motorIntake.setPower(0);
            drive.followTrajectorySequence(endL);
//            motorSlideRight.setTargetPosition(420);
//            motorSlideLeft.setTargetPosition(420);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(3400);
//            motorSlideLeft.setVelocity(3400);
//            sleep(1000);
//            servoTOT.setPosition(0.876);
//            servoBOT.setPosition(0.82);
//            sleep(1000);
//            servoROT.setPosition(0.8);
//            sleep(500);
//            drive.followTrajectorySequence(back3);
//            servoTOT.setPosition(0.89);
//            servoBOT.setPosition(0.9);
//            servoROT.setPosition(0.1);
//            sleep(500);
//            motorSlideRight.setTargetPosition(0);
//            motorSlideLeft.setTargetPosition(0);
//            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorSlideRight.setVelocity(3400);
//            motorSlideLeft.setVelocity(3400);
//            sleep(2500);
        }
    }

}
