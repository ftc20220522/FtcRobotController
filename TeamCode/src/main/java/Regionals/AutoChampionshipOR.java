package Regionals;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2dKt;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.concurrent.TimeUnit;

@Autonomous(name="AutoOutsideRed")
public class AutoChampionshipOR extends LinearOpMode {
    private final int READ_PERIOD = 2;
    private HuskyLens huskyLens;
    String mode = "TAG";
    String pos;
    int location = 0;
    double distance;
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
        Servo servoClamp = hardwareMap.servo.get("servo1");
        Servo servoHOT = hardwareMap.servo.get("servo5"); //Hook ot
        Servo servoFOT = hardwareMap.servo.get("servo4"); // flap ot
        Servo servoTOT = hardwareMap.servo.get("servo2"); // top ot
        Servo servoBOT = hardwareMap.servo.get("servo3"); // bottom ot
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Pose2d startPose = new Pose2d(-38, -61, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        DistanceSensor distanceOuttake = hardwareMap.get(DistanceSensor.class, "distanceOT");
        DistanceSensor distanceIntake = hardwareMap.get(DistanceSensor.class, "distanceIT");

        ColorSensor colorFlap = hardwareMap.get(ColorSensor.class, "colorFlap");
        ColorSensor colorHook = hardwareMap.get(ColorSensor.class, "colorHook");

        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");


        servoClamp.setPosition(0.6);
        motorSlideLeft.setTargetPosition(0);
        motorSlideRight.setTargetPosition(0);
        servoTOT.setPosition(0.83);
        servoBOT.setPosition(0.69);
        servoFOT.setPosition(0.51);
        servoHOT.setPosition(0.67);

        //Left Movement
        TrajectorySequence purpleL = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-39,-31))
                .build();
        TrajectorySequence getToPosL = drive.trajectorySequenceBuilder(purpleL.end())
                .lineToConstantHeading(new Vector2d(-33,-31))
                .lineToLinearHeading(new Pose2d(-33,-10,0))
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence toBoardL = drive.trajectorySequenceBuilder(getToPosL.end())
                .lineToConstantHeading(new Vector2d(38,-10))
                .build();
        TrajectorySequence posL = drive.trajectorySequenceBuilder(toBoardL.end())
                .lineToConstantHeading(new Vector2d(50,-26))
                .addDisplacementMarker(() -> {
                    distance = distanceOuttake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(50+distance+3,-26))
                .build();
        TrajectorySequence endL = drive.trajectorySequenceBuilder(posL.end())
                .lineToConstantHeading(new Vector2d(46,-26))
                .build();



        //Middle Movement
        TrajectorySequence purpleM = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-34,-15,0))
                .build();
        TrajectorySequence getToPosM = drive.trajectorySequenceBuilder(purpleM.end())
                .lineToConstantHeading(new Vector2d(-44,-10))
                .build();
        TrajectorySequence toBoardM = drive.trajectorySequenceBuilder(getToPosM.end())
                .lineToConstantHeading(new Vector2d(38, -10))
                .build();
        TrajectorySequence posM = drive.trajectorySequenceBuilder(toBoardM.end())
                .lineToConstantHeading(new Vector2d(50,-36))
                .addDisplacementMarker(() -> {
                    distance = distanceOuttake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(50+distance+3, -36))
                .build();
        TrajectorySequence endM = drive.trajectorySequenceBuilder(posM.end())
                .lineToConstantHeading(new Vector2d(46,-36))
                .build();



        //Right Movement
        TrajectorySequence purpleR = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-38,-31))
                .turn(Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-32,-31))
                .build();
        TrajectorySequence getToPosR = drive.trajectorySequenceBuilder(purpleR.end())
                .lineToConstantHeading(new Vector2d(-40,-31))
                .lineToLinearHeading(new Pose2d(-40,-10,0))
                .build();
        TrajectorySequence toBoardR = drive.trajectorySequenceBuilder(getToPosR.end())
                .lineToConstantHeading(new Vector2d(38,-10))
                .build();
        TrajectorySequence posR = drive.trajectorySequenceBuilder(toBoardR.end())
                .lineToConstantHeading(new Vector2d(50,-43))
                .addDisplacementMarker(() -> {
                    distance = distanceOuttake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(50+distance+3, -43))
                .build();
        TrajectorySequence endR = drive.trajectorySequenceBuilder(posR.end())
                .lineToConstantHeading(new Vector2d(46,-43))
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
        ElapsedTime timer = new ElapsedTime();
        telemetry.update();
        waitForStart();
        timer.reset();

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
                    location = 4;
                } else if (blocks[i].x > 100 && blocks[i].x <= 200) {
                    telemetry.addData("Pos:", "Middle");
                    telemetry.update();
                    location = 5;
                } else if (blocks[i].x > 200) {
                    telemetry.addData("Pos:", "Right");
                    telemetry.update();
                    location = 6;
                }
            }
            if (blocks.length == 0 && timer.milliseconds()>1500) {
                location = 6;
            }
            if (location != 0) {
                break;
            }
        }
        if (location == 4) {
            drive.followTrajectorySequence(purpleL);
            servoClamp.setPosition(0.1);
            sleep(250);
            drive.followTrajectorySequence(getToPosL);

            //To Backboard
            drive.followTrajectorySequence(toBoardL);

            //Viper Slides Up & Set
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(250);
            servoTOT.setPosition(0.53);
            servoBOT.setPosition(0.37);
            sleep(700);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(5000);

            //Position to Board
            drive.followTrajectorySequence(posL);
            sleep(50);
            servoFOT.setPosition(0.71);
            servoHOT.setPosition(0.52);
            sleep(50);

            //End
            drive.followTrajectorySequence(endL);
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            servoTOT.setPosition(0.83);
            servoBOT.setPosition(0.69);
            servoFOT.setPosition(0.53);
            sleep(1300);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(2000);
        } else if (location == 5) {
            drive.followTrajectorySequence(purpleM);
            servoClamp.setPosition(0.1);
            sleep(250);
            drive.followTrajectorySequence(getToPosM);

            //To Backboard
            drive.followTrajectorySequence(toBoardM);

            //Viper Slides Up & Set
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(250);
            servoTOT.setPosition(0.53);
            servoBOT.setPosition(0.37);
            sleep(700);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(5000);

            //Position to Board
            drive.followTrajectorySequence(posM);
            sleep(50);
            servoFOT.setPosition(0.71);
            servoHOT.setPosition(0.52);
            sleep(50);

            //End
            drive.followTrajectorySequence(endM);
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            servoTOT.setPosition(0.83);
            servoBOT.setPosition(0.69);
            servoFOT.setPosition(0.53);
            sleep(1300);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(2000);
        } else if (location == 6) {
            drive.followTrajectorySequence(purpleR);
            servoClamp.setPosition(0.1);
            sleep(250);
            drive.followTrajectorySequence(getToPosR);

            //To Backboard
            drive.followTrajectorySequence(toBoardR);

            //Viper Slides Up & Set
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(250);
            servoTOT.setPosition(0.53);
            servoBOT.setPosition(0.37);
            sleep(700);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(5000);

            //Position to Board
            drive.followTrajectorySequence(posR);
            sleep(50);
            servoFOT.setPosition(0.71);
            servoHOT.setPosition(0.52);
            sleep(50);

            //End
            drive.followTrajectorySequence(endR);
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            servoTOT.setPosition(0.83);
            servoBOT.setPosition(0.69);
            servoFOT.setPosition(0.53);
            sleep(1300);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(2000);
        }
    }
}

