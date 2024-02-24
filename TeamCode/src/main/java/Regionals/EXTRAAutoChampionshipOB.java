package Regionals;

import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;

@Autonomous(name="EXTRAAutoOB")
public class EXTRAAutoChampionshipOB extends LinearOpMode {
    TrajectoryVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
            new TranslationalVelocityConstraint(25),
            new AngularVelocityConstraint(2)
    ));

    TrajectoryAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(25);

    private final int READ_PERIOD = 1;
    int location = 0;
    double distance;
    float hsvFlapValues[] = {0F, 0F, 0F};
    final float valuesF[] = hsvFlapValues;
    boolean flapLightOn = false;
    float hsvHookValues[] = {0F, 0F, 0F};
    final float valuesH[] = hsvHookValues;
    boolean hookLightOn = false;
    final double SCALE_FACTOR = 8;
    int relativeLayoutId;
    boolean bPrevState = false;
    boolean bCurrState = false;
    boolean bLedOn = true;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor8");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor7");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor2");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorIntake = hardwareMap.dcMotor.get("motor5");
        DcMotor motorLauncher = hardwareMap.dcMotor.get("motor4");
        DcMotorEx motorSlideLeft = hardwareMap.get(DcMotorEx.class, "motor1");
        motorSlideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        DcMotorEx motorSlideRight = hardwareMap.get(DcMotorEx.class, "motor6");
        motorSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlideRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DistanceSensor distanceOuttake = hardwareMap.get(DistanceSensor.class, "distanceOT");
        DistanceSensor distanceIntake = hardwareMap.get(DistanceSensor.class, "distanceIT");
//
//        relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//
        ColorSensor colorFlap = hardwareMap.get(ColorSensor.class, "colorFlap");
        ColorSensor colorHook = hardwareMap.get(ColorSensor.class, "colorHook");
//        colorFlap.enableLed(bLedOn);
//        colorHook.enableLed(bLedOn);
//        RevBlinkinLedDriver lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        Servo servoClamp = hardwareMap.servo.get("servo1"); //Purple Pixel Clamp
        Servo servoHOT = hardwareMap.servo.get("servo5"); //Hook ot
        Servo servoFOT = hardwareMap.servo.get("servo4"); // flap ot
        Servo servoTOT = hardwareMap.servo.get("servo2"); // top ot
        Servo servoBOT = hardwareMap.servo.get("servo3"); // bottom ot
        Servo servoFL = hardwareMap.servo.get("servo6"); //Flight Launcher
        Servo servoWhite = hardwareMap.servo.get("servo7"); //Intake

        HuskyLens huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Pose2d startPose = new Pose2d(-38, 61, Math.toRadians(90));
        drive.setPoseEstimate(startPose);


        servoClamp.setPosition(0.6);
        servoTOT.setPosition(0.83);
        servoBOT.setPosition(0.69);
        servoFOT.setPosition(0.53);
        servoHOT.setPosition(0.67);
        servoFL.setPosition(0.69);
        servoWhite.setPosition(0.37);  //Down - 0.75   //Top of 5 pixel stack - 0.55

        motorSlideRight.setTargetPosition(60);
        motorSlideLeft.setTargetPosition(60);
        motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlideRight.setVelocity(1500);
        motorSlideLeft.setVelocity(1500);

        //Left Movement
        TrajectorySequence purpleL = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-38,31))
                .lineToConstantHeading(new Vector2d(-32,31))
                .build();
        TrajectorySequence moveL = drive.trajectorySequenceBuilder(purpleL.end())
                .lineToConstantHeading(new Vector2d(-38,31))
                .lineToLinearHeading(new Pose2d(-45,9,0))
                .build();
        TrajectorySequence whiteL = drive.trajectorySequenceBuilder(moveL.end())
                .addDisplacementMarker(() -> {
                    servoHOT.setPosition(0.52);
                    distance = distanceIntake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(-58.25+distance-3.5,7.5))
                .build();
        TrajectorySequence whiteL2 = drive.trajectorySequenceBuilder(whiteL.end())
                .lineToConstantHeading(new Vector2d(-58.25+distance+8,7.5))
                .addDisplacementMarker(() -> {
                    servoWhite.setPosition(0.37);
                    motorIntake.setPower(1);
                })
                .setConstraints(velConstraint, accelConstraint)
                .lineToConstantHeading(new Vector2d(-58.25+distance-1,7.5))
                .resetConstraints()
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-48, 7.5))
                .build();
        TrajectorySequence backL = drive.trajectorySequenceBuilder(whiteL2.end())
                .lineToConstantHeading(new Vector2d(38, 13))
                .build();
        //Board Pixel
        TrajectorySequence yellowL = drive.trajectorySequenceBuilder(backL.end())
                .lineToConstantHeading(new Vector2d(50,38))
                .addDisplacementMarker(() -> {
                    distance = distanceOuttake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(50+distance+3, 38))
                .build();
        TrajectorySequence endL = drive.trajectorySequenceBuilder((yellowL.end()))
                .lineToConstantHeading(new Vector2d(45,38))
                .build();




        //Middle Movement
        TrajectorySequence purpleM = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-41,24))
                .build();
        TrajectorySequence moveM = drive.trajectorySequenceBuilder(purpleM.end())
                .lineToConstantHeading(new Vector2d(-50, 24))
                .lineToLinearHeading(new Pose2d(-50,9,0))
                .build();
        TrajectorySequence whiteM = drive.trajectorySequenceBuilder(moveM.end())
                .addDisplacementMarker(() -> {
                    servoHOT.setPosition(0.52);
                    distance = distanceIntake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(-58.25+distance-3.5,9.5))
                .build();
        TrajectorySequence whiteM2 = drive.trajectorySequenceBuilder(whiteM.end())
                .lineToConstantHeading(new Vector2d(-58.25+distance+8,9.5))
                .addDisplacementMarker(() -> {
                    servoWhite.setPosition(0.37);
                    motorIntake.setPower(1);
                })
                .setConstraints(velConstraint, accelConstraint)
                .lineToConstantHeading(new Vector2d(-58.25+distance-2,9.5))
                .resetConstraints()
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-48, 7.5))
                .build();
        TrajectorySequence backM = drive.trajectorySequenceBuilder(whiteM2.end())
                .lineToConstantHeading(new Vector2d(38, 13))
                .build();
        //Board Pixel
        TrajectorySequence yellowM = drive.trajectorySequenceBuilder(backM.end())
                .lineToConstantHeading(new Vector2d(50,32))
                .addDisplacementMarker(() -> {
                    distance = distanceOuttake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(50+distance+3.5, 32))
                .build();
        TrajectorySequence endM = drive.trajectorySequenceBuilder((yellowM.end()))
                .lineToConstantHeading(new Vector2d(45,32))
                .build();




        //Right Movement
        TrajectorySequence purpleR = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-46, 37, 0))

                .build();
        TrajectorySequence moveR = drive.trajectorySequenceBuilder(purpleR.end())
                .lineToConstantHeading(new Vector2d(-46,43))
                .lineToConstantHeading(new Vector2d(-36,43))
                .lineToConstantHeading(new Vector2d(-36,9))
                .build();
        TrajectorySequence whiteR = drive.trajectorySequenceBuilder(moveR.end())
                .addDisplacementMarker(() -> {
                    servoHOT.setPosition(0.52);
                    distance = distanceIntake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(-58.25+distance-3,7.5))
                .build();
        TrajectorySequence whiteR2 = drive.trajectorySequenceBuilder(whiteR.end())
                .lineToConstantHeading(new Vector2d(-58.25+distance+8,7.5))
                .addDisplacementMarker(() -> {
                    servoWhite.setPosition(0.37);
                    motorIntake.setPower(1);
                })
                .setConstraints(velConstraint, accelConstraint)
                .lineToConstantHeading(new Vector2d(-58.25+distance-1,7.5))
                .resetConstraints()
                .waitSeconds(0.2)
                .lineToConstantHeading(new Vector2d(-48, 7.5))
                .build();
        TrajectorySequence backR = drive.trajectorySequenceBuilder(whiteR2.end())
                .lineToConstantHeading(new Vector2d(38, 13))
                .build();
        //Board Pixel
        TrajectorySequence yellowR = drive.trajectorySequenceBuilder(backR.end())
                .lineToConstantHeading(new Vector2d(50,33.25))
                .addDisplacementMarker(() -> {
                    distance = distanceOuttake.getDistance(DistanceUnit.INCH);
                })
                .lineToConstantHeading(new Vector2d(50+distance+3.5, 33.25))
                .build();
        TrajectorySequence endR = drive.trajectorySequenceBuilder((yellowR.end()))
                .lineToConstantHeading(new Vector2d(45,33.25))
                .build();


        //Husky Lens
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
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
                    location = 1;
                } else if (blocks[i].x > 100 && blocks[i].x <= 200) {
                    telemetry.addData("Pos:", "Middle");
                    telemetry.update();
                    location = 2;
                } else if (blocks[i].x > 200) {
                    telemetry.addData("Pos:", "Right");
                    telemetry.update();
                    location = 3;
                }
            }
            if (blocks.length == 0 && timer.milliseconds()>1500) {
                location = 3;
            }
            if (location != 0) {
                break;
            }
        }

        if (location == 1) {
            //Purple Pixel
            drive.followTrajectorySequence(purpleL);
            servoClamp.setPosition(0.1);
            sleep(250);

            //White Pixel
            drive.followTrajectorySequence(moveL);
            do {
                drive.followTrajectorySequence(whiteL);
                sleep(200);
                servoWhite.setPosition(0.58);
                sleep(400);
                drive.followTrajectorySequence(whiteL2);
                servoHOT.setPosition(0.67);
                motorIntake.setPower(-0.5);
                sleep(200);
                servoWhite.setPosition(0.37);
                sleep(200);
                motorIntake.setPower(0);
                sleep(200);
            } while (((DistanceSensor) colorFlap).getDistance(DistanceUnit.CM) > 2 && ((DistanceSensor) colorHook).getDistance(DistanceUnit.CM) > 2 && timer.seconds()>18);
            drive.followTrajectorySequence(backL);


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

            //Yellow Pixel
            drive.followTrajectorySequence(yellowL);

            if (((DistanceSensor) colorFlap).getDistance(DistanceUnit.CM) < 2 && ((DistanceSensor) colorHook).getDistance(DistanceUnit.CM) < 2) {
                sleep(50);
                servoFOT.setPosition(0.66);
                sleep(150);
                drive.followTrajectorySequence(endL);
                sleep(150);
                servoHOT.setPosition(0.52);
                sleep(250);
            } else {
                sleep(50);
                servoHOT.setPosition(0.52);
                sleep(100);
                servoFOT.setPosition(0.66);
                sleep(150);
                drive.followTrajectorySequence(endL);
            }

            //Park
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            servoTOT.setPosition(0.83);
            servoBOT.setPosition(0.69);
            servoFOT.setPosition(0.53);
            servoHOT.setPosition(0.52);
            sleep(1300);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(2000);
        } else if (location == 2) {
            //Purple Pixel
            drive.followTrajectorySequence(purpleM);
            sleep(250);
            servoClamp.setPosition(0.1);
            sleep(250);

            //White Pixel
            drive.followTrajectorySequence(moveM);
            do {
                drive.followTrajectorySequence(whiteM);
                sleep(200);
                servoWhite.setPosition(0.58);
                sleep(400);
                drive.followTrajectorySequence(whiteM2);
                servoHOT.setPosition(0.67);
                motorIntake.setPower(-0.5);
                sleep(200);
                servoWhite.setPosition(0.37);
                sleep(200);
                motorIntake.setPower(0);
                sleep(200);
            } while (((DistanceSensor) colorFlap).getDistance(DistanceUnit.CM) > 2 && ((DistanceSensor) colorHook).getDistance(DistanceUnit.CM) > 2 && timer.seconds()>18);
            drive.followTrajectorySequence(backM);


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

            //Yellow Pixel
            drive.followTrajectorySequence(yellowM);

            if (((DistanceSensor) colorFlap).getDistance(DistanceUnit.CM) < 2 && ((DistanceSensor) colorHook).getDistance(DistanceUnit.CM) < 2) {
                sleep(50);
                servoFOT.setPosition(0.66);
                sleep(150);
                drive.followTrajectorySequence(endM);
                sleep(150);
                servoHOT.setPosition(0.52);
                sleep(250);
            } else {
                sleep(50);
                servoHOT.setPosition(0.52);
                sleep(100);
                servoFOT.setPosition(0.66);
                sleep(150);
                drive.followTrajectorySequence(endM);
            }

            //Park
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            servoTOT.setPosition(0.83);
            servoBOT.setPosition(0.69);
            servoFOT.setPosition(0.53);
            servoHOT.setPosition(0.52);
            sleep(1300);
            motorSlideRight.setTargetPosition(0);
            motorSlideLeft.setTargetPosition(0);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            sleep(2000);


        } else if (location == 3) {
            //Purple Pixel
            drive.followTrajectorySequence(purpleR);
            sleep(250);
            servoClamp.setPosition(0.1);
            sleep(250);

            //White Pixel
            drive.followTrajectorySequence(moveR);
            do {
                drive.followTrajectorySequence(whiteR);
                sleep(200);
                servoWhite.setPosition(0.58);
                sleep(400);
                drive.followTrajectorySequence(whiteR2);
                servoHOT.setPosition(0.67);
                motorIntake.setPower(-0.5);
                sleep(200);
                servoWhite.setPosition(0.37);
                sleep(200);
                motorIntake.setPower(0);
                sleep(200);
            } while (((DistanceSensor) colorFlap).getDistance(DistanceUnit.CM) > 2 && ((DistanceSensor) colorHook).getDistance(DistanceUnit.CM) > 2 && timer.seconds()>18);
            drive.followTrajectorySequence(backR);


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

            //Yellow Pixel
            drive.followTrajectorySequence(yellowR);
            if (((DistanceSensor) colorFlap).getDistance(DistanceUnit.CM) < 2 && ((DistanceSensor) colorHook).getDistance(DistanceUnit.CM) < 2) {
                sleep(50);
                servoFOT.setPosition(0.66);
                sleep(300);
                drive.followTrajectorySequence(endR);
                sleep(150);
                servoHOT.setPosition(0.52);
                sleep(250);
            } else {
                sleep(50);
                servoFOT.setPosition(0.66);
                sleep(300);
                sleep(150);
                servoHOT.setPosition(0.52);
                sleep(500);
                drive.followTrajectorySequence(endR);
            }


            //Park
            motorSlideRight.setTargetPosition(1000);
            motorSlideLeft.setTargetPosition(1000);
            motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSlideRight.setVelocity(1000);
            motorSlideLeft.setVelocity(1000);
            servoTOT.setPosition(0.83);
            servoBOT.setPosition(0.69);
            servoFOT.setPosition(0.53);
            servoHOT.setPosition(0.52);
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

