package Tournament;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="AutoTournamentTests")
@Disabled
public class AutoTournamentTests extends LinearOpMode {
    private final int READ_PERIOD = 2;
    private HuskyLens huskyLens;
    String mode = "TAG";
    String pos;
    int location = 0;
    double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    int DESIRED_TAG_ID = location;     // Choose the tag you want to approach or set to -1 for ANY tag.
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
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

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  move           = 1;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 1;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 1;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process
        initAprilTag();

        setManualExposure(6, 250);



        //Middle Movement
        TrajectorySequence startM = drive.trajectorySequenceBuilder(startPose)
                .back(29)
                .build();
        TrajectorySequence backM = drive.trajectorySequenceBuilder(startM.end())
                .forward(5)
                //Clockwise is positive (right)
                //Counterclockwise is negative (left)
                .build();
        TrajectorySequence moveM = drive.trajectorySequenceBuilder(backM.end())
                .forward(3)
                .strafeLeft(18)
                .back(33)
                .turn(Math.toRadians(-90))
                .forward(106)
                .build();
        TrajectorySequence leftM = drive.trajectorySequenceBuilder(moveM.end())
                .strafeLeft(28)
                .forward(3)
                .build();
        TrajectorySequence backwardM = drive.trajectorySequenceBuilder(leftM.end())
                .back(8)
                .strafeRight(15)
                .build();


        //Right Movement
        TrajectorySequence startR = drive.trajectorySequenceBuilder(startPose)
                .back(26)
                .strafeLeft(11)
                .build();
        TrajectorySequence backR = drive.trajectorySequenceBuilder(startR.end())
                .forward(10)
                .build();
        TrajectorySequence moveR = drive.trajectorySequenceBuilder(backR.end())
                .strafeRight(16)
                .back(38)
                .turn(Math.toRadians(-90))
                .forward(83)
                .build();
        TrajectorySequence leftR = drive.trajectorySequenceBuilder(moveR.end())
                .strafeLeft(25)
                .forward(3)
                .build();
        TrajectorySequence backwardR = drive.trajectorySequenceBuilder(leftR.end())
                .back(8)
                .strafeRight(15)
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
                location = 2;
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

                //Viper Slide Up
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

                //Move to Board & Drop
                drive.followTrajectorySequence(leftM);

                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if (detection.id == DESIRED_TAG_ID) {
                            // Yes, we want to use this tag.
                            targetFound = true;
                            desiredTag = detection;
                        }
                    } else {
                        //add code to run override trajectory sequence
                    }
                }

                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                move  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", move, strafe, turn);
                moveRobot(move, strafe, turn);
                sleep(100);
                servoFOT.setPosition(0.72);
                sleep(200);

                //Move Away and Viper Slide Down
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
//                motorIntake.setPower(-0.45);
                drive.followTrajectorySequence(backR);
//                motorIntake.setPower(0);
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
                motorSlideRight.setTargetPosition(100);
                motorSlideLeft.setTargetPosition(100);
                motorSlideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorSlideRight.setVelocity(1000);
                motorSlideLeft.setVelocity(1000);
                sleep(3400);
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

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}

