package Other;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.concurrent.TimeUnit;

@TeleOp(group = "ZTest")
public class servoTest extends LinearOpMode{
    public void runOpMode() throws InterruptedException {
        Servo servoBOT = hardwareMap.servo.get("servo6");
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            while (gamepad1.a) {
                servoBOT.setPosition(0.66);
                TimeUnit.MILLISECONDS.sleep(250);
            }
            while (gamepad1.b) {
                servoBOT.setPosition(0.51);
                TimeUnit.MILLISECONDS.sleep(250);
            }
            telemetry.addData("servo pos.", servoBOT.getPosition());
            telemetry.update();
        }
    }
}