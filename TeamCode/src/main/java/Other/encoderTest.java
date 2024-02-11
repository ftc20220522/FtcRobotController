package Other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group="ZTest")
@Disabled
public class encoderTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        DcMotor testEncoder = hardwareMap.dcMotor.get("motor4");
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
         telemetry.addData("odometer pos", testEncoder.getCurrentPosition());
         telemetry.update();
        }
    }
}
