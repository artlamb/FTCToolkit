package test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="ServoTest", group="Test")
//@Disabled
@SuppressWarnings({"FieldCanBeLocal", "unused"})

public class ServoTest extends LinearOpMode {

    private CRServo continuousServo;
    private boolean aButtonReleased = true;
    private boolean servoOn = false;

    @Override
    public void runOpMode() {

        continuousServo = hardwareMap.get(CRServo.class, "servo");

        telemetry.addData("Status", "Continuous Servo, Press start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && aButtonReleased) {
                servoOn = !servoOn;
                aButtonReleased = false;
            } else if (!gamepad1.a) {
                aButtonReleased = true;
            }

            continuousServo.setPower(servoOn ? -1.0 : 0.0);

            telemetry.addData("Servo State", servoOn ? "Running" : "Stopped");
            telemetry.update();
            sleep(10);
        }
    }
}

