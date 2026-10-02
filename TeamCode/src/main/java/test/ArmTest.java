package test;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import common.ArmKinematics;

@TeleOp(name="IK Arm Test", group="Test")
@SuppressLint("DefaultLocale")
@Disabled

public class ArmTest extends LinearOpMode {

    // Arm lengths in inches or cm
    ArmKinematics kinematics = new ArmKinematics(15.0, 12.0);

    Servo shoulderServo;
    Servo elbowServo;

    @Override
    public void runOpMode() {
        shoulderServo = hardwareMap.get(Servo.class, "shoulder");
        elbowServo = hardwareMap.get(Servo.class, "elbow");

        waitForStart();

        while (opModeIsActive()) {
            // Use joysticks to set a target (x, y)
            double targetX = 10.0 + (gamepad1.left_stick_x * 5);
            double targetY = 10.0 - (gamepad1.left_stick_y * 5);

            ArmKinematics.Positions positions = kinematics.reverseKinematics(targetX, targetY);

            if (positions.isValid) {
                shoulderServo.setPosition(positions.shoulderServo);
                elbowServo.setPosition(positions.elbowServo);
            } else {
                telemetry.addLine("TARGET UNREACHABLE");
            }

            telemetry.addData("X", targetX);
            telemetry.addData("Y", targetY);

            double sPos = shoulderServo.getPosition();
            double ePos = elbowServo.getPosition();

            ArmKinematics.Point currentPos = kinematics.forwardKinematics(sPos, ePos);

            telemetry.addData("Current X", "%.2f inches", currentPos.x);
            telemetry.addData("Current Y", "%.2f inches", currentPos.y);

            telemetry.update();
        }
    }
}