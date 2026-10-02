package test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import common.Turret;

@TeleOp(name="Turret Test")
@com.acmerobotics.dashboard.config.Config
@Disabled

public class TurretTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Turret turret = new Turret(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            // Example: Press A to go to 90 degrees, B to return to 0
            if (gamepad1.a) {
                turret.rotateToAngle(90, 0.25);
            } else if (gamepad1.b) {
                turret.rotateToAngle(0, 0.25);
            }

            telemetry.addData("Target Angle", "90 or 0");
            telemetry.addData("Current Angle", "%.2f", turret.getCurrentAngle());
            telemetry.update();
        }
    }
}