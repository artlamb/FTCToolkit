package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import common.Config;
import common.Limelight;
import common.Logger;
import common.Robot;

@TeleOp(name="LimelightTest", group="Test")
@SuppressLint("DefaultLocale")

public class LimelightTest extends LinearOpMode {

    Robot robot;
    GoBildaPinpointDriver pinpoint;
    Limelight limelight;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            limelight = new Limelight(this);
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Config.PINPOINT);
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS, 0));

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                displayPose(false);
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void displayPose(boolean telemetryOnly) {

        Pose2D pose = limelight.getPosition(0);
        if (pose == null) {
            Logger.message("no apriltag found");
            return;
        }

        String str1 = String.format("x %5.1f  y %5.1f  heading %5.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));

        telemetry.addData("field pose", str1);
        telemetry.update();

        if (! telemetryOnly) {
            Logger.message("pose: %s", str1);
        }
    }
}
