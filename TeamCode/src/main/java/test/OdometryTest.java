package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import common.Config;
import common.DriveControl;
import common.Logger;
import common.Robot;
import drivers.GoBildaPinpointDriver;
import utils.Pose;

@TeleOp(name="OdometryTest", group="Test")
@SuppressLint("DefaultLocale")

public class OdometryTest extends LinearOpMode {
    public boolean telemetryOnly = true;

    Robot robot;
    GoBildaPinpointDriver pinpoint;
    DriveControl driveControl;
    Pose lastPose;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            driveControl = robot.getDriveControl();
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Config.PINPOINT);
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS, 0));

            lastPose = driveControl.getPose();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            angleTest(10,  0);
            angleTest(10,  10);
            angleTest(0,   10);
            angleTest(-10, 10);
            angleTest(-10, 0);
            angleTest(-10, -10);
            angleTest(0,   -10);
            angleTest(10,  -10);


            while (opModeIsActive()) {
                displayPose();
                moveTest();
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void moveTest() {
        Pose pose = driveControl.getPose();

        if (Math.abs(pose.getX() - lastPose.getX()) > 0.001 ||
            Math.abs(pose.getY() - lastPose.getY()) > 0.001 ||
            Math.abs(pose.getHeading() - lastPose.getHeading()) > 0.001) {
            driveControl.poseTest(new Pose(20, 0, 0));
            lastPose = pose;
        }
    }

    void angleTest(double x, double y) {
        Logger.message("x: %5.0f  y: %5.0f  arc tan2(y, x): %5.0f ", x, y, Math.toDegrees(Math.atan2(y, x)));
    }

    private void displayPose() {

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        Pose pose2 = driveControl.getPose();

        String str1 = String.format("x %5.1f  y %5.1f  heading %5.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        String str2 = String.format("x %5.1f  y %5.1f  heading %5.1f",
                pose2.getX(DistanceUnit.INCH),
                pose2.getY(DistanceUnit.INCH),
                pose2.getHeading(AngleUnit.DEGREES));
        String str3 = String.format("x %d  y %d", pinpoint.getEncoderX(), pinpoint.getEncoderY());

        telemetry.addData("pinpoint pose", str1);
        telemetry.addData("localizer pose", str2);
        telemetry.addData("encoders", str3);
        telemetry.update();

        if (! telemetryOnly) {
            Logger.message("pose: %s  localizer: %s  encoders %s", str1, str2, str3);
        }
    }

}
