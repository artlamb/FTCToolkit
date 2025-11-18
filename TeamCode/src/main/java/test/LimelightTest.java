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
import common.DriveControl;
import common.Limelight;
import common.Logger;
import common.Robot;
import utils.Pose;

@TeleOp(name="LimelightTest", group="Test")
@SuppressLint("DefaultLocale")

public class LimelightTest extends LinearOpMode {

    Robot robot;
    GoBildaPinpointDriver pinpoint;
    Limelight limelight;
    DriveControl driveControl;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);

            driveControl = robot.getDriveControl();

            limelight = robot.getLimelight();
            limelight.setPipeline(Limelight.Pipeline.LOCATION);

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

    private void lineUpWithGoal() {

        Pose current = limelight.getPosition();
        if (current == null) {
            Logger.message("no apriltag found");
            return;
        }

        // aim for the center of the goal
        int id = limelight.GetAprilTagID();
        Pose target;
        double num = 60;
        if (id == 20) {  // blue
            target = new Pose(-num, num, 135);
        } else if (id == 24) {     //red
            target = new Pose(num, num, 45);
        } else {
            Logger.message("apriltag if not found");
            return;
        }

        double a = target.getX() - current.getX();
        double b = target.getY() - current.getY();
        double angle = Math.atan2(b, a);
        double rotation = AngleUnit.normalizeRadians(current.getHeading() - angle);

        double distance = Math.hypot(a, b);

        Pose pose = driveControl.getPose();
        double heading =  AngleUnit.normalizeRadians(current.getHeading() + rotation);
        Pose newPose = new Pose(current.getX(), current.getY(), heading);
        //driveControl.moveToPose(newPose, 0.2, 1000);
        Logger.debug("current: %s  target: %s  pose: %s", current, target, newPose);

        telemetry.addData("April Tag:", "ID: %d  distance: %4.2", id, distance);
    }

    private void displayPose(boolean telemetryOnly) {

        Pose pose = limelight.getPosition();
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
