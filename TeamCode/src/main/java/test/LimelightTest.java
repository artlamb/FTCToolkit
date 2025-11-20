package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import common.Config;
import common.DriveControl;
import common.Limelight;
import common.Logger;
import common.Robot;
import utils.Pose;

@TeleOp(name="LimelightTest", group="Test")
@SuppressLint("DefaultLocale")
@Disabled

public class LimelightTest extends LinearOpMode {

    Robot robot;
    Limelight limelight;
    DriveControl driveControl;

    LED redLED;
    LED greenLED;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);

            driveControl = robot.getDriveControl();
            driveControl.setPose(new Pose(0,0, Math.toRadians(0)));

            limelight = robot.getLimelight();
            limelight.setPipeline(Limelight.Pipeline.LOCATION);

            greenLED = hardwareMap.get(LED.class, Config.GREEN_LEFT_LED);
            redLED = hardwareMap.get(LED.class, Config.RED_LEFT_LED);
            aprilTagVisible(false);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                //displayPose(false);
                lineUpWithGoal();
                telemetry.update();
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void lineUpWithGoal() {

        // aim for the center of the goal
        int id = limelight.GetAprilTagID();
        Pose target;
        double num = 70.5 - 9;
        if (id == 20) {  // blue
            target = new Pose(-num, num, Math.toRadians(135));
        } else if (id == 24) {     //red
            target = new Pose(num, num, Math.toRadians(45));
        } else {
            aprilTagVisible(false);
            Logger.message("april tag if not found");
            return;
        }

        Pose current = limelight.getPosition();
        if (current == null) {
            aprilTagVisible(false);
            Logger.message("no april tag found");
            return;
        }

        aprilTagVisible(true);

        double a = target.getX() - current.getX();
        double b = target.getY() - current.getY();
        double angle = Math.atan2(b, a);
        double rotation = AngleUnit.normalizeRadians(current.getHeading() - angle);

        double distance = Math.hypot(a, b);

        Pose pose = driveControl.getPose();
        double heading =  AngleUnit.normalizeRadians(pose.getHeading() + rotation);
        Pose newPose = new Pose(pose.getX(), pose.getY(), heading);
        //driveControl.moveToPose(newPose, 0.2, 1000);

        Logger.debug("current: %s   target: %s   pose: %s   new: %s   angle: %5.1f  rotation: %5.1f  ID: %d  distance: %4.1f",
                current, target, pose, newPose, Math.toDegrees(angle), Math.toDegrees(rotation), id, distance);
        telemetry.addData("April Tag:", "ID: %d  rotation: %5.1f  distance: %4.1f", id, Math.toDegrees(rotation), distance);
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

        if (! telemetryOnly) {
            Logger.message("pose: %s", str1);
        }
    }

    private void aprilTagVisible(boolean visible) {
        if (visible) {
            greenLED.on();
            redLED.off();
        } else {
            greenLED.off();
            redLED.on();
        }
    }
    // y= x * tan\left(r\right)\ -\ \frac{gx^{2}}{2\left(v\cos\left(r\right)\right)^{2}}
}
