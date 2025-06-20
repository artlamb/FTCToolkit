package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import common.Logger;
import common.Robot;
import utils.Dashboard;
import utils.Pose;

@Disabled
@TeleOp(name="Dashboard Test", group="Test")
@SuppressLint("DefaultLocale")

public class DashboardTest extends LinearOpMode {

    Robot robot;
    Dashboard dashboard;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            dashboard = new Dashboard();

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            //drawOnDashboard(new Pose2D(DistanceUnit.INCH,-(24), 23.5, AngleUnit.RADIANS, Math.PI));

            dashboard.setPose(new Pose(2, 0.75, 0, 0, Math.PI/2));
            dashboard.addWaypoint(new Pose(0, 0, 0, 0, 0));
            dashboard.addWaypoint(new Pose(1, 0, 0, 0, 0));
            dashboard.addWaypoint(new Pose(-2, -2, -10, -10, 0));

            dashboard.drawField();

            waitForStart();

            while (opModeIsActive()) {
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void drawOnDashboard(Pose2D pose) {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (! dashboard.isEnabled()) {
            Logger.warning("dashboard is not enabled");
            return;
        }

        double heading = pose.getHeading(AngleUnit.RADIANS);
        double robotWidth = 18;
        double robotLength = 18;
        double radius = Math.hypot(robotLength/2, robotWidth/2);
        double angle = heading + (2 * Math.PI * (7./8.));
        double offsetX = pose.getX(DistanceUnit.INCH) - Math.cos(angle) * radius;
        double offsetY = pose.getY(DistanceUnit.INCH) - Math.sin(angle) * radius;

        Logger.message("offset  x: %f  y: %f", offsetX, offsetY);

        TelemetryPacket packet = new TelemetryPacket(false);
        packet.fieldOverlay()
                //.setScale(0.75, 0.75)
                //.setTranslation(10, 10)


                .setRotation(-Math.PI/2)

                .setAlpha(0.5)
                .drawImage("/images/into-the-deep.png", 72, 72, 144, 144, Math.PI/2, 0, 0 , false)

                .drawGrid(0, 0, 144, 144, 7, 7)

                .setAlpha(1.0)
                .drawImage("/images/test.png", offsetX, offsetY, robotLength, robotWidth, -heading, 0, 0, false)


                .setStrokeWidth(1)
                .setStroke("green")
                //.strokeCircle(24, 24, 1)
                .setFill("red")
                .fillCircle(24, 24, 1)

                .strokeLine(0, 0, 0, 24) //y axis
                //.setStroke("red")
                //.strokeLine(0, 0, 24, 0)//x axis
                ;


        dashboard.sendTelemetryPacket(packet);
        sleep(20);
    }
}
