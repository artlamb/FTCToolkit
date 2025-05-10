package utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import common.Logger;

public class Dashboard {

    private static TelemetryPacket packet;

    public static void drawRobot(Pose2D pose) {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (! dashboard.isEnabled()) {
            Logger.warning("dashboard is not enabled");
            return;
        }

        TelemetryPacket packet = getPacket();

        double heading = pose.getHeading(AngleUnit.RADIANS);
        double robotWidth = 18;
        double robotLength = 18;
        double radius = Math.hypot(robotLength/2, robotWidth/2);
        double angle = heading + (2 * Math.PI * (7./8.));
        double scale = 24 / 23.5;                           // tiles are 23.5 inches not 24
        double offsetX = (pose.getX(DistanceUnit.INCH) - Math.cos(angle) * radius) * scale;
        double offsetY = (pose.getY(DistanceUnit.INCH) - Math.sin(angle) * radius) * scale;

        packet.fieldOverlay()

                .setRotation(-Math.PI/2)

                .setAlpha(0.5)
                .drawImage("/images/into-the-deep.png", 72, 72, 144, 144, Math.PI/2, 0, 0 , false)

                .setAlpha(1.0)
                .drawImage("/images/robot1.png", offsetX, offsetY, robotLength, robotWidth, -heading, 0, 0, false)

                .drawGrid(0, 0, 144, 144, 7, 7)
                ;

        dashboard.sendTelemetryPacket(packet);
    }

    private static TelemetryPacket getPacket() {
        if (packet == null) packet = new TelemetryPacket(false);
        return packet;
    }
}
