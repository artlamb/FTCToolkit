package utils;

import android.graphics.Point;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Locale;

import common.Logger;

@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Dashboard {

    public static boolean enabled = true;
    public static boolean drawFieldImage = true;

    private TelemetryPacket packet;
    Pose robotPose = new Pose(0, 0,0);
    ArrayList<Pose>  waypoints = new ArrayList<>();

    double scale = 24 / 23.5;                               // tiles are 23.5 inches not 24

    int queueSize = 100;
    PoseQueue poseQueue = new PoseQueue(queueSize);
    double[] poseTrackerX = new double[queueSize];
    double[] poseTrackerY = new double[queueSize];

    public void setPose(Pose pose) {

        if (! enabled) return;

        robotPose = pose;
        poseQueue.enqueue(pose);
    }

    public void addWaypoint(Pose waypoint) {

        if (! enabled) return;

        waypoints.add(waypoint);
    }

    public void drawField() {

        if (! enabled) return;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (! dashboard.isEnabled()) {
            Logger.warning("dashboard is not enabled");
            return;
        }

        TelemetryPacket packet = getPacket();
        Canvas canvas = packet.fieldOverlay();

        // Rotate the field image so the top right quadrant is positive x and y.
        canvas.setRotation(-Math.PI / 2);

        // Draw the image of the field
        if (drawFieldImage) {
            canvas.setAlpha(0.5);

            canvas.drawImage("/images/decode.webp", 72, -72, 144, 144, Math.PI, 0, 0, false);
            //canvas.drawImage("/images/into-the-deep.png", 72, 72, 144, 144, Math.PI / 2, 0, 0, false);
        }

        canvas.setAlpha(1.0);


        // draw the robot
        if (robotPose != null) {
            double heading = robotPose.getHeading(AngleUnit.RADIANS);
            double robotWidth = 17;
            double robotHeight = 17;
            double radius = Math.hypot(robotHeight/2, robotWidth/2);
            double angle = heading + (2 * Math.PI * (7./8.));
            double offsetX = (robotPose.getX(DistanceUnit.INCH) - Math.cos(angle) * radius) * scale;
            double offsetY = (robotPose.getY(DistanceUnit.INCH) - Math.sin(angle) * radius) * scale;

            canvas.drawImage("/images/robot.png", offsetX, offsetY, robotWidth, robotHeight, -heading, 0, 0, false);
        }

        drawPosesTrace(canvas);

        drawWaypoints(canvas);

        canvas.drawGrid(0, 0, 144, 144, 7, 7);

        sendPacket();
    }

    public static Pose calculateCoordinate(Pose origin, double offsetX, double offsetY) {

        double angleRadians = origin.getHeading(AngleUnit.RADIANS);

        // Rotate the local offsets to get global offsets
        double rotatedOffsetX = offsetX * Math.cos(angleRadians) - offsetY * Math.sin(angleRadians);
        double rotatedOffsetY = offsetX * Math.sin(angleRadians) + offsetY * Math.cos(angleRadians);

        // Add global offsets to the origin
        double x = origin.getX(DistanceUnit.INCH) + rotatedOffsetX;
        double y = origin.getY(DistanceUnit.INCH) + rotatedOffsetY;

        return new Pose(x, y, 0);
    }

    private void drawWaypoints(Canvas canvas) {
        canvas.setStrokeWidth(1);
        canvas.setFill("red");
        canvas.setStroke("red");

        for (Pose waypoint : waypoints) {
            double x = waypoint.getX(DistanceUnit.INCH) * scale;
            double y = waypoint.getY(DistanceUnit.INCH) * scale;
            canvas.fillCircle(x, y, 1);

            Pose end = calculateCoordinate(waypoint, 2, 0);
            canvas.strokeLine(x, y, end.getX(DistanceUnit.INCH) * scale, end.getY(DistanceUnit.INCH) * scale);

            double waypointX = waypoint.getX(DistanceUnit.INCH);
            double waypointY = waypoint.getY(DistanceUnit.INCH);
            String xString;
            String yString;

            if (Math.abs(waypointX) % 1.0 == 0.0)
                xString= String.format(Locale.US, "%.0f", waypointX);
            else
                xString= String.format(Locale.US, "%.1f", waypointX);
            if (Math.abs(waypointY) % 1.0 == 0.0)
                yString= String.format(Locale.US, "%.0f", waypointY);
            else
                yString= String.format(Locale.US, "%.1f", waypointY);
            String position = String.format(Locale.US, "(%s,%s)", xString, yString);
            canvas.fillText(position, x+1.5, y-1, "3px Arial", 0, false);
        }
    }

    private void drawPosesTrace(Canvas canvas) {

        Pose pose = new Pose();
        for (int i = 0; i < poseQueue.size(); i++) {
            pose = poseQueue.peek(i);
            poseTrackerX[i] = pose.getX();
            poseTrackerY[i] = pose.getY();
        }
        for (int i = poseQueue.size(); i < queueSize; i++) {
            poseTrackerX[i] = pose.getX();
            poseTrackerY[i] = pose.getY();
        }

        canvas.setStroke("#4CAF50");
        canvas.strokePolyline(poseTrackerX, poseTrackerY);
    }

    private TelemetryPacket getPacket() {
        if (packet == null) packet = new TelemetryPacket(false);
        return packet;
    }

    private void sendPacket() {
        if (packet != null) {
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            packet = null;
        }
    }

}
