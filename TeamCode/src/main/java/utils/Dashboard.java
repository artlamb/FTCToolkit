package utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

import common.Logger;

@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Dashboard {

    public static boolean enabled = true;
    public static boolean drawFieldImage = false;

    private TelemetryPacket packet;
    Pose robotPose = new Pose(0, 0,0);
    ArrayList<Pose>  waypoints = new ArrayList<>();

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
            canvas.drawImage("/images/into-the-deep.png", 72, 72, 144, 144, Math.PI / 2, 0, 0, false);
        }

        double scale = 24 / 23.5;                           // tiles are 23.5 inches not 24
        canvas.setAlpha(1.0);

        // draw the robot
        if (robotPose != null) {
            double heading = robotPose.getHeading(AngleUnit.RADIANS);
            double robotWidth = 18;
            double robotLength = 18;
            double radius = Math.hypot(robotLength/2, robotWidth/2);
            double angle = heading + (2 * Math.PI * (7./8.));
            double offsetX = (robotPose.getX(DistanceUnit.INCH) - Math.cos(angle) * radius) * scale;
            double offsetY = (robotPose.getY(DistanceUnit.INCH) - Math.sin(angle) * radius) * scale;

            canvas.drawImage("/images/robot.png", offsetX, offsetY, robotLength, robotWidth, -heading, 0, 0, false);
        }

        // Draw the waypoints
        canvas.setStrokeWidth(1);
        canvas.setFill("red");
        for (Pose waypoint : waypoints) {
            double x = waypoint.getX(DistanceUnit.INCH) * scale;
            double y = waypoint.getY(DistanceUnit.INCH) * scale;
            canvas.fillCircle(x, y, 0.9);
        }

        drawPosesTrace(canvas);

        canvas.drawGrid(0, 0, 144, 144, 7, 7);

        sendPacket();
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
