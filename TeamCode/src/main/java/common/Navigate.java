package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

import utils.Dashboard;
import utils.PathSegment;
import utils.Pose;

/**
 * This class manages path creation and following. Paths consist
 * of and end post or an end pose and one or more waypoints(poses).
 * The start of a path is the robot's current pose.
 */
public class Navigate {

    private static class Path {
        String          name;
        ArrayList<PathSegment> segment;
    }
    private final ArrayList<Path> paths = new ArrayList<>();

    private final LinearOpMode opMode;
    private final DriveControl driveControl;

    public Navigate(LinearOpMode opMode, DriveControl driveControl) {

        this.opMode = opMode;
        this.driveControl = driveControl;
        driveControl.resetIMU();
    }

    public void setStartingPose(int index) {
        Path path = paths.get(index);
        setStartingPose(path.segment.get(0).pose);
    }

    public void setStartingPose(Pose pose) {
        driveControl.setPose(pose);
        Logger.message("Start (%.0f, %.0f) heading: %.0f",
                pose.getX(), pose.getY(), pose.getHeading(AngleUnit.DEGREES));
    }

    public void followPath(int index) {

        Path path = paths.get(index);
        if (path == null) {
            Logger.warning("path for %d missing", index);
            return;
        }

        if (path.segment.get(0) == null) {
            Logger.warning("pose for %d missing", index);
        }

        followPath(path);
    }

    /**
     * Follow the path with the specified name.
     * @param name name of the path
     *
     * @noinspection unused
     */
    public void followPath(String name) {
        for (Path path : paths) {
            if (path.name.equals(name)) {
                followPath(path);
            }
        }
    }

    private void followPath(Path path) {

        for (PathSegment segment: path.segment) {
            Pose pose = segment.pose;
            double x = pose.getX();
            double y = pose.getY();
            double heading = pose.getHeading(AngleUnit.DEGREES);
            Logger.message("path %s (%.1f, %.1f) heading: %.0f", path.name, x, y, heading);
        }
        driveControl.followPath(path.segment, 5000);
    }

    /**
     * Display the current pose of the robot
     */
    public void displayPose () {
        Pose pose = driveControl.getPose();
        opMode.telemetry.addData("pose", "x %5.1f  y %5.1f  heading %5.1f  is busy %b",
                pose.getX(),
                pose.getY(),
                pose.getHeading(AngleUnit.DEGREES),
                driveControl.isBusy());
        opMode.telemetry.update();
    }

    /**
     * Add a path with an ending pose and zero or more waypoints (poses)
     *
     * @param name name of the pose
     * @param speed speed of the path
     * @param tolerance tolerance of the path's end pose
     * @param poses one or more poses
     */
    public void addPath(String name, double speed, double tolerance, Pose... poses) {
        Path path = new Path();
        path.name = name;
        path.segment = new ArrayList<>();
        for (Pose pose: poses) {
            path.segment.add(new PathSegment(pose, speed, tolerance));
        }
        paths.add(path);
    }

    /**
     * Append a pose to an existing path.
     *
     * @param name name of the path
     * @param pose pose to append
     */
    public void appendPose(String name, double speed, double tolerance, Pose pose) {
        for (Path path : paths) {
            if (path.name.equals(name)) {
                path.segment.add(new PathSegment(pose, speed, tolerance));
                return;
            }
        }
        Logger.warning("path not found");
    }

    /**
     * Set the poses of an existing path.
     * @param name - name of the path
     * @param poses -
     */
    public void setPath(String name, double speed, Pose... poses) {
        for (Path path : paths) {
            if (path.name.equals(name)) {
                path.segment = new ArrayList<>();
                for (Pose pose: poses) {
                    path.segment.add(new PathSegment(pose, speed));
                }
                return;
            }
        }
        Logger.warning("path not found");
    }

    /**
     * Check if a path with the specified name exist.
     *
     * @param name  name of the path
     * @return true if the path exist
     */
    public boolean pathExists(String name) {
        for (Path path : paths) {
            if (path.name.equals(name)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Return the number paths defined.
     * @return number of paths
     */
    public int numberOfPaths() {
        return paths.size();
    }

    /**
     * Return the name of the specified path.
     *
     * @param index of the path
     * @return name of the path
     */
    public String getPathName(int index) {
        return paths.get(index).name;
    }

    /**
     * Draw the paths on the FTC Dashboard field view.
     */
    public void drawPaths() {

        if (! Debug.drawPaths())
            return;

        Dashboard dashboard = new Dashboard();

        dashboard.drawField();

        for (Path path : paths) {
            for (PathSegment segment: path.segment) {
                Pose pose = segment.pose;
                dashboard.addWaypoint(pose);
                dashboard.setPose(pose);
                dashboard.drawField();
                opMode.sleep(500);
            }
        }
        dashboard.drawField();
    }

    public void printPaths() {
        for (Path path : paths) {
            for (PathSegment segment: path.segment) {
                Logger.message("path %-12s %s  speed: %5.2f  tolerance: %5.2f", path.name, segment.toString(), segment.speed, segment.tolerance);
            }
        }
    }
}

