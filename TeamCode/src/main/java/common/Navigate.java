package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Collections;

import utils.Dashboard;
import utils.Pose;

/**
 * This class manages path creation and following. Paths consist
 * of and end post or an end pose and one or more waypoints(poses).
 * The start of a path is the robot's current pose.
 */
public class Navigate {

    private static class Path {
        String          name;
        ArrayList<Pose> poses;
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
        setStartingPose(path.poses.get(0));
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

        if (path.poses.get(0) == null) {
            Logger.warning("pose for %d missing", index);
        }

        followPath(path);
    }

    public void followPath(String name) {
        for (Path path : paths) {
            if (path.name.equals(name)) {
                followPath(path);
            }
        }
    }

    private void followPath(Path path) {

        for (Pose pose: path.poses) {
            double x = pose.getX();
            double y = pose.getY();
            double heading = pose.getHeading(AngleUnit.DEGREES);
            Logger.message("path %s (%.1f, %.1f) heading: %.0f", path.name, x, y, heading);
        }
        if (path.poses.size() > 1)
            driveControl.followPath(path.poses, 4000);
        else
            driveControl.moveToPose(path.poses.get(0), 4000);
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
     * @param poses one or more poses
     */
    public void addPath(String name, Pose... poses) {
        Path path = new Path();
        path.name = name;
        path.poses = new ArrayList<>();
        Collections.addAll(path.poses, poses);
        paths.add(path);
    }

    /**
     * Append a pose to an existing path.
     *
     * @param name name of the path
     * @param pose pose to append
     */
    public void appendPose(String name, Pose pose) {
        for (Path path : paths) {
            if (path.name.equals(name)) {
                path.poses.add(pose);
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

    public int numberOfPaths() {
        return paths.size();
    }

    public void displayPaths() {
        Dashboard dashboard = new Dashboard();

        dashboard.drawField();

        for (Path path : paths) {
            for (Pose pose : path.poses) {
                dashboard.addWaypoint(pose);
                dashboard.setPose(pose);
                dashboard.drawField();
                opMode.sleep(1000);
            }
        }
        dashboard.drawField();
    }
}

