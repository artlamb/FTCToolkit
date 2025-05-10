package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Collections;

import utils.Pose;

/**
 * This class manages path creation and following. Paths consist
 * of and end post or an end pose and one or more waypoints(poses).
 * The start of a path is the robot's current pose.
 */
@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard
public class Navigate {

    private static class Path {
        String          name;
        ArrayList<Pose> poses;
    }
    private final ArrayList<Path> paths = new ArrayList<>();

    private final ElapsedTime timer = new ElapsedTime();
    private final LinearOpMode opMode;
    private final DriveControl navigator;

    public Navigate(LinearOpMode opMode, DriveControl navigator) {

        this.opMode = opMode;
        this.navigator = navigator;
        navigator.resetIMU();
    }

    public void setStartingPose(Pose pose) {
        navigator.setPose(pose);
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

        for (Pose pose: path.poses) {
            double x = pose.getX();
            double y = pose.getY();
            double heading = pose.getHeading(AngleUnit.DEGREES);
            Logger.message("path %s index %d  (%.1f, %.1f) heading: %.0f", path.name, index, x, y, heading);
        }

        navigator.followPath(path.poses, 4000);
    }

    /**
     * Wait until the robot stops moving
     */
    public void waitUntilNotMoving () {
        Logger.message("waiting, navigator is %b", navigator.isBusy());
        if (!navigator.isBusy() )
            Logger.warning("navigator is not busy");
        timer.reset();
        while (navigator.isBusy() &&  opMode.opModeIsActive()) {
            if (timer.milliseconds() > 3000) {
                Logger.warning("navigator timed out");
                break;
            }
        }
        Logger.message("done waiting, time: %5.0f", timer.milliseconds());
    }

    /**
     * Display the current pose of the robot
     */
    public void displayPose () {
        Pose pose = navigator.getPose();
        opMode.telemetry.addData("pose", "x %5.1f  y %5.1f  heading %5.1f  is busy %b",
                pose.getX(),
                pose.getY(),
                pose.getHeading(AngleUnit.DEGREES),
                navigator.isBusy());
        opMode.telemetry.update();
    }

    /**
     * Create a path with an ending pose and zero or more waypoints (poses)
     *
     * @param name name of the pose
     * @param index order of the path
     * @param poses one or more poses
     */
    public void createPath(String name, int index, Pose... poses) {
        Path path = new Path();
        path.name = name;
        path.poses = new ArrayList<>();
        Collections.addAll(path.poses, poses);

        paths.add(index, path);
    }
}

