package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import utils.Pose;

@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Auto {

    public static boolean enableWait = false;

    public enum PathState {
        START, WAYPOINT_1, PARK;

        public static PathState next(int id) {
            return values()[id];
        }
    }
    private PathState pathState = PathState.START;

    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    boolean running;

    private final LinearOpMode opMode;
    private final Robot robot;
    private final DriveControl driveControl;
    private final Navigate navigate;

    public Auto(LinearOpMode opMode) {

        this.opMode = opMode;
        robot = new Robot(opMode);
        driveControl = robot.getDriveControl();
        driveControl.reset();
        navigate = new Navigate(opMode, driveControl);

        driveControl.MAX_SPEED = 0.40;  //todo remove
    }

    public void runAuto() {

        opMode.telemetry.addLine("Press start");
        opMode.telemetry.update();
        opMode.waitForStart();

        running = true;
        elapsedTime.reset();
        while (running && opMode.opModeIsActive()) {

            PathState currentState = pathState;
            Logger.info("%s starts,  time: %6.2f", currentState, elapsedTime.seconds());

            switch (pathState) {
                case START:
                    followPath();
                    break;

                case WAYPOINT_1:
                    waitForButtonPress();
                    waitUntilNotMoving();
                    //robot.lineUpTarget();
                    //waitUntilRobotIdIdle();
                    robot.fireAll();
                    waitUntilRobotIdIdle();
                    followPath();
                    break;

                case PARK:
                    waitUntilNotMoving();
                    running = false;
                    break;
            }
            Logger.info("%s ends,  time: %6.2f\n", currentState, elapsedTime.seconds());
        }
        Logger.message("opmode elapsed time %4.1f", elapsedTime.seconds());
    }

    public void addPath(PathState pathState, double x, double y, double heading) {
        navigate.addPath(getPathName(pathState), createPose(x, y, heading));
    }

    private String getPathName(PathState pathState) {
        return String.format("%s", pathState);
    }

    private void followPath() {

        waitUntilOkToMove();

        int index = pathState.ordinal() + 1;
        pathState  = PathState.next(index);
        navigate.followPath(getPathName(pathState));
    }

    public void waitUntilNotMoving () {
        Logger.message("waiting, driveControl is %b", driveControl.isBusy());
        if (!driveControl.isBusy() )
            Logger.warning("driveControl is not busy");
        timer.reset();
        while (driveControl.isBusy() &&  opMode.opModeIsActive()) {
            if (timer.milliseconds() > 3000) {
                Logger.warning("driveControl timed out");
                break;
            }
        }
        Logger.message("done waiting, time: %5.0f", timer.milliseconds());
    }

    private void waitUntilRobotIdIdle() {
        Logger.message("waiting");
        long timeout = 4000;
        timer.reset();
        while (robot.isBusy() && opMode.opModeIsActive()) {
            if (timer.milliseconds() > timeout) {
                Logger.warning("robot timed out after %d ms", timeout);
                break;
            }
            Thread.yield();
        }
        Logger.message("done waiting, time: %5.0f", timer.seconds());
    }

    private void waitUntilOkToMove() {
        Logger.message("waiting");
        timer.reset();
        while (!robot.okToMove() && opMode.opModeIsActive()) {
            if (timer.milliseconds() > 5000) {
                Logger.warning("robot timed out");
                break;
            }
            Thread.yield();
        }
        Logger.message("done waiting, time: %5.0f", timer.seconds());
    }

    private void waitForButtonPress() {

        if (! enableWait) return;

        Logger.message("waiting for button press");

        boolean pressed = false;
        while (opMode.opModeIsActive()) {
            navigate.displayPose();

            if (pressed) {
                if (! opMode.gamepad1.x) {
                    break;
                }
            } else if (opMode.gamepad1.x) {
                pressed = true;
            }
        }
        Logger.message("done waiting for button press");
    }

    private Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, Math.toRadians(heading));
    }

    public void setStartPose (double x, double y, double heading) {
        navigate.setStartingPose(createPose(x, y, heading));
    }
}

