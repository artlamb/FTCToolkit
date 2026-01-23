package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import utils.Pose;
import utils.PoseData;

@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Auto {

    public static double MAX_SPEED = 0.60;
    public static double LOW_SPEED = 0.20;
    public static double MIN_TOLERANCE = 0;
    public static double LOW_TOLERANCE = 0.30;
    public static double HIGH_TOLERANCE = 0.60;

    public static PoseData START_AUDIENCE = new PoseData(12.25, -62,    90);
    public static PoseData START_GOAL =     new PoseData(50.50, 50.50,  45);
    public static PoseData SHOOT_GOAL =     new PoseData(23.50, 23.50,  45);
    public static PoseData SHOOT_AUDIENCE = new PoseData(11.50, 11.50,  45);
    public static PoseData ARTIFACT =       new PoseData(27.00, 11.75,  0);
    public static PoseData PARK_AUDIENCE =  new PoseData(12.25, -36,    0);
    public static PoseData PARK_GOAL =      new PoseData(23.50, -12,    0);

    public enum Alliance {BLUE, RED}

    public Alliance alliance;

    public enum StartPosition {GOAL, AUDIENCE}

    public StartPosition startPosition;

    public enum Order {TOP, MIDDLE, BOTTOM}

    public Order[] order;

    public enum PathState {
        START, SHOOT, ARTIFACT_1, ARTIFACT_2, ARTIFACT_3, PARK, UNKNOWN;

        public static PathState get(int index) {
            return values()[index];
        }
    }

    private int pathIndex = 0;

    private double launcherSpeed = 28;
    private long delay = 0;

    private final ElapsedTime elapsedTime = new ElapsedTime();
    private final ElapsedTime timer = new ElapsedTime();
    boolean running;

    private final LinearOpMode opMode;
    private final Robot robot;
    private final DriveControl driveControl;
    private final Navigate navigate;
    private final Launcher launcher;
    private final Limelight limelight;

    public Auto(LinearOpMode opMode) {

        this.opMode = opMode;
        robot = new Robot(opMode);

        launcher = robot.getLauncher();

        driveControl = robot.getDriveControl();
        driveControl.reset();
        navigate = new Navigate(opMode, driveControl);

        limelight = robot.getLimelight();
        limelight.setPipeline(Limelight.Pipeline.LOCATION);
    }

    public Auto(LinearOpMode opMode,
                Alliance alliance,
                StartPosition startPosition,
                Order[] order,
                double launcherSpeed,
                long delay) {

        this(opMode);
        this.alliance = alliance;
        this.startPosition = startPosition;
        this.order = order;
        this.launcherSpeed = launcherSpeed;
        this.delay = delay;

        generatePaths();
        drawPaths();
        navigate.printPaths();

        setLauncherSpeed(launcherSpeed);
        navigate.setStartingPose(pathIndex);
    }

    public void runAuto() {

        opMode.telemetry.addData("Alliance", alliance);
        opMode.telemetry.addData("Start Position", startPosition);
        opMode.telemetry.addData("Launcher Speed", launcherSpeed);
        opMode.telemetry.addData("Delay", delay);
        for (Order value : order) {
            opMode.telemetry.addData("Order", value);
        }
        opMode.telemetry.addLine("Press start");
        opMode.telemetry.update();

        opMode.waitForStart();

        if (delay > 0)
            opMode.sleep(delay * 1000);

        running = true;
        elapsedTime.reset();
        while (running && opMode.opModeIsActive()) {

            PathState state = getPathState(navigate.getPathName(pathIndex));
            Logger.info("%s starts,  time: %6.2f", state, elapsedTime.seconds());

            switch (state) {
                case START:
                    followPath();
                    break;

                case SHOOT:
                    waitUntilNearPose();
                    robot.powerLauncher(true, launcherSpeed);
                    waitUntilNotMoving();
                    robot.fireAllArtifacts();
                    waitUntilRobotIdIdle();
                    setOdometer();
                    followPath();
                    break;

                case ARTIFACT_1:
                case ARTIFACT_2:
                case ARTIFACT_3:
                    robot.powerIntake(true);
                    waitUntilNotMoving();
                    followPath();
                    robot.delay(1000);
                    robot.powerIntake(false);
                    robot.loadArtifact();
                    break;

                case PARK:
                    waitUntilNotMoving();
                    running = false;
                    break;
            }
            Logger.info("%s ends,  time: %6.2f\n", state, elapsedTime.seconds());
        }
        Logger.info("opmode elapsed time %4.1f", elapsedTime.seconds());
    }

    private void generatePaths() {

        double heading;

        // The coordinates are the same for both alliances except the x coordinate are negative for the blue alliance.
        double xSign = (alliance == Alliance.RED) ? 1 : -1;

        // Create the pose for a starting position at the goal, near the obelisk or by the audience.
        Pose start = null;
        if (startPosition == StartPosition.GOAL) {
            heading = (alliance == Alliance.RED) ? START_GOAL.h : START_GOAL.h + 90;
            start = createPose(START_GOAL.x * xSign, START_GOAL.y, heading);
        } else if (startPosition == StartPosition.AUDIENCE) {
            start = createPose(START_AUDIENCE.x * xSign, START_AUDIENCE.y, START_AUDIENCE.h);
        }
        navigate.addPath(getPathName(PathState.START), MAX_SPEED, MIN_TOLERANCE, start);

        // Create a path to the shooting position.
        Pose shoot = null;
        heading = (alliance == Alliance.RED) ? SHOOT_GOAL.h : SHOOT_GOAL.h + 90;
        if (startPosition == StartPosition.GOAL) {
            shoot = createPose(SHOOT_GOAL.x * xSign, SHOOT_GOAL.y, heading);
        } else if (startPosition == StartPosition.AUDIENCE) {
            shoot = createPose(SHOOT_AUDIENCE.x * xSign, SHOOT_AUDIENCE.y, heading);
        }
        navigate.addPath(getPathName(PathState.SHOOT), MAX_SPEED, MIN_TOLERANCE, shoot);

        // Create a path for to pickup each group of artifact on the order specified and a path back to the shooting position.
        for (Order o : order) {
            heading = (alliance == Alliance.RED) ? 180 : 0;
            double tileSize = 23.5;
            double x1 = ARTIFACT.x;
            double y1 = ARTIFACT.y;
            double x2 = (tileSize * 2);
            double y2 = y1;
            PathState pathState;
            switch (o) {
                case TOP:
                    pathState = PathState.ARTIFACT_1;
                    break;
                case MIDDLE:
                    pathState = PathState.ARTIFACT_2;
                    y2 -= tileSize;
                    break;
                case BOTTOM:
                    pathState = PathState.ARTIFACT_3;
                    y2 -= tileSize * 2;
                    break;
                default:
                    return;
            }
            Pose artifactStart = createPose(x1 * xSign, y2, heading);
            Pose artifactEnd = createPose(x2 * xSign, y2, heading);
            navigate.addPath(getPathName(pathState), MAX_SPEED, MIN_TOLERANCE, artifactStart);
            navigate.appendPose(pathState.name(), LOW_SPEED, LOW_TOLERANCE, artifactEnd);
            if (startPosition == StartPosition.AUDIENCE) {
                navigate.appendPose(pathState.name(), MAX_SPEED, LOW_TOLERANCE, artifactStart);
            }
            navigate.addPath(getPathName(PathState.SHOOT), MAX_SPEED, MIN_TOLERANCE, shoot);
        }

        // Create a path to the parking position.
        Pose park;
        if (startPosition == StartPosition.GOAL) {
            park = createPose(PARK_GOAL.x * xSign, PARK_GOAL.y, PARK_GOAL.h);
        } else {
            park = createPose(PARK_AUDIENCE.x * xSign, PARK_AUDIENCE.y, PARK_AUDIENCE.h);
        }
        navigate.addPath(getPathName(PathState.PARK), MAX_SPEED, HIGH_TOLERANCE, park);
    }

    private Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, Math.toRadians(heading));
    }

    public void drawPaths() {
        if (Debug.drawPaths()) {
            navigate.drawPaths();
        }
    }

    public void addPath(PathState pathState, double x, double y, double heading) {
        navigate.addPath(getPathName(pathState), MAX_SPEED, 0, createPose(x, y, heading));
    }

    public void setStartPose(double x, double y, double heading) {
        navigate.setStartingPose(createPose(x, y, heading));
    }

    private String getPathName(PathState pathState) {
        return String.format("%s", pathState);
    }

    private PathState getPathState(String pathName) {
        for (PathState state : PathState.values()) {
            if (state.name().equals(pathName)) {
                return state;
            }
        }
        return PathState.UNKNOWN;
    }

    private void followPath() {
        waitForButtonPress();
        waitUntilOkToMove();
        pathIndex++;
        navigate.followPath(pathIndex);
    }

    public void setLauncherSpeed(double speed) {
        launcher.setSpeed(speed);
    }

    public void waitUntilNotMoving() {
        Logger.debug("waiting");
        long start = System.currentTimeMillis();
        driveControl.waitUntilNotMoving(5000);
        Logger.debug("done waiting, time: %5d", System.currentTimeMillis() - start);

    }

    private void waitUntilRobotIdIdle() {
        Logger.debug("waiting");
        long timeout = 6000;
        timer.reset();
        while (robot.isBusy() && opMode.opModeIsActive()) {
            if (timer.milliseconds() > timeout) {
                Logger.warning("robot timed out after %d ms", timeout);
                break;
            }
            Thread.yield();
        }
        Logger.debug("done waiting, time: %5.0f", timer.seconds());
    }

    private void waitUntilOkToMove() {
        robot.waitUntilOkToMove();
    }

    private void waitUntilNearPose() {
        Logger.debug("waiting");
        long timeout = 5000;
        timer.reset();
        while (opMode.opModeIsActive()) {
            if (driveControl.nearPose() ) {
                break;
            } else {
                Thread.yield();
            }
            if (timer.milliseconds() > timeout) {
                Logger.warning("robot timed out");
                break;
            }
        }
        Logger.debug("done waiting, time: %5.0f", timer.milliseconds());
    }

    private void waitForButtonPress() {

        if (!Debug.waitForButtonPress()) return;

        Logger.debug("waiting for button press");

        boolean pressed = false;
        while (opMode.opModeIsActive()) {
            navigate.displayPose();

            if (pressed) {
                if (!opMode.gamepad1.x) {
                    break;
                }
            } else if (opMode.gamepad1.x) {
                pressed = true;
            }
        }
        Logger.debug("done waiting for button press");
    }

    private void setOdometer() {

        int blueID = 20;
        int redID = 24;
        Pose current;

        int id = limelight.GetAprilTagID();
        if ((id == blueID && alliance == Alliance.BLUE) || (id == redID) && alliance == Alliance.RED) {
            Logger.debug("april tag found with id: %d", id);
            current = limelight.getPosition();
            if (current != null) {
                Logger.debug("odometer's position set from: %s to: %s", driveControl.getPose(), current);
                // todo should we do this? driveControl.setPose(current);
            }
        }
    }
}
