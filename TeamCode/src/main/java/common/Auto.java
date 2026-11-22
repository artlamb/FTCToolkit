package common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import utils.Pose;
import utils.PoseData;

@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Auto {

    public static double MAX_SPEED = 0.80;
    public static double FAST_SPEED = 0.60;
    public static double SLOW_SPEED = 0.25;

    public static double MIN_TOLERANCE = 0;
    public static double LOW_TOLERANCE = 0.1;
    public static double HIGH_TOLERANCE = 0.20;

    public static PoseData START_HIVE_UP =   new PoseData(+12.60, +61.50, 270);
    public static PoseData START_HIVE_DOWN = new PoseData(+12.60, -61.50, 0);
    public static PoseData START_CONNOR =    new PoseData(+61.50, -61.50, 0);
    public static PoseData SHOOT_ONE =       new PoseData(+12.60, +61.50, 270);
    public static PoseData SHOOT_TWO =       new PoseData(+12.60, -61.50, 270);
    public static PoseData GARDEN =          new PoseData(+61.50, +61.50, 0);
    public static PoseData FLOWER =          new PoseData(+58.00, +23.50, 0);
    public static PoseData FLOWER_AUDIENCE = new PoseData(+23.50, -58.00, 270);
    public static PoseData CHANGE_SIDES =    new PoseData(+47, 47.00, 270);
    public static PoseData PARK =            new PoseData(+55.00, -20.00, 180);
    public static PoseData PARK_AUDIENCE =   new PoseData(+55.00, -50.00, 180);

    public enum Alliance {BLUE, RED}
    public Alliance alliance;

    public enum PathState {
        START_HIVE_UP, START_HIVE_DOWN, START_CONNOR, SHOOT_ONE, SHOOT_TWO, GARDEN, FLOWER, FLOWER_AUDIENCE, CHANGE_SIDES, PARK, PARK_AUDIENCE, UNKNOWN;
        public static PathState get(int index) {
            return values()[index];
        }
    }
    public PathState[] order;
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
    private final Limelight limelight;

    public Auto(LinearOpMode opMode) {

        this.opMode = opMode;
        robot = new Robot(opMode);

        driveControl = robot.getDriveControl();
        driveControl.reset();
        navigate = new Navigate(opMode, driveControl);
        limelight = robot.getLimelight();
        limelight.setPipeline(Limelight.Pipeline.APRIL_TAG);
    }

    public Auto(LinearOpMode opMode,
                Alliance alliance,
                PathState[] order,
                double launcherSpeed,
                long delay) {

        this(opMode);
        this.alliance = alliance;
        this.order = order;
        this.launcherSpeed = launcherSpeed;
        this.delay = delay;

        generatePaths();
        drawPaths();
        navigate.printPaths();

        robot.setLauncherSpeed(launcherSpeed);
        navigate.setStartingPose(0);
    }

    public void runAuto() {

        opMode.telemetry.addData("Alliance", alliance);
        opMode.telemetry.addData("Launcher Speed", launcherSpeed);
        opMode.telemetry.addData("Delay", delay);
        for (PathState value : order) {
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
                case START_HIVE_UP:
                case START_HIVE_DOWN:
                case START_CONNOR:
                    robot.powerLauncher(true, launcherSpeed);
                    waitUntilRobotIsIdle();
                    followPath();
                    break;

                case SHOOT_ONE:
                    waitUntilNotMoving();
                    robot.fireAll();
                    waitUntilRobotIsIdle();
                    followPath();
                    break;

                case SHOOT_TWO:
                    waitUntilNotMoving();
                    waitUntilHiveTip();
                    robot.fireAll();
                    waitUntilRobotIsIdle();
                    followPath();
                    break;

                case CHANGE_SIDES:
                    waitUntilRobotIsIdle();
                    waitUntilNotMoving();
                    followPath();
                    break;

                case GARDEN:
                case FLOWER:
                case FLOWER_AUDIENCE:
                    robot.powerIntake(true);
                    waitUntilNotMoving();
                    followPath();
                    robot.delay(1000);
                    robot.powerIntake(false);
                    break;

                case PARK:
                case PARK_AUDIENCE:
                    waitUntilNotMoving();
                    running = false;
                    break;
            }
            Logger.info("%s ends,  time: %6.2f\n", state, elapsedTime.seconds());
        }
        Logger.info("opmode elapsed time %4.1f", elapsedTime.seconds());
    }

    /**
     * Generates and constructs all autonomous navigation paths based on the configured {@link #order}.
     * <p>
     * Iterates through each {@link PathState} in the execution order sequence and calculates alliance-specific
     * coordinates and headings (flipping X/Y coordinate signs and adding 180° heading offsets for Red alliance).
     * Registers the generated target poses with the {@link Navigate} subsystem.
     */
    private void generatePaths() {

        Pose pose;
        Pose startPose;
        Pose endPose;
        String pathName;

        // Create a path for each state of the robot during autonomous.
        for (PathState stateOrder : order) {
            double tileSize = 23.5;
            switch (stateOrder) {
                case START_HIVE_UP:
                    pose = createPose(START_HIVE_UP.x, START_HIVE_UP.y, START_HIVE_UP.h);
                    navigate.addPath(getPathName(Auto.PathState.START_HIVE_UP), FAST_SPEED, MIN_TOLERANCE, pose);
                    break;

                case START_HIVE_DOWN:
                    pose = createPose(START_HIVE_DOWN.x, START_HIVE_DOWN.y, START_HIVE_DOWN.h);
                    navigate.addPath(getPathName(Auto.PathState.START_HIVE_DOWN), FAST_SPEED, MIN_TOLERANCE, pose);
                    break;

                case START_CONNOR:
                    pose = createPose(START_CONNOR.x, START_CONNOR.y, START_CONNOR.h);
                    navigate.addPath(getPathName(Auto.PathState.START_CONNOR), FAST_SPEED, MIN_TOLERANCE, pose);
                    break;

                case SHOOT_ONE:
                    pose = createPose(SHOOT_ONE.x, SHOOT_ONE.y, SHOOT_ONE.h);
                    navigate.addPath(getPathName(Auto.PathState.SHOOT_ONE), FAST_SPEED, MIN_TOLERANCE, pose);
                    break;

                case SHOOT_TWO:
                    Pose shootAudience = createPose(SHOOT_TWO.x, SHOOT_TWO.y, SHOOT_TWO.h);
                    navigate.addPath(getPathName(Auto.PathState.SHOOT_TWO), FAST_SPEED, MIN_TOLERANCE, shootAudience);
                    break;

                case GARDEN:
                    pathName = getPathName(Auto.PathState.GARDEN);
                    startPose = createPose(GARDEN.x, GARDEN.y, GARDEN.h);
                    endPose = createPose((GARDEN.x + (tileSize / 2)), GARDEN.y, GARDEN.h);
                    navigate.addPath(pathName, MAX_SPEED, LOW_TOLERANCE, startPose);
                    navigate.appendPose(pathName, SLOW_SPEED, LOW_TOLERANCE, endPose);
                    break;

                case FLOWER:
                    pose = createPose(FLOWER.x, FLOWER.y, FLOWER.h);
                    navigate.addPath(getPathName(Auto.PathState.FLOWER), MAX_SPEED, MIN_TOLERANCE, pose);
                    break;

                case FLOWER_AUDIENCE:
                    pose = createPose(FLOWER_AUDIENCE.x, FLOWER_AUDIENCE.y, FLOWER_AUDIENCE.h);
                    navigate.addPath(getPathName(Auto.PathState.FLOWER_AUDIENCE), MAX_SPEED, MIN_TOLERANCE, pose);
                    break;

                case CHANGE_SIDES:
                    pathName = getPathName(Auto.PathState.CHANGE_SIDES);
                    startPose = createPose(CHANGE_SIDES.x, CHANGE_SIDES.y, CHANGE_SIDES.h);
                    endPose = createPose((CHANGE_SIDES.x), (CHANGE_SIDES.y - (tileSize * 4)), CHANGE_SIDES.h);
                    navigate.addPath(pathName, MAX_SPEED, HIGH_TOLERANCE, startPose);
                    navigate.appendPose(pathName, MAX_SPEED, HIGH_TOLERANCE, endPose);
                    break;

                case PARK:
                    pose = createPose(PARK.x, PARK.y, PARK.h);
                    navigate.addPath(getPathName(Auto.PathState.PARK), MAX_SPEED, MIN_TOLERANCE, pose);
                    break;

                case PARK_AUDIENCE:
                    pose = createPose(PARK_AUDIENCE.x, PARK_AUDIENCE.y, PARK_AUDIENCE.h);
                    navigate.addPath(getPathName(Auto.PathState.PARK_AUDIENCE), MAX_SPEED, MIN_TOLERANCE, pose);
                    break;

                default:
                    return;
            }
        }
    }

    private Pose createPose(double x, double y, double heading) {

        // The coordinates are the same for both alliances except the red alliance is a flipped mirror image
        double sign   = (alliance == Alliance.BLUE) ? 1 : -1;
        double rotate  = (alliance == Alliance.BLUE) ? 0 : - 180;
        return new Pose(x * sign, y * sign, Math.toRadians(AngleUnit.normalizeDegrees(heading + rotate)));
    }

    public void drawPaths() {
        if (Debug.drawPaths()) {
            navigate.drawPaths();
        }
    }

    public void addPath(PathState pathState, double x, double y, double heading) {
        navigate.addPath(getPathName(pathState), FAST_SPEED, 0, createPose(x, y, heading));
    }

    public void setStartPose(double x, double y, double heading) {
        navigate.setStartingPose(createPose(x, y, heading));
    }

    private String getPathName(PathState pathState) {
        return String.format("%s", pathState);
    }

    private PathState getPathState(String pathName) {
        for (PathState state : Auto.PathState.values()) {
            if (state.name().equals(pathName)) {
                return state;
            }
        }
        return Auto.PathState.UNKNOWN;
    }

    private void followPath() {
        waitForButtonPress();
        waitUntilOkToMove();
        pathIndex++;
        navigate.followPath(pathIndex);
    }

    public void waitUntilNotMoving() {
        Logger.debug("waiting");
        long start = System.currentTimeMillis();
        driveControl.waitUntilNotMoving(5000);
        Logger.debug("done waiting, time: %5d", System.currentTimeMillis() - start);

    }

    /**
     * Blocks execution until the robot is no longer busy or until a 6-second timeout occurs.
     * <p>
     * Continuously checks if the robot is busy and yields the current thread while waiting.
     * Logs a warning if the 6,000 ms timeout is reached before the robot becomes idle.
     */
    private void waitUntilRobotIsIdle() {
        Logger.debug("waiting");
        long timeout = 6000;
        timer.reset();
        while (robot.isBusy() && opMode.opModeIsActive()) {
            if (timer.milliseconds() > timeout) {
                Logger.warning("robot timed out after %d ms", timeout);
                break;
            }
            opMode.sleep(10);
        }
        Logger.debug("done waiting, time: %5.0f", timer.seconds());
    }

    private void waitUntilOkToMove() {
        robot.waitUntilOkToMove();
    }

    /**
     * Wait until the robot is near the pose.
     *
     * @noinspection unused
     */
    private void waitUntilNearPose() {
        Logger.debug("waiting");
        long timeout = 5000;
        timer.reset();
        while (opMode.opModeIsActive()) {
            if (driveControl.nearPose() ) {
                break;
            } else {
                opMode.sleep(10);
            }
            if (timer.milliseconds() > timeout) {
                Logger.warning("robot timed out");
                break;
            }
        }
        Logger.debug("done waiting, time: %5.0f", timer.milliseconds());
    }

    private void waitUntilHiveTip() {
        Logger.debug("waiting");
        long timeout = 25000;
        timer.reset();
        while (limelight.GetAprilTagID() == 0) {        // todo May need additional checks.
            if (timer.milliseconds() > timeout) {
                Logger.warning("robot timed out after %d ms", timeout);
                break;
            }
            opMode.sleep(10);
        }
        Logger.debug("done waiting, time: %5.0f", timer.seconds());
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
            opMode.sleep(10);
        }
        Logger.debug("done waiting for button press");
    }

}
