package common;

/*
 * This file defines a Java Class that performs all the setup and configuration for the robot's hardware (motors and sensors).
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.Semaphore;

import utils.Pose;

//@SuppressWarnings({"FieldCanBeLocal", "unused"})
@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Robot extends Thread {

    public Drive            drive = null;
    private DriveGamepad    driveGamepad;
    private DriveControl    driveControl;
    private Launcher        launcher;
    private Limelight       limelight;
    //private ColorSensor     colorSensor;

    private final LinearOpMode opMode;

    private enum ROBOT_STATE { IDLE, FIRE, FIRE_ALL, LINE_UP_TARGET }
    private ROBOT_STATE robotState = ROBOT_STATE.IDLE;

    private Semaphore okToMove;

    boolean testRobot = false;

    public Robot(LinearOpMode opMode) {
        this.opMode = opMode;
        init();
    }

    /**
     * Initialize the Robot
     */
    public void init() {
        this.setName("robot");
        drive = new Drive(opMode);

        driveControl = new DriveControl(opMode, drive);
        driveControl.start();

        driveGamepad = new DriveGamepad(opMode, driveControl);

        try {
            launcher = new Launcher(opMode);
            launcher.start();
        } catch (Exception e) {
            Logger.error(e, "hardware not found", 2);
        }

        limelight = new Limelight(opMode);
        limelight.setPipeline(Limelight.Pipeline.APRIL_TAG);

        //colorSensor = new ColorSensor(opMode);
        //colorSensor.enable(true);

        okToMove = new Semaphore(1);

        if (! testRobot)
            start();
    }

    public void startDriveGamepad() {
        driveGamepad.start();
    }

    public void run() {

        try {
            Logger.message("robot thread started");

            runRobot();
            
            Logger.message("robot thread stopped");

        }  catch (Exception e) {
            Logger.error(e, "Exception");
            throw e;
        }
    }

    public void runRobot () {

        while (!opMode.isStarted()) Thread.yield();

        while (opMode.opModeIsActive()) {

            if (robotState == ROBOT_STATE.IDLE) {
                //colorSensor.update();
                Thread.yield();
                continue;
            }

            synchronized (this) {
                long start = System.currentTimeMillis();
                Logger.info("%s starts", robotState);

                switch (robotState) {
                    case FIRE:
                        launcher.fireLauncher();
                        while (launcher.isBusy())
                            delay(10);
                        setOkToMove(true);
                        break;

                    case FIRE_ALL:
                        launcher.fireAllArtifacts();
                        while (launcher.isBusy())
                            delay(10);
                        setOkToMove(true);
                        break;

                    case LINE_UP_TARGET:
                        lineUp();
                        break;
                }

                Logger.info("%s ends, time %6.2f", robotState, (double)(System.currentTimeMillis()-start)/1000.);

                robotState = ROBOT_STATE.IDLE;
            }

            //colorSensor.enable(false);
        }
    }

    private void delay (long milliseconds) {
        try {
            if (opMode.isStopRequested()) return;
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private void lineUp () {
        double angle = limelight.GetTx();
        if (angle == 0) return;

        Pose pose = driveControl.getPose();
        double heading = AngleUnit.normalizeRadians(pose.getHeading() - Math.toRadians(angle));
        Pose newPose = new Pose(pose.getX(), pose.getY(), heading);
        driveControl.moveToPose(newPose,0.2, 1000);
        while (driveControl.isBusy())
            delay(10);
        Logger.message("angle: %5.2f  pose: %s  new pose: %s", angle, pose.toString(), newPose.toString());
    }

    public boolean isBusy () {
        return robotState != ROBOT_STATE.IDLE  || launcher.isBusy();
    }

    public void fire() {
        synchronized (this) {
            setOkToMove(false);
            robotState = ROBOT_STATE.FIRE;
        }
    }

    public void fireAll() {
        synchronized (this) {
            setOkToMove(false);
            robotState = ROBOT_STATE.FIRE_ALL;
        }
    }

    public void lineUpTarget() {
        synchronized (this) {
            robotState = ROBOT_STATE.LINE_UP_TARGET;
        }
    }

    public void setLauncherSpeed(double speed) {
        launcher.setSpeed(speed);
    }

    private void waitUnitTime(double time) {
        while (System.currentTimeMillis() < time)
            delay(1);
    }

    private void setOkToMove(boolean ok)  {

        if (testRobot) return;

        try {
            if (ok) {
                Logger.message("ok to move");
                okToMove.release();
            } else {
                Logger.message("not ok to move");
                okToMove.acquire();
            }
        } catch (InterruptedException e) {
            Logger.message("InterruptedException");
        }
    }

    public boolean okToMove () {

        return okToMove.availablePermits() == 1;
    }

    public void waitUntilOkToMove () {
        if (okToMove.availablePermits() != 1) {
            Logger.message("wait until ok to move");
            while (okToMove.availablePermits() != 1)
                delay(1);                       // ToDo this a better why
            Logger.message("ok to move, continue");
        }
    }

    public boolean emergencyStop() {
        return opMode.gamepad1.back;
    }

    public void moveToCoordinate(double targetX, double targetY, double targetHeading, double timeout) {
        driveControl.moveToPose(targetX, targetY, targetHeading, timeout);
    }

    public boolean driveIsBusy() {
        return driveControl.isBusy();
    }

    public DriveControl getDriveControl () {
        return driveControl;
    }

    public Limelight getLimelight() {
        return limelight;
    }

} // end of class

