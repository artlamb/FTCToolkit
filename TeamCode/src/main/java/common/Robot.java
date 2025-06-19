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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.Semaphore;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
@com.acmerobotics.dashboard.config.Config           // allows public static to be changed in TFC Dashboard

public class Robot extends Thread {

    // lifter
    public static double LIFTER_SPEED = 0.50;
    public static double LIFTER_UP_SPEED = 0.70;
    public static double LIFTER_DOWN_SPEED = 0.50;
    public static double LIFTER_SPEED_LOW = 0.20;
    public static int    LIFTER_STOP_TICKS = 500;
    public static int    LIFTER_UP_POSITION = 1614;
    public static int    LIFTER_DOWN_POSITION = 0;
    public static int    LIFTER_HALF_UP_POSITION = 807;
    public static int    LIFTER_TOP_BAR_POSITION = 800;

    // arm extender
    public final int    ARM_IN = 0;
    public final int    ARM_OUT = 2000;
    public final int    AMR_OUT_PART_WAY = 750;
    public final int    ARM_OUT_START = 190;
    public final int    ARM_EXCHANGE = 270;
    public final int    ARM_AUTO_PICK = 620;
    public final int    ARM_AUTO_PICK_ROTATED = 560;
    public final double ARM_SPEED = 0.5;
    public final double ARM_HIGH_SPEED = 0.75;

    // Grabbers
    private final double PICKER_UP_POSITION   = 0.230;
    private final double PICKER_DOWN_POSITION = 0.890;

    private final double PICKER_FINGER_CLOSED  = 0.400;
    private final double PICKER_FINGER_OPEN    = 0.670;

    public  final double PICKER_YAW_0_DEGREES   = 0.167;
    public  final double PICKER_YAW_45_DEGREES  = 0.320;
    public  final double PICKER_YAW_90_DEGREES  = 0.494;
    public  final double PICKER_YAW_135_DEGREES = 0.657;

    private final double DROPPER_UP_POSITION     = 0.672;
    private final double DROPPER_ASCENT_POSITION = 0.600;
    private final double DROPPER_DROP_POSITION   = 0.280;
    private final double DROPPER_DOWN_POSITION   = 0.020;

    private final double DROPPER_SPECIMEN_UP   = 0.550;
    private final double DROPPER_SPECIMEN_DOWN = 0.550;

    private final double DROPPER_FINGER_CLOSED = 0.415;
    private final double DROPPER_FINGER_OPEN   = 0.590;

    private final double SPECIMEN_STOP_POSITION = 5;

    private boolean pickerOpened = true;
    private boolean dropperOpened = false;
    private boolean pickerUp = false;
    private boolean dropperUp = false;
    private double  pickerYawPosition = PICKER_YAW_0_DEGREES;

    // Define Motor and Servo objects
    private Servo           pickerWrist;
    private Servo           pickerFingers;
    private Servo           pickerYaw;
    private Servo           dropperWrist;
    private Servo           dropperFingers;

    private Lifter          lifter;

    private DcMotor         arm;
    private MotorControl    armControl;

    // drivetrain
    public Drive      drive = null;
    private DriveGamepad driveGamepad;
    private DriveControl driveControl;

    // Declare OpMode members.
    private final LinearOpMode opMode;

    private enum ROBOT_STATE { IDLE, SET_TO_START_POSITION, SET_TO_STOP_POSITION, PICKUP_SAMPLE, PICKUP_ROTATED_SAMPLE, MOVE_SAMPLE_TO_DROPPER, DROP_SAMPLE_INTO_TOP_BUCKET, SCORE_SPECIMEN }
    private ROBOT_STATE robotState = ROBOT_STATE.IDLE;

    private int pickingPosition = AMR_OUT_PART_WAY;

    private Semaphore okToMove;
    private Semaphore okToLift;

    boolean testRobot = false;

    // Define a constructor that allows the OpMode to pass a reference to itself.
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

        okToMove = new Semaphore(1);
        okToLift = new Semaphore(1);

        try {
            pickerWrist = opMode.hardwareMap.get(Servo.class, Config.PICKER_WRIST);
            pickerFingers = opMode.hardwareMap.get(Servo.class, Config.PICKER_FINGERS);
            pickerYaw = opMode.hardwareMap.get(Servo.class, Config.PICKER_YAW);

            dropperWrist = opMode.hardwareMap.get(Servo.class, Config.DROPPER_WRIST);
            dropperFingers = opMode.hardwareMap.get(Servo.class, Config.DROPPER_FINGERS);

        } catch (Exception e) {
            Logger.error(e, "hardware not found", 2);
        }

        try {
            arm = opMode.hardwareMap.get(DcMotor.class, Config.ARM);
            arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            arm.setDirection(DcMotorSimple.Direction.REVERSE);
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);       // ToDo should we do this?
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            armControl = new MotorControl(opMode, arm);
            armControl.setName("arm");
            armControl.start();

        } catch (Exception e) {
            Logger.error(e, "hardware not found", 2);
        }

        try {
            lifter = new Lifter(opMode);
            lifter.setLowSpeedThreshold(LIFTER_STOP_TICKS);
            lifter.start();

        } catch (Exception e) {
            testRobot = true;
            Logger.error(e, "hardware not found", 2);
        }

        if (! testRobot)
            start();
    }

    public void resetEncoders () {
        DcMotor.RunMode mode = arm.getMode();
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(mode);

        lifter.resetEncoders();
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

        long start;

        while (opMode.opModeIsActive()) {
            switch (robotState) {
                case IDLE:
                    Thread.yield();
                    continue;

                case SET_TO_START_POSITION:
                    Logger.message("\n** Set to start position");
                    synchronized (this) {
                        double  time = System.currentTimeMillis();
                        dropperClose();
                        dropperUp();
                        armMoveTo(ARM_OUT_START, ARM_HIGH_SPEED);
                        while (armIsBusy()) delay(10);
                        pickerRotateTo(PICKER_YAW_0_DEGREES);
                        pickerOpen();
                        pickerDown();
                        //armMoveTo(pickingPosition, ARM_HIGH_SPEED);
                        waitUnitTime(time + 300);       // make sure the dropper is in the up position
                        robotState = ROBOT_STATE.IDLE;
                    }
                    Logger.message("\n** Set to start position ends");
                    break;

                case SET_TO_STOP_POSITION:
                    Logger.message("\n** Set to stop position");
                    synchronized (this) {
                        pickerRotateTo(PICKER_YAW_0_DEGREES);
                        pickerClose();
                        pickerDown();
                        dropperOpen();
                        dropperDown();
                        armMoveTo(ARM_IN);
                        robotState = ROBOT_STATE.IDLE;
                    }
                    Logger.message("\n** Set to stop position ends");
                    break;

                case PICKUP_SAMPLE:
                    Logger.message("\n** Pickup sample");
                    start = System.currentTimeMillis();
                    synchronized (this) {
                        pickerClose();
                        dropperOpen();
                        dropperDown();
                        delay(350);
                        pickerUp();
                        pickerRotateTo(PICKER_YAW_0_DEGREES);
                        setOkToMove(true);
                        robotState = ROBOT_STATE.IDLE;
                    }
                    Logger.message("\n** Pickup sample ends, time %d", System.currentTimeMillis()-start);
                    break;

                case PICKUP_ROTATED_SAMPLE:
                    Logger.message("\n** Pickup rotated sample");
                    start = System.currentTimeMillis();
                    synchronized (this) {
                        pickerClose();
                        dropperOpen();
                        dropperDown();
                        delay(350);
                        armMoveTo(ARM_EXCHANGE, ARM_HIGH_SPEED);
                        delay(400);
                        pickerUp();
                        pickerRotateTo(PICKER_YAW_0_DEGREES);
                        setOkToMove(true);
                        robotState = ROBOT_STATE.IDLE;
                    }
                    Logger.message("\n** Pickup sample rotated ends, time %d", System.currentTimeMillis()-start);
                    break;

                case MOVE_SAMPLE_TO_DROPPER:
                    Logger.message("\n** move sample to dropper");
                    start = System.currentTimeMillis();
                    synchronized (this) {
                        pickerUp();
                        delay(500);
                        while (lifterIsBusy() && opMode.opModeIsActive()) {
                            delay(10);
                        }
                        dropperClose();
                        delay(200);          // wait for the dropper to get to the closed position
                        pickerOpen();
                        delay(100);
                        setOkToLift(true);
                        dropperUp();
                        pickerDown();
                        delay(400);
                        robotState = ROBOT_STATE.IDLE;
                    }
                    Logger.message("\n** move sample to dropper ends, time %d", System.currentTimeMillis()-start);
                    break;

                case DROP_SAMPLE_INTO_TOP_BUCKET:
                    Logger.message("\n**  Drop sample into top bucket");
                    synchronized (this) {
                        while (lifterIsBusy() && opMode.opModeIsActive()) {
                            delay(10);
                        }
                        dropperDropPosition();
                        delay(200);
                        dropperOpen();
                        delay(200);
                        dropperUp();
                        delay(100);
                        pickerRotateTo(PICKER_YAW_0_DEGREES);
                        setOkToMove(true);
                        robotState = ROBOT_STATE.IDLE;
                    }
                    break;

                case SCORE_SPECIMEN:
                    Logger.message("** Score specimen");
                    synchronized (this) {
                        lifterToTopBar();
                        while (lifterIsBusy() && opMode.opModeIsActive()) {
                            delay(10);
                        }

                        dropperSpecimenUp();
                        delay(200);

                        driveControl.moveToObject(SPECIMEN_STOP_POSITION, 2000);
                        while (driveControl.isBusy()&& opMode.opModeIsActive()) {
                            delay(10);
                        }

                        dropperSpecimenDown();
                        delay(200);

                        dropperOpen();
                        delay(100);

                        dropperUp();
                        delay(200);

                        lifterDown();
                        setOkToMove(true);
                    }
            }
        }
    }

    private void delay (long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    /**
     * Extend lifter to the specified position
     */
    public void LifterExtend() {
        lifter.runLifter(LIFTER_SPEED);
    }

    /**
     * Retract lifter to the specified position
     */
    public void lifterRetract () {
        lifter.runLifter(-LIFTER_SPEED);
    }

    /**
     * Stop the lifter from moving
     */
    public void lifterStop () {
        lifter.stopLifter();
    }

    /**
     * Raise the lifter to the specified position
     */
    public void lifterUp() {
        Logger.message("set lifter to position %d at %4.2f speed", LIFTER_UP_POSITION, LIFTER_UP_SPEED);
         lifter.setPosition(LIFTER_UP_POSITION, LIFTER_UP_SPEED);
    }

    /**
     * Lower the lifter to its down position
     */
    public void lifterDown() {
        lifter.setPosition(LIFTER_DOWN_POSITION, LIFTER_DOWN_SPEED, LIFTER_SPEED_LOW);
    }

    public void lifterHalfUp() {
        Logger.message("set lifter to position %d at %4.2f speed", LIFTER_HALF_UP_POSITION, LIFTER_UP_SPEED);
        lifter.setPosition(LIFTER_HALF_UP_POSITION, LIFTER_UP_SPEED);
    }


    /**
     * Raise the lifter to the specimen top bar scoring position
     */
    public void lifterToTopBar () {
        lifter.setPosition(LIFTER_TOP_BAR_POSITION, LIFTER_SPEED, LIFTER_SPEED);
    }

    /**
     * Check if the lifter is moving
     * @return true if the lifter is moving
     */
    public boolean lifterIsBusy () {
        return lifter.lifterIsBusy();
    }

    /**
     * Returns true if the arm is extendable
     * @return true if extendable, false if fully extend
     */
    public boolean armExtendable() {
        int position = arm.getCurrentPosition();
        boolean extendable = position < ARM_OUT;
        Logger.message("arm position %d %B", position, extendable);
        return extendable;
    }

    /**
     * Returns true if the arm is retractable from its current position
     * @return true if fully retracted
     */
    public boolean armRetractable() {
        int position = arm.getCurrentPosition();
        boolean retractable = position > ARM_IN;
        Logger.message("arm position %d %B", position, retractable);
        return  retractable;
    }

    /**
     * Extend arm to the specified position
     */
    public void armExtend() {
        armControl.runMotor(ARM_SPEED);
    }

    /**
     * Retract arm to the specified position
     */
    public void amrRetract() {
        armControl.runMotor(-ARM_SPEED);
    }

    /**
     * Stop the arm from moving
     */
    public void armStop () {
        armControl.stopMotor();
    }

    public boolean armIsBusy() {
        return armControl.motorIsBusy();
    }
    /**
     * Move the arm to the specified position
     * @param position target position
     */
    public void armMoveTo (int position) {
        armControl.setPosition(position, ARM_SPEED, ARM_SPEED);
    }

    public void armMoveTo (int position, double speed) {
        armControl.setPosition(position, speed, speed);
    }

    private void runMotorToPosition(DcMotor motor, int position, double speed, double lowSpeed) {

        ElapsedTime elapsedTime = new ElapsedTime();

        int current = motor.getCurrentPosition();
        int last = current;
        Logger.message("run from %d to %d", current, position);
        DcMotor.RunMode mode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(speed);

        double lastMoveTime = 0;
        elapsedTime.reset();
        while (opMode.opModeIsActive()) {
            if (! motor.isBusy())
                break;
            if (emergencyStop())
                break;

            // if the motor has not moved for a while, kill the power
            current = motor.getCurrentPosition();
            if (current != last) {
                lastMoveTime = elapsedTime.milliseconds();
                last = current;
            } else if (elapsedTime.milliseconds() - lastMoveTime > 100) {
                Logger.message("motor not moving");
                break;
            }

            int remaining = Math.abs(position-current);
            if (remaining < LIFTER_STOP_TICKS && speed != lowSpeed) {
                motor.setPower(lowSpeed);
                Logger.message("remaining %d set to lower speed");
            }

            //Logger.message("position %5d   remaining %5d  elapsed %6.2f ", current, remaining, elapsedTime.milliseconds());
        }
        motor.setPower(0);
        motor.setMode(mode);
    }

    public void pickerUp() {
        Logger.message("set picker to position %f ", PICKER_UP_POSITION);
        pickerWrist.setPosition(PICKER_UP_POSITION);
        pickerUp = true;
    }

    public void pickerDown() {
        Logger.message("set picker to position %f", PICKER_DOWN_POSITION);
        pickerWrist.setPosition(PICKER_DOWN_POSITION);
        pickerUp = false;
    }

    public void pickerOpen(){
        Logger.message("set picker to position %f", PICKER_FINGER_OPEN);
        pickerFingers.setPosition(PICKER_FINGER_OPEN);
        pickerOpened = true;
    }

    public void pickerClose() {
        Logger.message("set picker to position %f", PICKER_FINGER_CLOSED);
        pickerFingers.setPosition(PICKER_FINGER_CLOSED);
        pickerOpened = false;
    }

    public boolean pickerIsOpen() {
        Logger.message("picker is open: %b", pickerOpened);
        return pickerOpened;
    }

    public boolean pickerIsUp() {
        return pickerUp;
    }

    public void pickerRotate() {

        if (pickerYawPosition == PICKER_YAW_0_DEGREES) {
            pickerYawPosition = PICKER_YAW_45_DEGREES;
        } else if (pickerYawPosition == PICKER_YAW_45_DEGREES) {
            pickerYawPosition = PICKER_YAW_90_DEGREES;
        } else if (pickerYawPosition == PICKER_YAW_90_DEGREES) {
            pickerYawPosition = PICKER_YAW_135_DEGREES;
        } else {
            pickerYawPosition = PICKER_YAW_0_DEGREES;
        }
        pickerYaw.setPosition(pickerYawPosition);
    }

    public void pickerRotateTo(double position) {
        pickerYaw.setPosition(position);
        pickerYawPosition = position;
    }

    public void dropperUp() {
        Logger.message("set dropper to position %f", DROPPER_UP_POSITION);
        dropperWrist.setPosition(DROPPER_UP_POSITION);
        dropperUp = true;
    }

    public boolean dropperIsUp () {
        return dropperUp;
    }

    public void dropperDropPosition () {
        Logger.message("set dropper to position %f", DROPPER_DOWN_POSITION);
        dropperWrist.setPosition(DROPPER_DROP_POSITION);
        dropperUp = false;
    }

    public void dropperDown() {
        Logger.message("set dropper to position %f", DROPPER_DOWN_POSITION);
        dropperWrist.setPosition(DROPPER_DOWN_POSITION);
        dropperUp = false;
    }

    public void dropperSpecimenUp() {
        dropperWrist.setPosition(DROPPER_SPECIMEN_UP);
        dropperUp = false;
    }

    public void dropperSpecimenDown() {
        dropperWrist.setPosition(DROPPER_SPECIMEN_DOWN);
        dropperUp = false;
    }

    /** @noinspection unused*/
    public void dropperAscent() {
        dropperWrist.setPosition(DROPPER_ASCENT_POSITION);
        dropperUp = false;
    }

    public void dropperOpen(){
        Logger.message("set dropper to position %f", DROPPER_FINGER_OPEN);
        dropperFingers.setPosition(DROPPER_FINGER_OPEN);
        dropperOpened = true;
    }

    public void dropperClose(){
        Logger.message("set dropper to position %f", DROPPER_FINGER_CLOSED);
        dropperFingers.setPosition(DROPPER_FINGER_CLOSED);
        dropperOpened = false;
    }

    public boolean dropperIsOpen() {
        Logger.message("dropper is open: %b", pickerOpened);
        return dropperOpened;
    }

    public boolean emergencyStop() {
        return opMode.gamepad1.back;
    }

    public void turn(double degrees) {
        drive.turn(degrees);
    }

    public void forward (double distance) {
        drive.forward(distance);
    }

    public void back (double distance) {
        drive.back(distance);
    }

    public void strafeLeft (double distance) { drive.strafeLeft(distance); }

    public void strafeRight (double distance) {
        drive.strafeRight(distance);
    }

    public boolean isBusy () {
        return lifter.lifterIsBusy() || armControl.motorIsBusy() || robotState != ROBOT_STATE.IDLE;
    }

    public void setToStartPosition() {
        synchronized (this) {
            pickingPosition = ARM_EXCHANGE;
            robotState = ROBOT_STATE.SET_TO_START_POSITION;
        }
    }

    public void setToStopPosition() {
        synchronized (this) {
            robotState = ROBOT_STATE.SET_TO_STOP_POSITION;
        }
    }

    public void setToPickingPosition(int position) {
        synchronized (this) {
            pickingPosition = position;
            robotState = ROBOT_STATE.SET_TO_START_POSITION;
        }
    }

    public void pickUpRotatedSample() {
        synchronized (this) {
        robotState = ROBOT_STATE.PICKUP_ROTATED_SAMPLE;
            setOkToMove(false);
        }
    }

    public void pickUpSample() {
        synchronized (this) {
            robotState = ROBOT_STATE.PICKUP_SAMPLE;
            setOkToMove(false);
        }
    }

    public void moveSampleToDropper () {
        synchronized (this) {
            robotState = ROBOT_STATE.MOVE_SAMPLE_TO_DROPPER;
            setOkToLift(false);
        }
    }

    public void dropSampleInTopBucket() {
        synchronized (this) {
            robotState = ROBOT_STATE.DROP_SAMPLE_INTO_TOP_BUCKET;
            setOkToMove(false);
        }
    }

    public void scoreSpecimen() {
        synchronized (this) {
            robotState = ROBOT_STATE.SCORE_SPECIMEN;
            setOkToMove(false);
        }
    }

    public void pushSample() {

    }

    private void waitUnitTime(double time) {
        while (System.currentTimeMillis() < time)
            delay(1);
    }

    public void waitUntilOkToMove () {
        if (okToMove.availablePermits() != 1) {
            Logger.message("wait until ok to move");
            while (okToMove.availablePermits() != 1)
                delay(1);                       // ToDo this a better why
            Logger.message("ok to move, continue");
        }
    }

    public boolean okToMove () {
        return okToMove.availablePermits() == 1;
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

    public void waitUntilOkToLift () {
        if (okToLift.availablePermits() != 1) {
            Logger.message("wait until ok to lift");
            while (okToLift.availablePermits() != 1)
                delay(1);                       // ToDo this a better why
            Logger.message("ok to lift");
        }
    }

    public boolean okToLift () {
        return okToLift.availablePermits() == 1;
    }

    private void setOkToLift(boolean ok)  {

        if (testRobot) return;

        try {
            if (ok) {
                Logger.message("set ok to lift");
                okToLift.release();
            } else {
                Logger.message("set not ok to move");
                okToLift.acquire();
            }
        } catch (InterruptedException e) {
            Logger.message("InterruptedException");
        }
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

} // end of class

