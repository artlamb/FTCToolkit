package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import common.Drive;
import common.DriveControl;
import common.Logger;
import utils.Pose;

@TeleOp(name="DecelerationTest", group="Test")
@SuppressLint("DefaultLocale")

public class DecelerationTest extends LinearOpMode {

    public static double MIN_POWER = 0.25;
    public static double MAX_POWER = 0.95;
    public static double INCHES_TO_DRIVE = 60;

    double ODOMETER_WHEEL_DIAMETER = 48/25.4;   // wheel diameter in inches
    double ODOMETER_COUNT_PER_REV = 2000;       // encoder count
    double ODOMETER_COUNT_PER_WHEEL_REV = ODOMETER_COUNT_PER_REV * ODOMETER_WHEEL_DIAMETER * Math.PI;

    Drive drive  = null;
    DriveControl driveControl;
    DcMotorEx odometer;

    @Override
    public void runOpMode() {
        try {
            drive = new Drive(this);
            drive.start();
            driveControl = new DriveControl(this, drive);
            driveControl.resetIMU();

            odometer = drive.leftFrontDrive;

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            accelerateToVelocity();

        } catch (Exception e) {
            Logger.error(e, "Error");
        }

    }

    private void accelerateToVelocity () {

        ElapsedTime timer = new ElapsedTime();
        drive.setRunWithEncoders(false);

        double maxVelocity = drive.getMaxVelocity();
        for (double percent = 0.1; percent <= 0.7; percent += 0.1) {
            double targetVelocity = maxVelocity * percent;
            double currentVelocity;
            double accelerationTime = 0;
            double scaled =  Math.min(maxVelocity, targetVelocity * 1.5);
            Logger.message("percent %5.0f   velocity %5.2f", percent*100, targetVelocity);
            drive.resetEncoders();

            drive.setVelocity(scaled, scaled, scaled, scaled);
            timer.reset();

            boolean accelerate = true;
            Pose start = null;
            Pose stop;
            Pose velocity;
            double magnitude = 0;
            double angle = 0;
            do {
                currentVelocity = drive.getCurrentVelocity();
                velocity = driveControl.getVelocity();
                Logger.message("%5.2f  %5.0f  %5.0f   %6.2f  %6.2f  %6.2f",
                        timer.seconds(), currentVelocity, currentVelocity/targetVelocity*100, velocity.getX(), velocity.getY(), Math.toDegrees(velocity.getHeading()));

                if (accelerate && currentVelocity >= targetVelocity) {
                    start = driveControl.getPose();
                    accelerationTime = timer.seconds();
                    magnitude  = Math.hypot(velocity.getX(), velocity.getY());
                    angle = Math.toDegrees(Math.atan2(velocity.getX(), velocity.getY()));
                    drive.setVelocity(0);
                    accelerate = false;

                } else if ((! accelerate) && (Math.abs(currentVelocity) <= 10)) {
                    stop = driveControl.getPose();
                    break;
                }
            } while (true);

            double distance = Math.hypot(stop.getX() - start.getX(), stop.getY() - start.getY());
            double decelerationTime = timer.seconds() - accelerationTime;
            double traveled = drive.getDistanceTraveled();
            Logger.message("percent %5.0f  time %5.2f  acceleration %5.2f  deceleration %5.2f  velocity %5.2f  angle %6.1f  magnitude %6.1f  deceleration distance %5.2f",
                    percent*100, timer.seconds(), accelerationTime, decelerationTime, targetVelocity, angle, magnitude, distance);

            sleep(2000);

            drive.moveDistance(Drive.DIRECTION.BACK, 0.2, traveled, 4000);
        }
    }

    private void forwardTest() {

        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();


        while (opModeIsActive()) {

            for (double power = MIN_POWER; power <= MAX_POWER; power += 0.05) {

                // move forward to calculate the overshoot error
                double maxVelocity = move(power, 1, 0);
                sleep(2000);

                double odometerInches = Math.abs(odometer.getCurrentPosition()) / ODOMETER_COUNT_PER_WHEEL_REV;
                double error = odometerInches - INCHES_TO_DRIVE;
                displayResults(power, voltageSensor.getVoltage(), maxVelocity, error);

                // return to start position
                move(power, -1, 0);
                sleep(2000);

                // test adjusting for overshoot error
                move(power, 1, error);
                sleep(2000);

                odometerInches = Math.abs(odometer.getCurrentPosition()) / ODOMETER_COUNT_PER_WHEEL_REV;
                error = odometerInches - INCHES_TO_DRIVE;
                displayResults(power, voltageSensor.getVoltage(), maxVelocity, error);

                // return to start position
                move(power, -1, error);
                sleep(2000);

                Logger.addLine("\n");
            }
        }
    }

    private double move (double power, int sign, double error) {

        // Determine new target position
        int target = (int) (Math.max(INCHES_TO_DRIVE - error, 0) * drive.encoderTicksPerInch()) * sign;

        for (DcMotor motor : drive.motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(target);
        }

        // Drive until all motor reach target position
        double maxVelocity = 0;
        drive.accelerationReset();
        while (opModeIsActive()) {
            for (DcMotor motor : drive.motors) {
                double rampPower = drive.accelerationLimit(power);
                motor.setPower(rampPower * sign);
            }

            boolean busy = false;
            for (DcMotor motor : drive.motors) {
                busy |= motor.isBusy();
            }
            if (! busy) break;

            double velocity = odometer.getVelocity();
            maxVelocity = Math.max(maxVelocity, velocity);
        }

        // Stop all motion;
        for (DcMotor motor : drive.motors)
            motor.setPower(0);

        return maxVelocity;
    }

    private void displayResults(double power, double voltage, double maxVelocity, double error) {
        Logger.message("%s",
                String.format("power %4.1f   ", power) +
                String.format("voltage: %5.2f", voltage) +
                String.format("max velocity:  %5.0f   ", maxVelocity) +
                String.format("error %6.2f", error));
    }
}
