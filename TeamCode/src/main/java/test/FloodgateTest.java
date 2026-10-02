package test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import common.Config;
import common.Floodgate;
import common.Logger;

@TeleOp(name=" Floodgate Test", group="Test")
//@SuppressLint("DefaultLocale")
@com.acmerobotics.dashboard.config.Config

public class FloodgateTest extends LinearOpMode {

    public static double power = 0.2;

    Floodgate floodgate;
    DcMotorEx motor;
    private long displayTime;


    @Override
    public void runOpMode() {
        try {
            initialize();

            waitForStart();

            motor.setPower(power);
            while (opModeIsActive()) {
                floodgate.update();
                display(1000);

                handleGamepad();
                Logger.debug("motor current %f", motor.getCurrent(CurrentUnit.AMPS));
                sleep(500);
                Thread.yield();
            }
            motor.setPower(0);

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    private void initialize() {

        floodgate = new Floodgate(this);
        motor = hardwareMap.get(DcMotorEx.class, Config.LEFT_FRONT);

        telemetry.addLine("Floodgate Test");
        telemetry.update();
    }

    private void handleGamepad() {

    }

    /**
     * Periodically display the current the robot is consuming.
     *
     * @param frequency the frequency in milliseconds to display the current
     */
    public void display(long frequency) {

        if (System.currentTimeMillis() - displayTime > frequency) {
            displayTime = System.currentTimeMillis();
            Logger.monitor(floodgate.toString());
        }
    }
}

