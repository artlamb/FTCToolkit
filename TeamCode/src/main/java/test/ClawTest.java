package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import common.Logger;
import common.Robot;

@TeleOp(name=" Claw Test", group="Test")
@SuppressLint("DefaultLocale")

public class ClawTest extends LinearOpMode {

    private double SERVO_CLOSED = 0.57;
    private double SERVO_OPENED = 0.50;
    private double SERVO_CLOSED_VOLTAGE =0.3;

    Robot robot;
    AnalogInput currentSensor;
    DigitalChannel digitalChannel;
    LED redLED;
    LED greenLED;
    Servo servo;
    double lastRead = 0;


    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            servo = hardwareMap.get(Servo.class, "servo");
            currentSensor = hardwareMap.get(AnalogInput.class, "currentSensor");
            greenLED = hardwareMap.get(LED.class, "greenLED");
            redLED = hardwareMap.get(LED.class, "redLED");

            redLED.off();
            greenLED.off();

            //digitalChannel = hardwareMap.get(DigitalChannel.class, "led");
            //digitalChannel.setMode(DigitalChannel.Mode.OUTPUT);
            //digitalChannel.setState(true);

            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                if (gamepad1.x) {
                    servo.setPosition(SERVO_OPENED);
                    while (gamepad1.x) sleep(10);
                } else if (gamepad1.b) {
                    servo.setPosition(SERVO_CLOSED);
                    while (gamepad1.b) sleep(10);
                }

                readCurrentSensor();
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

    void readCurrentSensor() {
        double milliSeconds = System.currentTimeMillis();
        if (milliSeconds - lastRead < 500)
            return;

        lastRead = milliSeconds;

        double max = currentSensor.getMaxVoltage();
        double voltage = currentSensor.getVoltage();

        double RS = .10;          // Shunt resistor value (in ohms)

        // Follow the equation given by the INA169 datasheet to
        // determine the current flowing through RS. Assume RL = 10k
        // Is = (Vout x 1k) / (RS x RL)
        double current = voltage / (10 * RS);

        boolean closed =  (voltage > SERVO_CLOSED_VOLTAGE);
        redLED.enableLight(closed);
        greenLED.enableLight(!closed);

        Logger.message("sensor value: %f  max: %f  current: %f  closed: %b", voltage, max, voltage, current, closed);
    }
}
