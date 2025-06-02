package test;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import common.Logger;
import common.Robot;
import utils.Dashboard;
import utils.Pose;

@TeleOp(name=" Claw Test", group="Test")
@SuppressLint("DefaultLocale")

public class ClawTest extends LinearOpMode {

    Robot robot;
    AnalogInput currentSensor;
    DigitalChannel digitalChannel;

    @Override
    public void runOpMode() {
        try {
            robot = new Robot(this);
            currentSensor = hardwareMap.get(AnalogInput.class, "currentSensor");
            //currentSensor = new AnalogInput(0);

            digitalChannel = hardwareMap.get(DigitalChannel.class, "led");
            digitalChannel.setMode(DigitalChannel.Mode.OUTPUT);

            boolean state = false;
            for (int i = 0; i < 10; i++) {
                state = ! state;
                digitalChannel.setState(state);
                Logger.message("led is %b", state);
                if (state)
                    sleep(3000);
                else
                    sleep(1000);
            }



            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            telemetry.addLine("Press start");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                double max = currentSensor.getMaxVoltage();
                double sensorValue = currentSensor.getVoltage();

                // Remap the ADC value into a voltage number (5V reference)
                int RS = 10;          // Shunt resistor value (in ohms)
                int VOLTAGE_REF = 5;  // Reference voltage for analog read
                double voltage = (sensorValue * VOLTAGE_REF) / 1023;

                // Follow the equation given by the INA169 datasheet to
                // determine the current flowing through RS. Assume RL = 10k
                // Is = (Vout x 1k) / (RS x RL)
                double current = voltage / (10 * RS);

                Logger.message("sensor value: %f  max: %f  voltage: %f  current: %f", sensorValue, max, voltage, current);
                sleep(500);
            }

        } catch (Exception e) {
            Logger.error(e, "Error");
        }
    }

}
