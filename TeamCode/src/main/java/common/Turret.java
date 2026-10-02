package common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    private DcMotor turretMotor;

    // Constants based on 5203-312 RPM Motor and 1:2 External Gearing
    private static final double MOTOR_TICKS_PER_REV = 384.5;
    private static final double EXTERNAL_GEAR_RATIO = 2.0; // Motor turns twice for 1 turret turn
    private static final double TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * EXTERNAL_GEAR_RATIO) / 360.0;

    public Turret(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "turret");

        // Reset encoder and set to BRAKE mode so it doesn't drift
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setTargetPosition(0);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Rotates the turret to a specific angle.
     * @param degrees The target angle (e.g., 90 for a quarter turn).
     * @param power   The maximum speed (0.0 to 1.0).
     */
    public void rotateToAngle(double degrees, double power) {
        int targetTicks = (int) (degrees * TICKS_PER_DEGREE);

        turretMotor.setTargetPosition(targetTicks);
        turretMotor.setPower(power);
    }

    /**
     * Checks if the turret is still moving toward the target.
     */
    public boolean isBusy() {
        return turretMotor.isBusy();
    }

    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }
}