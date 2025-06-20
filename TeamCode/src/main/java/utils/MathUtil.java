package utils;

public class MathUtil {

    /**
     * This returns the sign (positive/negative) of a number.
     *
     * @param get the number.
     * @return returns the sign of the number.
     */
    public static double getSign(double get) {
        if (get == 0) return 0;
        if (get > 0) return 1;
        return -1;
    }

    /**
     * Returns an signed angle in radians between -pi and pi.
     *
     * @param radians angle in radians
     * @return signed angle in radians between -pi and pi.
     * @noinspection unused
     */
    private double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

    /**
     * Convert a polar angle, (0 - 2PI) where 0 is positive x to a signed angle (-PI to PI) where
     * 0 is positive y.
     *
     * @param polarAngle (0 - 2PI) 0 is positive x
     * @return signed angle (-PI to PI) 0 is positive y, counterclockwise is positive direction
     * @noinspection unused
     */
    private double polarToSignedAngle(double polarAngle) {

        double angle = polarAngle - Math.PI/2;
        if (angle > Math.PI)
            angle = angle - (Math.PI * 2);
        return angle;
    }

}
