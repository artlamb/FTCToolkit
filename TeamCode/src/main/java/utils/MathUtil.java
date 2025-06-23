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
     * Returns the value of the first argument raised to the power of the second argument.
     * <p>
     * Special cases:
     * If the base is a negative number return the value of the positive base raise to the
     * exponent with a negative sign.
     *
     * @param a base
     * @param b exponent
     * @return a to the power of b.
     */
    public static double pow(double a, double b) {
        if (b == 1)
            return a;
        else if (b == 0)
            return 0;
        else if (a >= 0)
            return Math.pow(a, b);

        return(Math.pow(Math.abs(a), b) * MathUtil.getSign(a));
    }

    /**
     * Returns an signed angle in radians between -pi and pi.
     *
     * @param radians angle in radians
     * @return signed angle in radians between -pi and pi.
     * @noinspection unused
     */
    public double angleWrap(double radians) {
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
    public double polarToSignedAngle(double polarAngle) {

        double angle = polarAngle - Math.PI/2;
        if (angle > Math.PI)
            angle = angle - (Math.PI * 2);
        return angle;
    }

}
