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
}
