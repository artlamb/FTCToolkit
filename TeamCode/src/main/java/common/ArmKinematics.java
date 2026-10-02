package common;

/**
 * <h6>
 * Forward Kinematics and Inverse Kinematics for a 2-segment Robot Arm.
 * </h6>
 * <div> <br>
 * This class use Forward Kinematics and Inverse Kinematics to controls a planar 2-DOF arm, a two-section
 * arm that operates within a 2D plane and has two independent degrees of freedom (DOF), meaning two
 * independent inputs (e.g., motor angles) are required to control the position of its end-effector.
 *  </div> <br>
 *  Forward Kinematics Math:<br>
 *  <ul>
 *  Calculates the (x, y) position of the arm tip based on known joint angles:
 *  <li>theta1: The angle of the first segment (L1) relative to the ground.</li>
 *  <li>theta2: The angle of the second segment (L2) relative to the first segment.</li>
 *  <br>
 *
 *  Using Trigonometry:<br>
 *  <li>The x-coordinate is the sum of the horizontal components:<br>
 *      x = (L1 * cos(theta1)) + (L2 * cos(theta1 + theta2))</li>
 *  <li>The y-coordinate is the sum of the vertical components:<br>
 *      y = (L1 * sin(theta1)) + (L2 * sin(theta1 + theta2))</li>
 *  <br>
 *  Note: We add theta1 and theta2 together for the second segment because the elbow motor
 *  is mounted on the first segment, so its angle is relative to the arm's current orientation.
 * <br>
 * </ul><br>
 * Inverse Kinematics Math:<br>
 * <ul>
 * Given a target (x, y) coordinate, we need to find:<br>
 * <li>theta2 (Elbow Angle): How much the second section is bent relative to the first.</li>
 * <li>theta1 (Shoulder Angle): The angle of the first section relative to the ground.</li>
 * <br>
 * Using the Law of Cosines:<br>
 * <li>L1 and L2 are the lengths of the two sections.</li>
 * <li>c^2 = x^2 + y^2 (The squared distance from base to target)</li>
 * <li>cos(theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)</li>
 * <li>theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2))</li>
 * <br>
 * Note: FTC servos usually take a value from 0.0 to 1.0. This class converts
 * the calculated radians into those specific servo positions.
 * </ul><br>
 * Assumes the arm moves in a 2D vertical plane (X = forward, Y = up).
 */
public class ArmKinematics {

    private final double L1; // Length of the first segment (e.g., shoulder to elbow)
    private final double L2; // Length of the second segment (e.g., elbow to wrist)

    // Servo calibration constants (these depend on how you mounted your servos)
    // You want to map "0 radians" to a specific servo position.
    // These values assumes that when the IK math calculates 0 radians (the arm pointing straight out),
    // the servo should be at its center point. If the arm is physically constrained or mounted
    // such that "0 radians" is at one end of the servo's travel, you would change the offset to 0.0 or 1.0.
    private final double shoulderOffset = 0.5;
    private final double elbowOffset = 0.5;

    // Radians are converted to servo positions assuming a servo with 300 degrees of rotation and a
    // 1:1 gear ratio between the servo and the arm.
    private final double gearRatio = 1.0; // Servo Teeth / Arm Teeth
    private final double servoRotation = 300;   // 300 degrees of rotation
    private final double shoulderTicksPerRadian = (1.0 / Math.toRadians(servoRotation)) / gearRatio;
    private final double elbowTicksPerRadian = (1.0 / Math.toRadians(servoRotation)) / gearRatio;

    /**
     * @param L1 Length of the first arm segment.
     * @param L2 Length of the second arm segment.
     */
    public ArmKinematics(double L1, double L2) {
        this.L1 = L1;
        this.L2 = L2;
    }

    /**
     * Result class to hold the calculated servo positions.
     */
    public static class Positions {
        public double shoulderServo;
        public double elbowServo;
        public boolean isValid; // False if the target is out of reach

        public Positions(double s, double e, boolean valid) {
            this.shoulderServo = s;
            this.elbowServo = e;
            this.isValid = valid;
        }
    }

    /**
     * Result class to hold calculated (x, y) coordinates.
     */
    public static class Point {
        public double x;
        public double y;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    /**
     * Calculates the (x, y) position of the arm tip given the current servo positions.
     * This is Forward Kinematics.
     *
     * @param shoulderServo Current position of shoulder servo (0.0 to 1.0)
     * @param elbowServo    Current position of elbow servo (0.0 to 1.0)
     * @return Point object containing x and y coordinates relative to shoulder base.
     */
    public Point forwardKinematics(double shoulderServo, double elbowServo) {
        // 1. Convert servo positions back to radians
        // Formula: theta = (servoPos - offset) / ticksPerRadian
        double theta1 = (shoulderServo - shoulderOffset) / shoulderTicksPerRadian;
        double theta2 = (elbowServo - elbowOffset) / elbowTicksPerRadian;

        // 2. Calculate coordinates using trigonometry
        // The elbow angle (theta2) is relative to the first segment (L1).
        // Therefore, the global angle of the second segment is (theta1 + theta2).
        double x = L1 * Math.cos(theta1) + L2 * Math.cos(theta1 + theta2);
        double y = L1 * Math.sin(theta1) + L2 * Math.sin(theta1 + theta2);

        return new Point(x, y);
    }
    /**
     * Calculates servo positions to reach a specific (x, y) point.
     * @param x Horizontal distance from the shoulder base.
     * @param y Vertical distance from the shoulder base.
     * @return ArmResult containing servo values 0.0 - 1.0.
     */
    public Positions reverseKinematics(double x, double y) {
        // Distance from base to target squared
        double distSq = x * x + y * y;
        double dist = Math.sqrt(distSq);

        // Check if the point is reachable (Triangle Inequality)
        if (dist > (L1 + L2) || dist < Math.abs(L1 - L2)) {
            return new Positions(0, 0, false);
        }

        // Calculate Elbow Angle (Theta 2) using Law of Cosines
        // cos(theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)
        double cosTheta2 = (distSq - L1 * L1 - L2 * L2) / (2 * L1 * L2);

        // Safety check for rounding errors
        cosTheta2 = Math.max(-1, Math.min(1, cosTheta2));

        double theta2 = Math.acos(cosTheta2);

        // Calculate Shoulder Angle (Theta 1)
        // theta1 = atan2(y, x) - atan2(L2*sin(t2), L1 + L2*cos(t2))
        double theta1 = Math.atan2(y, x) - Math.atan2(L2 * Math.sin(theta2), L1 + L2 * Math.cos(theta2));

        // Convert Radians to Servo Positions (0.0 to 1.0)
        // This mapping depends entirely on your specific build!
        double shoulderPos = shoulderOffset + (theta1 * shoulderTicksPerRadian);
        double elbowPos = elbowOffset + (theta2 * elbowTicksPerRadian);

        return new Positions(
                clamp(shoulderPos),
                clamp(elbowPos),
                true
        );
    }

    private double clamp(double val) {
        return Math.max(0.0, Math.min(1.0, val));
    }
}