package frc.robot.util;

public class LookUpTable {

    /**
     * Represents a single entry in the lookup table with distance and corresponding values
     */
    private static class LookUpTableEntry {
        final double distance;
        final double hoodpiv;
        final double rpm;

        public LookUpTableEntry(double distance, double hoodpiv, double rpm) {
            this.distance = distance;
            this.hoodpiv = hoodpiv;
            this.rpm = rpm;
        }
    }

    /**
     * Lookup table entries - adjust these values based on your robot's calibration
     * Distance is in meters (or your preferred unit)
     */
    private static final LookUpTableEntry[] LOOKUP_TABLE = {
        // Format: distance, armPivot (0..90, larger when closer), shooterVelocity (e.g., RPM)
        new LookUpTableEntry(1.0601, 85.0, 120),
        new LookUpTableEntry(1.25,   78.0, 110),
        new LookUpTableEntry(1.5,    70.0, 100),
        new LookUpTableEntry(1.75,   62.0, 20),
        new LookUpTableEntry(2.0,    52.0, 90),
        new LookUpTableEntry(2.25,   45.0, 80),
        new LookUpTableEntry(2.5,    35.0, 70),
        new LookUpTableEntry(3.0,    20.0, 70),
        new LookUpTableEntry(3.5,    10.0, 60),
        new LookUpTableEntry(4.0,     5.0, 50)
    };

    public LookUpTable() {
    }

    public static class LookUpTableTest {
        double distance = 0;
        double armpiv = 0;
        double shooterRPS = 0;

        public LookUpTableTest(double hoodpiv, double shooterRPS) {
            this.armpiv = hoodpiv;
            this.shooterRPS = shooterRPS;
        }

        public double getAngle() {
            return armpiv;
        }

        public double getRPS() {
            return shooterRPS;
        }
    }

    /**
     * Gets the lookup table output for a given distance using linear interpolation
     * 
     * @param distance The distance to look up (in meters or your preferred unit)
     * @return LookUpTableTest containing interpolated values for arm pivot and shooter speeds
     */
    public static LookUpTableTest LookUpTableOutput(double distance) {
        // Handle edge cases: distance below minimum or above maximum
        if (distance <= LOOKUP_TABLE[0].distance) {
            LookUpTableEntry entry = LOOKUP_TABLE[0];
            return new LookUpTableTest(entry.hoodpiv, entry.rpm);
        }

        if (distance >= LOOKUP_TABLE[LOOKUP_TABLE.length - 1].distance) {
            LookUpTableEntry entry = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
            return new LookUpTableTest(entry.hoodpiv, entry.rpm);
        }

        // Find the two entries to interpolate between
        for (int i = 0; i < LOOKUP_TABLE.length - 1; i++) {
            LookUpTableEntry lower = LOOKUP_TABLE[i];
            LookUpTableEntry upper = LOOKUP_TABLE[i + 1];

            if (distance >= lower.distance && distance <= upper.distance) {
                // Linear interpolation
                double t = (distance - lower.distance) / (upper.distance - lower.distance);

                double interpolatedArmPiv = lerp(lower.hoodpiv, upper.hoodpiv, t);
                double interpolatedShooterRight = lerp(lower.rpm, upper.rpm, t);

                return new LookUpTableTest(interpolatedArmPiv, interpolatedShooterRight);
            }
        }

        // Fallback (should never reach here)
        LookUpTableEntry entry = LOOKUP_TABLE[LOOKUP_TABLE.length - 1];
        return new LookUpTableTest(entry.hoodpiv, entry.rpm);
    }

    /**
     * Linear interpolation helper function
     * 
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor (0.0 to 1.0)
     * @return Interpolated value
     */
    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }
}
