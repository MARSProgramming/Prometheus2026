package frc.robot.util;

public class LidarFilter {
    // Smoothing factor in [0,1]. Higher = faster response, lower = smoother.
    private final double alpha;

    // Current filtered estimate
    private double estimate = 0.0;

    public LidarFilter() {
        this(0.2);
    }

    /**
     * @param alpha smoothing factor in range (0,1]
     */
    public LidarFilter(double alpha) {
        if (alpha <= 0 || alpha > 1) {
            throw new IllegalArgumentException("alpha must be in (0,1]");
        }

        this.alpha = alpha;
    }

    public void update(double lidarDistance) {
        estimate = alpha * lidarDistance + (1.0 - alpha) * estimate;
    }

    public double getFilteredDistance() {
        return estimate;
    }
}