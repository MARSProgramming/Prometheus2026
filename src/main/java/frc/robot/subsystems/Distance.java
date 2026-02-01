package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LidarFilter;

public class Distance extends SubsystemBase {
    private CANrange s_distance;
    LidarFilter filter;

    public Distance() {
        filter = new LidarFilter(1);

        s_distance = new CANrange(25);
    }

    public double getDistance() {
        return s_distance.getDistance().getValueAsDouble();
    }

    public double getFilteredDistance() {
        return filter.getFilteredDistance();
    }

    @Override
    public void periodic() {
        double distance = getDistance();
        filter.update(distance);
        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Distance-Smoothed", getFilteredDistance());
    }
}
