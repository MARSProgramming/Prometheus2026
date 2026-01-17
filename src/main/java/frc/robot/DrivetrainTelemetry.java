package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive;

public class DrivetrainTelemetry extends SubsystemBase {
    private Drive dt;
    StructPublisher<Pose2d> publisher;

    public DrivetrainTelemetry(Drive drivetrain)  {
        dt = drivetrain;
         publisher = NetworkTableInstance.getDefault()
.getStructTopic("AdvantageKitPose", Pose2d.struct).publish();


        
    }

 


    @Override
    public void periodic() {

       publisher.set(dt.getState().Pose);      
    }
}