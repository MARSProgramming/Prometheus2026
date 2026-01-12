package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DrivetrainTelemetry extends SubsystemBase {
    private CommandSwerveDrivetrain dt;

    public DrivetrainTelemetry(CommandSwerveDrivetrain thedrivetrain)   {
        dt = thedrivetrain;
    }

 

    

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
  .getStructTopic("AdvantageKitPose", Pose2d.struct).publish();



    @Override
    public void periodic() {

      publisher.set(dt.getState().Pose);
      
    }
}
