// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.DrivetrainTelemetry;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    //ivate final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController pilot = new CommandXboxController(0);
    Swerve swervebase = new Swerve();
    DrivetrainTelemetry dttel = new DrivetrainTelemetry(swervebase);
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    private void configureBindings() {  

        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swervebase, 
            () -> -pilot.getLeftY(), 
            () -> -pilot.getLeftX(), 
            () -> -pilot.getRightX()
        );

        swervebase.setDefaultCommand(manualDriveCommand); // Handles teleoperated driving
        pilot.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric())); // Re-seeds field-centric heading when 'back' button is pressed


    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        return autoChooser.getSelected();
    }
    /**
     * Applies a deadband to inputs.
     * @param value The input value to apply the deadband to.
     * @param deadband The size of the deadband.
     */


}
