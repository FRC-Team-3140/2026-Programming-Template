// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.ControllerSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Creating subsystems
  public static final ControllerSubsystem ControllerSubsystem = new ControllerSubsystem();
  public static final SwerveDrive swerveDrive = new SwerveDrive();

  public RobotContainer() {
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new PrintCommand("Placeholder Auto");
  }
}
