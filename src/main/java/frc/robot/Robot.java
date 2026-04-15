// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ControllerSubsystem;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveModule;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    Logger.recordMetadata("ProjectName", "2026-Programming-Template"); // Set a metadata value
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RobotContainer.swerveDrive.setDefaultCommand(SwerveDrive.controllerDriveCommand());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    final double dt = 0.02;
    final double vbus = 12.0;

    // motor free speeds (RPM)
    final double vortexFreeRPM = 6784.0; // drive (Spark Flex + NEO Vortex)
    final double neoFreeRPM = 5676.0; // steer (Spark MAX + NEO)

    for (SwerveModule module : RobotContainer.swerveDrive.swerveModules) {

      // --- DRIVE ---
      double driveDuty = module.driveMotorSim.getAppliedOutput();
      double driveRPM = driveDuty * vortexFreeRPM;
      module.driveMotorSim.iterate(driveRPM, vbus, dt);

      // --- STEER ---
      double turnDuty = module.turnMotorSim.getAppliedOutput();
      double turnRPM = turnDuty * neoFreeRPM;
      module.turnMotorSim.iterate(turnRPM, vbus, dt);

      // update custom absolute encoder (convert motor rotations → wheel degrees)
      double wheelRotations = module.turnMotorSim.getPosition() / Constants.SwerveDrive.Ratios.steerGearRatio;
      double encoderTurns = wheelRotations;

      // normalize to [0, 1)
      encoderTurns = ((encoderTurns % 1) + 1) % 1;

      module.turnEncoderSim.set(encoderTurns);
    }

    SwerveModuleState[] positions = RobotContainer.swerveDrive.getStates();
    ChassisSpeeds speeds = RobotContainer.swerveDrive.kinematics.toChassisSpeeds(positions);
    double omegaDegreesPerSecond = Math.toDegrees(speeds.omegaRadiansPerSecond);
    SwerveDrive.swerveGyro.setSimAngle(SwerveDrive.swerveGyro.getAngle() + omegaDegreesPerSecond * dt);
  }
}
