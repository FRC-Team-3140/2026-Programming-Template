// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.SwerveDrive.SwerveModule;

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
    // This is the setup for AdvantageKit logging 
    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    // This creates the robot container
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
    RobotContainer.swerveDrive.setDefaultCommand(SwerveDrive.SwerveCommands.controllerDriveCommand());
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
    /*  Simulation works by updating each motor simulation every loop.
     *  The simulated motors update the real motor objects that are used with the real robot.
     *  This means that the code that runs on the real robot should behave the same in simulation without much extra code.
     */

    // This is the difference in time between simulation loops
    final double dt = 0.02;
    // This is the simulated voltage. For simplicity it is locked at 12
    final double vbus = 12.0;

    // motor free speeds (RPM)
    final double vortexFreeRPM = 6784.0; // drive (Spark Flex + NEO Vortex)
    final double neoFreeRPM = 5676.0; // steer (Spark MAX + NEO)

    // SwerveDrive Simulation, runs for each module
    for (SwerveModule module : RobotContainer.swerveDrive.swerveModules) {

      // --- DRIVE ---
      // Get the motor output 
      double driveDuty = module.driveMotorSim.getAppliedOutput();
      // Calculate RPM
      double driveRPM = driveDuty * vortexFreeRPM;
      // Update the motor sim
      module.driveMotorSim.iterate(driveRPM, vbus, dt);

      // --- STEER ---
      // Same for the turning motor
      double turnDuty = module.turnMotorSim.getAppliedOutput();
      double turnRPM = turnDuty * neoFreeRPM;
      module.turnMotorSim.iterate(turnRPM, vbus, dt);

      // Update the analog encoder 
      // Get the turns of the wheel based on the motor's rotation
      double wheelRotations = module.turnMotorSim.getPosition() / Constants.SwerveDrive.Ratios.steerGearRatio;
      // Clamp between 0 and 1
      wheelRotations = ((wheelRotations % 1) + 1) % 1;
      // Updating the encoder sim with the new value
      module.turnEncoderSim.set(wheelRotations);
    }

    // Gets the current speed of the robot
    ChassisSpeeds speeds = RobotContainer.swerveDrive.kinematics.toChassisSpeeds(RobotContainer.swerveDrive.getStates());
    // Calculates the rotation speed of the bot
    double omegaDegreesPerSecond = Math.toDegrees(speeds.omegaRadiansPerSecond);
    
    // Applies the rotation to the simulated and "real" gyro angles
    SwerveDrive.swerveGyro.setSimAngle(SwerveDrive.swerveGyro.getAngle() + omegaDegreesPerSecond * dt);
    SwerveDrive.swerveGyro.setRealSimAngle(SwerveDrive.swerveGyro.getRealSimAngle() + omegaDegreesPerSecond * dt);
  }
}
