package frc.robot.subsystems.swervedrive;

import org.littletonrobotics.junction.AutoLogOutput;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ControllerSubsystem;

public class SwerveDrive extends SubsystemBase {
  // WPILIB provided class for doing the swerve drive math
  public SwerveDriveKinematics kinematics;

  // WPILIB provided class that tracks the robot position
  // Pose is a class that stores position and rotation
  private SwerveDrivePoseEstimator poseEstimator;

  // Custom class that allows simulation of our gyro
  // When looking online, this is the same as AHRS. NavX inherits AHRS to overload the angle functions to store angles in sim.
  public static NavX swerveGyro; 

  // Changes drive to bot oriented or field oriented
  public static boolean fieldRelative = true;

  // Start Pose 
  private Pose2d startingPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

  // 4 Swerve modules that handle powering the motors based on the math from kinematics
  // TODO: If motors are driving backwards, change the booleans to invert the drive motors
  public SwerveModule[] swerveModules = new SwerveModule[] {
    new SwerveModule(Constants.MotorIDs.frontLeftDrive, Constants.MotorIDs.frontLeftTurn, Constants.SensorIDs.frontLeftTurnEncoder, false, Constants.SwerveDrive.Offsets.Rotation.frontLeft),
        new SwerveModule(Constants.MotorIDs.frontRightDrive, Constants.MotorIDs.frontRightTurn, Constants.SensorIDs.frontRightTurnEncoder, false, Constants.SwerveDrive.Offsets.Rotation.frontRight),
        new SwerveModule(Constants.MotorIDs.backLeftDrive, Constants.MotorIDs.backLeftTurn, Constants.SensorIDs.backLeftTurnEncoder, false, Constants.SwerveDrive.Offsets.Rotation.backLeft),
        new SwerveModule(Constants.MotorIDs.backRightDrive, Constants.MotorIDs.backRightTurn, Constants.SensorIDs.backRightTurnEncoder, false, Constants.SwerveDrive.Offsets.Rotation.backRight),
  };


  // This initlizes the swerve drive
  public SwerveDrive() {
    // This tells the kinematics class where the serve drives are on the robot
    kinematics = new SwerveDriveKinematics(Constants.SwerveDrive.Offsets.Translation.frontLeft, 
        Constants.SwerveDrive.Offsets.Translation.frontRight, 
        Constants.SwerveDrive.Offsets.Translation.backLeft, 
        Constants.SwerveDrive.Offsets.Translation.backRight);

    // This creates a gyro 
    swerveGyro = new NavX(NavXComType.kMXP_SPI);

    // Initializing the pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, swerveGyro.getRotation2d(), 
        new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() }, 
        startingPose);
  }

  @Override
  public void periodic() {
    // This takes the current swerve module states and calculates how it moves the robot
    poseEstimator.update(swerveGyro.getRotation2d(), getCurrentSwerveModulePositions());
  }

  // This converts the swerve module states into an array of SwerveModulePositions, this is useful for classes like the post estimator that take this as an argument
  public SwerveModulePosition[] getCurrentSwerveModulePositions() {
    return new SwerveModulePosition[]{
      new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()), // Front-Left
          new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()), // Front-Right
          new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()), // Back-Left
          new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())  // Back-Right
    };
  }

  // This takes velocities and drives the swerveDrive that way
  public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
    if(fieldRelative) {
      RobotContainer.swerveDrive.setSpeeds(
          ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed * Constants.SwerveDrive.maxSpeed, 
            ySpeed * Constants.SwerveDrive.maxSpeed, 
            rotSpeed * Constants.SwerveDrive.maxRot, swerveGyro.getRotation2d()));
    } else {
      RobotContainer.swerveDrive.setSpeeds(
          ChassisSpeeds.fromRobotRelativeSpeeds(
            xSpeed * Constants.SwerveDrive.maxSpeed, 
            ySpeed * Constants.SwerveDrive.maxSpeed, 
            rotSpeed * Constants.SwerveDrive.maxRot, swerveGyro.getRotation2d()));
    }
  }

  // This takes a chassisspeeds object and updates the kinimatics
  public void setSpeeds(ChassisSpeeds speeds) {

    // This accounts for the fact that each loop takes 20ms, and doesn't update continuously
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

    // The kinimatics take the speeds and calculates the swerve module states
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretizedSpeeds);

    // This prevents driving linearly from overpowering turning, letting the swerve turn even while driving max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrive.maxSpeed);

    // Sets the module states
    setStates(states);
  }

  // Takes the states and sets the swerve modules to that state
  public void setStates(SwerveModuleState[] states) {
    swerveModules[0].setState(states[0]);
    swerveModules[1].setState(states[1]);
    swerveModules[2].setState(states[2]);
    swerveModules[3].setState(states[3]);
  }

  // This is a feature of AdvanageKit that automatically logs the return value of this function to network tables. 
  // I use this to view the swerve states in advantagescope
  @AutoLogOutput(key = "SwerveStates")
  // This function just gets the state of the swerve modules in velocity and angle instead of position and angle. 
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(swerveModules[0].getVelocity(), swerveModules[0].getAngle()), // Front-Left
          new SwerveModuleState(swerveModules[1].getVelocity(), swerveModules[1].getAngle()), // Front-Right
          new SwerveModuleState(swerveModules[2].getVelocity(), swerveModules[2].getAngle()), // Back-Left
          new SwerveModuleState(swerveModules[3].getVelocity(), swerveModules[3].getAngle())  // Back-Right
    };
  }

  // This is the same advantagekit feature, but this allows me to see the robot's position in the networktables
  // This returns the pose that the robot thinks it is at
  @AutoLogOutput(key = "SwervePose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public static class SwerveCommands {
    // This command takes speeds and sets the swerveDrive states to move the bot in that way
    public static Command driveCommmand(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        return RobotContainer.swerveDrive.run(()->{ RobotContainer.swerveDrive.drive(xSpeed, ySpeed, rot, fieldRelative); });
    }

    // This command uses the controller inputs to drive the robot
    public static Command controllerDriveCommand() {
      return RobotContainer.swerveDrive.run(()-> {
        RobotContainer.swerveDrive.drive(
          ControllerSubsystem.primaryController.getLeftY(), 
          ControllerSubsystem.primaryController.getLeftX(),
          -ControllerSubsystem.primaryController.getRightX(),
          SwerveDrive.fieldRelative); 
      });
    }

  }
}
