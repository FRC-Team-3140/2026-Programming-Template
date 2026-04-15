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
  public SwerveDriveKinematics kinematics;

  private SwerveDrivePoseEstimator poseEstimator;
  public static NavX swerveGyro; 

  public static boolean fieldRelative = true;

  public SwerveModule[] swerveModules = new SwerveModule[] {
    new SwerveModule(Constants.MotorIDs.frontLeftDrive, Constants.MotorIDs.frontLeftTurn, Constants.SensorIDs.frontLeftTurnEncoder, false, Constants.SwerveDrive.Offsets.Rotation.frontLeft),
        new SwerveModule(Constants.MotorIDs.frontRightDrive, Constants.MotorIDs.frontRightTurn, Constants.SensorIDs.frontRightTurnEncoder, false, Constants.SwerveDrive.Offsets.Rotation.frontRight),
        new SwerveModule(Constants.MotorIDs.backLeftDrive, Constants.MotorIDs.backLeftTurn, Constants.SensorIDs.backLeftTurnEncoder, false, Constants.SwerveDrive.Offsets.Rotation.backLeft),
        new SwerveModule(Constants.MotorIDs.backRightDrive, Constants.MotorIDs.backRightTurn, Constants.SensorIDs.backRightTurnEncoder, false, Constants.SwerveDrive.Offsets.Rotation.backRight),

  };


  public SwerveDrive() {
    kinematics = new SwerveDriveKinematics(Constants.SwerveDrive.Offsets.Translation.frontLeft, 
        Constants.SwerveDrive.Offsets.Translation.frontRight, 
        Constants.SwerveDrive.Offsets.Translation.backLeft, 
        Constants.SwerveDrive.Offsets.Translation.backRight);
    swerveGyro = new NavX(NavXComType.kMXP_SPI);
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, swerveGyro.getRotation2d(), 
        new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() }, 
        new Pose2d(0.0, 0.0, new Rotation2d(0.0)));
  }

  @Override
  public void periodic() {
    poseEstimator.update(swerveGyro.getRotation2d(), getCurrentSwerveModulePositions());
  }

  public SwerveModulePosition[] getCurrentSwerveModulePositions() {
    return new SwerveModulePosition[]{
      new SwerveModulePosition(swerveModules[0].getDistance(), swerveModules[0].getAngle()), // Front-Left
          new SwerveModulePosition(swerveModules[1].getDistance(), swerveModules[1].getAngle()), // Front-Right
          new SwerveModulePosition(swerveModules[2].getDistance(), swerveModules[2].getAngle()), // Back-Left
          new SwerveModulePosition(swerveModules[3].getDistance(), swerveModules[3].getAngle())  // Back-Right
    };
  }

  public void setSpeeds(ChassisSpeeds speeds) {
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrive.maxSpeed);
    setStates(states);
  }

  public void setStates(SwerveModuleState[] states) {
    swerveModules[0].setState(states[0]);
    swerveModules[1].setState(states[1]);
    swerveModules[2].setState(states[2]);
    swerveModules[3].setState(states[3]);
  }

  @AutoLogOutput(key = "SwerveStates")
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
      new SwerveModuleState(swerveModules[0].getVelocity(), swerveModules[0].getAngle()), // Front-Left
          new SwerveModuleState(swerveModules[1].getVelocity(), swerveModules[1].getAngle()), // Front-Right
          new SwerveModuleState(swerveModules[2].getVelocity(), swerveModules[2].getAngle()), // Back-Left
          new SwerveModuleState(swerveModules[3].getVelocity(), swerveModules[3].getAngle())  // Back-Right
    };
  }

  public static Command controllerDriveCommand() {
      return RobotContainer.swerveDrive.run(()->{
        if(SwerveDrive.fieldRelative) {
          RobotContainer.swerveDrive.setSpeeds(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                ControllerSubsystem.primaryController.getLeftY() * Constants.SwerveDrive.maxSpeed, 
                ControllerSubsystem.primaryController.getLeftX() * Constants.SwerveDrive.maxSpeed, 
                -ControllerSubsystem.primaryController.getRightX() * Constants.SwerveDrive.maxRot, swerveGyro.getRotation2d()));
        } else {
          RobotContainer.swerveDrive.setSpeeds(
              new ChassisSpeeds(
                ControllerSubsystem.primaryController.getLeftY() * Constants.SwerveDrive.maxSpeed, 
                ControllerSubsystem.primaryController.getLeftX() * Constants.SwerveDrive.maxSpeed, 
                -ControllerSubsystem.primaryController.getRightX() * Constants.SwerveDrive.maxRot));
        }

      });
  }
  public static Command driveCommmand(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (fieldRelative) {
      return RobotContainer.swerveDrive.run(()->{
        RobotContainer.swerveDrive.setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, swerveGyro.getRotation2d()));
      });
    }
    return RobotContainer.swerveDrive.run(()->{
      RobotContainer.swerveDrive.setSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rot));
    });
  }

  @AutoLogOutput(key = "SwervePose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }
}
