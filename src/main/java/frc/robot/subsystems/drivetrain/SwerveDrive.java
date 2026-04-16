package frc.robot.subsystems.drivetrain;

import org.littletonrobotics.junction.AutoLogOutput;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ControllerSubsystem;

/* This class contains:
 *  SwerveDrive code
 *  SwerveDrive commands
 *  Simulated bot code
 *  SwerveModule code
 *  Localization/Odometry code
 */

public class SwerveDrive extends SubsystemBase {
  // WPILIB provided class for doing the swerve drive math
  public SwerveDriveKinematics kinematics;

  // WPILIB provided class that tracks the robot position
  // Pose is a class that stores position and rotation
  private SwerveDrivePoseEstimator poseEstimator;


  // This will hold the "real" position of the bot. Unline the poseEstimator, this won't be updated by vision or reset after init
  private SwerveDrivePoseEstimator simPoseEstimator;

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
    // Set the gyro to the correct "real" rotation 
    swerveGyro.setRealSimAngle(startingPose.getRotation().getDegrees());

    // Initializing the pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, swerveGyro.getRotation2d(), 
        new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() }, 
        Pose2d.kZero);

    // Initializing the pose estimator
    simPoseEstimator = new SwerveDrivePoseEstimator(kinematics, swerveGyro.getRealSimRotation2d(), 
        new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() }, 
        startingPose);
  }

  @Override
  public void periodic() {
    // This takes the current swerve module states and calculates how it moves the robot
    poseEstimator.update(swerveGyro.getRotation2d(), getCurrentSwerveModulePositions());
    // This takes the "real" robot and drives it
    simPoseEstimator.update(swerveGyro.getRealSimRotation2d(), getCurrentSwerveModulePositions());
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

  public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDev) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp, stdDev);
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
  @AutoLogOutput(key = "EstimatedPose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  // Same thing, but for the sim robot
  @AutoLogOutput(key = "SimPose")
  public Pose2d getSimPose() {
    return simPoseEstimator.getEstimatedPosition();
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

  // The Swerve Module class
  public static class SwerveModule extends SubsystemBase {

    // This is the drive vortext motor
    private SparkFlex driveMotor;
    // This is a simulated motor that allows the swerve to work in sim
    public SparkFlexSim driveMotorSim;

    // This is the turning neo motor
    private SparkMax turnMotor;

    // This is a simulated motor that allows the swerve to work in sim
    public SparkMaxSim turnMotorSim;

    // This is the offset of the turning encoder
    private double angleOffset;

    // This is the encoder on the top of the swerve drive
    public AnalogEncoder turnEncoder;
    // This simulates the encoder
    public AnalogEncoderSim turnEncoderSim;

    // This PID calculats how to drive the turning motor to get to the angle setpoint
    private PIDController turningPIDController = new PIDController(0.01, 0, 0.0002);

    // These are conversion factors for different ratios
    private final double metersPerMotorRotation =
      2 * Math.PI * Units.inchesToMeters(2) * Constants.SwerveDrive.Ratios.driveRatio;
    private final double motorRotationsPerMinutePerMetersPerSecond = 60.0 / metersPerMotorRotation;


    public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID, boolean driveMotorInverted, double angleOffset) {
      // This creates the real motors
      driveMotor = new SparkFlex(driveMotorID, SparkFlex.MotorType.kBrushless);
      turnMotor = new SparkMax(turnMotorID, SparkMax.MotorType.kBrushless);

      // This creates the real encoder
      turnEncoder = new AnalogEncoder(turnEncoderID);

      // To configure the motors, we first create a config object
      SparkFlexConfig config = new SparkFlexConfig();

      // We then configure the object to have different configurations
      // Both motors should be in break mode, so they stop when the bot is disabled.
      config.idleMode(IdleMode.kBrake);

      // This is the config for the  turn motor. It shouldn't be inverted, and it has a configurable current limit defined in constants
      config.inverted(false).smartCurrentLimit(Constants.CurrentLimits.SwerveDrive.turnMotorCurrentLimit);
      // This applies the config to the turn motor
      turnMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


      // The drive motor uses the internal encoder, so we can use the PID controller that is built into the sparkmax
      // This is more accurate than an onboard PID controller, as it updates 1000x a second, the rio only updates 50 times a second.
      // There is also a feedForward that helps overcome static friction
      config.closedLoop.p(0.00008).i(0).d(0).feedForward.sva(0.0, 0.0014, 0);
      // This is the config for the drive motor. It may be inverted, and it has a configurable current limit defined in constants
      config.inverted(driveMotorInverted).smartCurrentLimit(Constants.CurrentLimits.SwerveDrive.driveMotorCurrentLimit);
      // The internal encoder is updated with the conversion factor, so all reads of the encoder's position result in linear meters that the wheel would travel.
      config.encoder.positionConversionFactor(metersPerMotorRotation);
      // This applies the config to the drive motor
      driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

      this.angleOffset = angleOffset;

      // The motors and encoder sims are created to control the real motors when in simulation
      // This means we can write all of the code to use the real motors, and the sim motors will act as hardware
      driveMotorSim = new SparkFlexSim(driveMotor, DCMotor.getNeoVortex(1));
      turnMotorSim = new SparkMaxSim(turnMotor, DCMotor.getNEO(1));
      turnEncoderSim = new AnalogEncoderSim(turnEncoder);

      // This lets the module's PID controlelr wrap around
      turningPIDController.enableContinuousInput(-180, 180);
    }

    // This gets the distance traveled by the wheel, using the drive motor's built in encoder
    public double getDistance() {
      return driveMotor.getEncoder().getPosition(); 
    }

    // This gets the velocity of the wheel
    public double getVelocity() {
      return driveMotor.getEncoder().getVelocity() * metersPerMotorRotation / 60.0;
    }

    // This gets the rotation of the module
    public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(turnEncoder.get() * 360 - angleOffset);
    }

    // This takes in a SwerveModuleState and updates the PID setpoints
    public void setState(SwerveModuleState state) {
      // This optimizes the state of the wheel. 
      // For example, when reversing driving direction, instead of driving the wheels in one direction and rotating the wheels 180 degress, it just flips the driving direction of the wheels
      // This prevents unneed angle turns and makes the swervedrive act more naturally
      state.optimize(getAngle());

      // This sets the built in PID and FeedFoward setpoint to the correct speed
      driveMotor.getClosedLoopController().setSetpoint(
          state.speedMetersPerSecond * motorRotationsPerMinutePerMetersPerSecond,
          SparkFlex.ControlType.kVelocity);

      // This sets the rotation setpoint of the wheel
      turningPIDController.setSetpoint(state.angle.getDegrees());
    }

    @Override
    public void periodic() {
      // This updates the turn motor with the PID controller's output
      turnMotor.set(turningPIDController.calculate(getAngle().getDegrees()));
    }
  }
}
