package frc.robot.subsystems.swervedrive;

import java.awt.Robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.simulation.AnalogEncoderSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SwerveModule extends SubsystemBase {

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
