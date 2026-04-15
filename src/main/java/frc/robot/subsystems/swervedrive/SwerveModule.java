package frc.robot.subsystems.swervedrive;

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

public class SwerveModule extends SubsystemBase {
  private static final double STEER_IDLE_SPEED_THRESHOLD_MPS = 0.05;

  private SparkFlex driveMotor;
  public SparkFlexSim driveMotorSim;

  private SparkMax turnMotor;
  public SparkMaxSim turnMotorSim;

  private double angleOffset = 0;

  public AnalogEncoder turnEncoder;
  public AnalogEncoderSim turnEncoderSim;

  private PIDController turningPIDController = new PIDController(0.03, 0, 0.0002);

  private final double metersPerMotorRotation =
      2 * Math.PI * Units.inchesToMeters(2) * Constants.SwerveDrive.Ratios.driveRatio;
  private final double motorRotationsPerMinutePerMetersPerSecond = 60.0 / metersPerMotorRotation;
  private Rotation2d lastTargetAngle = new Rotation2d();
  private boolean angleSetpointInitialized = false;

  public SwerveModule(int driveMotorID, int turnMotorID, int turnEncoderID, boolean driveMotorInverted, double angleOffset) {
    driveMotor = new SparkFlex(driveMotorID, SparkFlex.MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorID, SparkMax.MotorType.kBrushless);

    turnEncoder = new AnalogEncoder(turnEncoderID);

    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);

    config.inverted(false).smartCurrentLimit(Constants.CurrentLimits.SwerveDrive.turnMotorCurrentLimit);
    turnMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    config.closedLoop.p(0.00008).i(0).d(0).feedForward.sva(0.1, 0.0014, 0);
    config.inverted(driveMotorInverted).smartCurrentLimit(Constants.CurrentLimits.SwerveDrive.driveMotorCurrentLimit);

    // 2 * pi * r (4 inch wheels)
    // * drive ratio
    config.encoder.positionConversionFactor(metersPerMotorRotation);
    driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    this.angleOffset = angleOffset;

    driveMotorSim = new SparkFlexSim(driveMotor, DCMotor.getNeoVortex(1));
    turnMotorSim = new SparkMaxSim(turnMotor, DCMotor.getNEO(1));
    turnEncoderSim = new AnalogEncoderSim(turnEncoder);

    turningPIDController.enableContinuousInput(-180, 180);
    turningPIDController.setTolerance(1.5);
  }

  public double getDistance() {
    return driveMotor.getEncoder().getPosition(); 
  }

  public double getVelocity() {
    return driveMotor.getEncoder().getVelocity() * metersPerMotorRotation / 60.0;
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(turnEncoder.get() * 360 - angleOffset);
  }

  public void setState(SwerveModuleState state) {
    state.optimize(getAngle());
    SwerveModuleState optimizedState = state;

    if (!angleSetpointInitialized) {
      lastTargetAngle = getAngle();
      angleSetpointInitialized = true;
    }

    double speedMetersPerSecond = optimizedState.speedMetersPerSecond;
    if (Math.abs(speedMetersPerSecond) < 0.01) {
      speedMetersPerSecond = 0.0;
    }

    driveMotor.getClosedLoopController().setSetpoint(
        speedMetersPerSecond * motorRotationsPerMinutePerMetersPerSecond,
        SparkFlex.ControlType.kVelocity);

    if (Math.abs(speedMetersPerSecond) > STEER_IDLE_SPEED_THRESHOLD_MPS) {
      lastTargetAngle = optimizedState.angle;
    }
    turningPIDController.setSetpoint(lastTargetAngle.getDegrees());
  }

  @Override
  public void periodic() {
    double currentAngle = MathUtil.inputModulus(getAngle().getDegrees(), -180.0, 180.0);
    double turnOutput = turningPIDController.calculate(currentAngle);
    if (turningPIDController.atSetpoint()) {
      turnOutput = 0.0;
    }
    turnMotor.set(MathUtil.clamp(turnOutput, -1.0, 1.0));
  }
}
