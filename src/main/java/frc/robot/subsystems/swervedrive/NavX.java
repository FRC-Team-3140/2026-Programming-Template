package frc.robot.subsystems.swervedrive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class NavX extends AHRS {
  // This is the actual rotation of the robot in sim.
  public double realSimBotAngle = 0;

  // This is the angle the gyro thinks it is at during sim.
  // The reason these are different variables is that sometimes the gyro is reset or offset.
  // This makes the simulated gyro behave the same as a real robot.
  public double simAngle = 0;

  // This constructor just passes values to the AHRS
  public NavX(NavXComType comType) {
    super(comType);
  }

  // Only effects the function if it is called in sim so that it uses the sim angles
  @Override 
  public Rotation2d getRotation2d() {
    if(Robot.isReal()) return super.getRotation2d();
    return Rotation2d.fromDegrees(simAngle);
  }

  // Only effects the function if it is called in sim so that it uses the sim angles
  @Override 
  public double getAngle() {
    if(Robot.isReal()) return super.getAngle();
    return simAngle;
  }

  // The sim function that allows the gyro to be set to a value
  public void setSimAngle(double angle) {
    simAngle = angle;
  }

  // The sim function that allows the actual position to be referenced
  public double getRealSimAngle() {
    return realSimBotAngle;
  }

  // The sim function that allows the actual position to be referenced
  public Rotation2d getRealSimRotation2d() {
    return Rotation2d.fromDegrees(realSimBotAngle);
  }

  // The sim function that allows the actual position to be set, should only be done at the beginning of the robot
  public void setRealSimAngle(double angle) {
    realSimBotAngle = angle;
  }

  // Only effects the function if it is called in sim so that it uses the sim angles
  @Override
  public void reset() {
    if(Robot.isReal()) super.reset();
    simAngle = 0;
  }
}
