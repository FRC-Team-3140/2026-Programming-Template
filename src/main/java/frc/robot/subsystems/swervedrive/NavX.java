package frc.robot.subsystems.swervedrive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;

public class NavX extends AHRS {
  public double realSimBotAngle = 0;
  public double simAngle = 0;

  public NavX(NavXComType comType) {
    super(comType);
  }

  @Override 
  public Rotation2d getRotation2d() {
    if(Robot.isReal()) return super.getRotation2d();
    return Rotation2d.fromDegrees(simAngle);
  }
  @Override 
  public double getAngle() {
    if(Robot.isReal()) return super.getAngle();
    return simAngle;
  }

  public void setSimAngle(double angle) {
    simAngle = angle;
  }

  public double getRealSimAngle() {
    return realSimBotAngle;
  }

  public Rotation2d getRealSimRotation2d() {
    return Rotation2d.fromDegrees(realSimBotAngle);
  }

  public void setRealSimAngle(double angle) {
    realSimBotAngle = angle;
  }

  @Override
  public void reset() {
    if(Robot.isReal()) super.reset();
    simAngle = 0;
  }
}
