package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class ControllerSubsystem extends SubsystemBase {
  public static final Controller primaryController = new Controller(0);
  public static final Controller secondaryController = new Controller(1);

  public ControllerSubsystem() {}


  @Override
  public void periodic() {}

  enum AxisMapping {
    LEFT_X(Constants.Config.useLinuxControlScheme ? 0 : 0),
    RIGHT_X(Constants.Config.useLinuxControlScheme ? 3 : 4),
    LEFT_Y(Constants.Config.useLinuxControlScheme ? 1 : 1),
    RIGHT_Y(Constants.Config.useLinuxControlScheme ? 4 : 5),
    LEFT_TRIGGER(Constants.Config.useLinuxControlScheme ? 2 : 2),
    RIGHT_TRIGGER(Constants.Config.useLinuxControlScheme ? 5 : 3);

    public int axisValue;

    AxisMapping(int axisValue) {
      this.axisValue = axisValue;
    }
  }

  public static class Controller extends XboxController {
    private double triggerDeadband = 0.5;
    private double joystickDeadband = 0.1;

    public Controller(int port) {
      super(port);
    }

    @Override
    public double getLeftX() {
      double input = getRawAxis(AxisMapping.LEFT_X.axisValue); 
      return Math.abs(input) > joystickDeadband ? input * input * Math.signum(input) : 0.0; // Apply deadband
    }


    @Override
    public double getLeftY() {
      double input = getRawAxis(AxisMapping.LEFT_Y.axisValue);
      return Math.abs(input) > joystickDeadband ? input * input * Math.signum(input) : 0.0;
    }

    @Override
    public double getRightX() {
      double input = getRawAxis(AxisMapping.RIGHT_X.axisValue);
      return Math.abs(input) > joystickDeadband ? input * input * Math.signum(input) : 0.0;
    }

    @Override
    public double getRightY() {
      double input = getRawAxis(AxisMapping.RIGHT_Y.axisValue);
      return Math.abs(input) > joystickDeadband ? input * input * Math.signum(input) : 0.0;
    }

    public double getLeftTrigger() {
      return getRawAxis(AxisMapping.LEFT_TRIGGER.axisValue);
    }

    public double getRightTrigger() {
      return getRawAxis(AxisMapping.RIGHT_TRIGGER.axisValue);
    }

    public boolean leftTriggered() {
      return getLeftTrigger() > triggerDeadband;
    }

    public boolean rightTriggered() {
      return getRightTrigger() > triggerDeadband;
    }
  }
}
