package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.XboxController;

public class ControllerSubsystem extends SubsystemBase {
  public static final Controller primaryController = new Controller(0);
  public static final Controller secondaryController = new Controller(1);

  public static RobotMode currentMode = RobotMode.AUTO;

  public ControllerSubsystem() {}



  // TODO: Update control scheme for the robot here
  private static void universalControls() {
  
  }

  static class ManualModeScheme implements ControlScheme {
    public void init() {
      System.out.println("Entering Manual Mode");
    }

    public void periodic() {

    }

    public void end() {
      System.out.println("Exiting Manual Mode");
    }
  }

  static class AutoModeScheme implements ControlScheme {
    public void init() {
      System.out.println("Entering Auto Mode");
    }

    public void periodic() {

    }

    public void end() {
      System.out.println("Exiting Auto Mode");
    }
  }

  public enum RobotMode {
    MANUAL(new ManualModeScheme()),
    AUTO(new AutoModeScheme());

    Runnable init;
    Runnable periodic;
    Runnable end;

    RobotMode(ControlScheme controlScheme) {
      this.init = controlScheme::init;
      this.periodic = controlScheme::periodic;
      this.end = controlScheme::end;
    }

    void init() { init.run(); }
    void execute() { periodic.run(); }
    void end() { end.run(); }
  }

  private void checkModeSwitch() {
    if (!(secondaryController.getLeftStickButton() && secondaryController.getRightStickButton())) return;

    setMode(secondaryController.getRightBumperButton() ? RobotMode.MANUAL: currentMode);
    setMode(secondaryController.getLeftBumperButton() ? RobotMode.AUTO : currentMode);

  }

  public void setMode(RobotMode mode) {
    if (mode == currentMode) return;
    currentMode = mode;

  }

  interface ControlScheme {
    public default void init() {};
    public default void periodic() {};
    public default void end() {};
  }


  @Override
  public void periodic() {
    checkModeSwitch();

    currentMode.execute();

    universalControls();
  }

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
    private double joystickDeadband = 0.15;

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
