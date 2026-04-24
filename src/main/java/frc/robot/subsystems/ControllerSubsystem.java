package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import edu.wpi.first.wpilibj.XboxController;

public class ControllerSubsystem extends SubsystemBase {
  private static final Controller primaryController = new Controller(0);
  private static final Controller secondaryController = new Controller(1);

  public static RobotMode currentMode = RobotMode.AUTO;


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

      if(primaryController.getXButtonPressed()) {
        SwerveDrive.fieldRelative = !SwerveDrive.fieldRelative;
      }
    }

    public void end() {
      System.out.println("Exiting Auto Mode");
    }
  }

  public static class DefaultCommands {
    public static Command getSwerveDriveCommand() {
      return RobotContainer.swerveDrive.run(()->{
        RobotContainer.swerveDrive.drive(
            primaryController.getLeftY(), 
            primaryController.getLeftX(),
            -primaryController.getRightX(),
            SwerveDrive.fieldRelative); 
      });
    }
  }

  interface ControlScheme {
    public default void init() {};
    public default void periodic() {};
    public default void end() {};
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

  /// Methods 
  private void checkModeSwitch() {
    if (!(secondaryController.getLeftStickButton() && secondaryController.getRightStickButton())) return;

    currentMode.end();

    setMode(secondaryController.getRightBumperButton() ? RobotMode.MANUAL: currentMode);
    setMode(secondaryController.getLeftBumperButton() ? RobotMode.AUTO : currentMode);

    currentMode.init();
  }

  public void setMode(RobotMode mode) {
    if (mode == currentMode) return;
    currentMode = mode;

  }



  /// Subsystem Methods

  public ControllerSubsystem() {}

  @Override
  public void periodic() {
    checkModeSwitch();

    currentMode.execute();

    universalControls();
  }

  /// Controller Mapping

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
