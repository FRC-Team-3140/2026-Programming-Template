package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class Config {
    public static final boolean useLinuxControlScheme = Robot.isSimulation();
  }

  public static final class MotorIDs {
    public static final int frontLeftTurn = 1;
    public static final int frontLeftDrive = 2;
    public static final int frontRightTurn = 3;
    public static final int frontRightDrive = 4;
    public static final int backLeftTurn = 5;
    public static final int backLeftDrive = 6;
    public static final int backRightTurn = 7;
    public static final int backRightDrive = 8;
  }

  public static final class SensorIDs {
    public static final int frontLeftTurnEncoder = 2;
    public static final int frontRightTurnEncoder = 0;
    public static final int backLeftTurnEncoder = 3;
    public static final int backRightTurnEncoder = 1;
  }

  public static final class SwerveDrive {
    public static final class Offsets {
      public static final class Translation {
        // TODO: Update these to match the real Swerve Drive offsets
        public static final Translation2d frontLeft = new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(12));
        public static final Translation2d frontRight = new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(-12));
        public static final Translation2d backLeft = new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(12));
        public static final Translation2d backRight = new Translation2d(Units.inchesToMeters(-12), Units.inchesToMeters(-12));
      }
      public static final class Rotation {
        // TODO: Update these angles to match the swerve drive's zeroed angle
        public static final double frontLeft = 166.5;
        public static final double frontRight = 109.3;
        public static final double backLeft = 254.4;
        public static final double backRight = 249;
      }
    }
    public static final class Ratios {
      public static final double driveRatio = 1 / 6.75 / 1.14; // L2 swerve + pinion conversion kit
      public static final double steerGearRatio = 150 / 7;
    }
    public static final double maxSpeed = Units.feetToMeters(16); // m / s
    public static final double maxRot = 10; // rads / sec

    public static class PathPlanner {
      public static final PIDConstants transPID = new PIDConstants(5, 0, 0);
      public static final PIDConstants rotPID = new PIDConstants(5, 0, 0);

      public static final RobotConfig config;
      static {
        try {
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          System.err.println(e.getMessage());
          throw new RuntimeException("Failed to load RobotConfig from GUI settings", e);
        }
      };
    }
  }

  public static final class CurrentLimits {
    public static final class SwerveDrive {
      public static final int driveMotorCurrentLimit = 40;
      public static final int turnMotorCurrentLimit = 20; 
    }
  }
}
