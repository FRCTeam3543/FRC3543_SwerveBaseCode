package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static enum RobotMode {
    AUTONOMOUS,
    TELEOP,
    DISABLED
  }

  public static boolean enableTunableValues = true;

  public static final class driverControllers {
    public static final int driver = 0;
    public static final int operator = 1;
  }

  public static final class SwerveConstants {

    public static enum DriveMode {
      DriverInput,
      Snap,
      AutonomousSnap,
    }

    public static final double[] snapPID = { 1, 0, 0 };
    public static final double[] snapSVA = { 0.015, 0.2, 0 };

    /* Drive Controls */
    public static final double stickDeadband = 0.1;

    /* Gyro */
    public static final SPI.Port GYRO_PORT = SPI.Port.kOnboardCS0;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.75);
    public static final double wheelBase = Units.inchesToMeters(21.75);
    public static final double wheelDiameter = Units.inchesToMeters(4.0); 
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = 150.0 / 7.0; // 150/7:1

    /* Kinematics */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.2; // meters per second
    public static final double maxAngularVelocity = Math.PI * 2; // rads per second

  
    public static final String[] moduleNames = { "Front Left", "Front Right", "Back Left", "Back Right" }; 
    // #1, #2, #3

    public static final double[] driveSVA = new double[] { 0.3, 2.5, 0.0 };
    public static final double[] drivePID = new double[] { 0.15, 0.0000, 0.00010 };
    public static final double[] anglePID = new double[] { 0.02, 0.0, 0.0005 };

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 0;
      public static final double angleOffset = 0;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 1;
      public static final double angleOffset = 0 ;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 2;
      public static final double angleOffset = 0;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 3;
      public static final double angleOffset = 0;
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset, anglePID, drivePID, driveSVA);
    }
  }

  public static final class AutoConstants {
    // public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxSpeedMetersPerSecond = 2;

    // public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI * 5;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI * 4;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

}