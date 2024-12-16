package frc.robot.subsystems.Drive;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.util.LoggedTunableNumber;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants.DriveMode;

public class Swerve extends SubsystemBase {
  private SwerveModule[] mSwerveMods;
  private ADXRS450_Gyro gyro;


  private final SwerveDrivePoseEstimator poseEstimator;

  private final PIDController snapPIDController;
  private SimpleMotorFeedforward snapFFModel;
  private DriveMode driveMode = DriveMode.DriverInput;
  private Rotation2d snapSetpoint = new Rotation2d();
  private Rotation2d snapVelocity = new Rotation2d();
  private Rotation2d snapGoal = new Rotation2d();

  private Field2d debugField2d = new Field2d();
  private Pose2d lastActivePathPose = new Pose2d();

  private Translation2d robotSpeed = new Translation2d();

  // Tunable values
  private LoggedTunableNumber driveP = new LoggedTunableNumber("driveP", Constants.SwerveConstants.drivePID[0]);
  private LoggedTunableNumber driveI = new LoggedTunableNumber("driveI", Constants.SwerveConstants.drivePID[1]);
  private LoggedTunableNumber driveD = new LoggedTunableNumber("driveD", Constants.SwerveConstants.drivePID[2]);

  private LoggedTunableNumber driveS = new LoggedTunableNumber("driveS", Constants.SwerveConstants.driveSVA[0]);
  private LoggedTunableNumber driveV = new LoggedTunableNumber("driveV", Constants.SwerveConstants.driveSVA[1]);
  private LoggedTunableNumber driveA = new LoggedTunableNumber("driveA", Constants.SwerveConstants.driveSVA[2]);

  private LoggedTunableNumber angleP = new LoggedTunableNumber("angleP", Constants.SwerveConstants.anglePID[0]);
  private LoggedTunableNumber angleI = new LoggedTunableNumber("angleI", Constants.SwerveConstants.anglePID[1]);
  private LoggedTunableNumber angleD = new LoggedTunableNumber("angleD", Constants.SwerveConstants.anglePID[2]);


  public Swerve() {
    /* Gyro setup */
  gyro = new ADXRS450_Gyro();

    /* Swerve modules setup */
    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
        new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
        new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
        new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
    };

    poseEstimator = new SwerveDrivePoseEstimator(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions(),
        new Pose2d());

    snapPIDController = new PIDController(
        Constants.SwerveConstants.snapPID[0],
        Constants.SwerveConstants.snapPID[1],
        Constants.SwerveConstants.snapPID[2]);
    snapFFModel = new SimpleMotorFeedforward(
        Constants.SwerveConstants.snapSVA[0],
        Constants.SwerveConstants.snapSVA[1],
        Constants.SwerveConstants.snapSVA[2]);

    snapPIDController.setTolerance(Units.degreesToRadians(5));
    snapPIDController.enableContinuousInput(0, Math.PI * 2);
    snapPIDController.setIntegratorRange(-0.1, 0.1);

  }

  public Optional<Rotation2d> getRotationTargetOverride() {
    if (driveMode == DriveMode.Snap) {
      return Optional.of(snapGoal);
    } else {
      return Optional.empty();
    }
  }

  public void setSnapSetpoint(Rotation2d setpoint, Rotation2d velocity) {
    this.snapSetpoint = setpoint;
    this.snapVelocity = velocity;
  }

  public void setSnapGoal(Rotation2d goal) {
    this.snapGoal = goal;
  }

  public boolean isSnapAtGoal() {
    double errorBound = (Math.PI * 2) / 2.0;
    var m_positionError = MathUtil.inputModulus(snapGoal.getRadians() - getYawForSnap().getRadians(), -errorBound,
        errorBound);
    return Math.abs(m_positionError) < Units.degreesToRadians(5);
  }

  public boolean isSnapAtSetpoint() {
    return snapPIDController.atSetpoint();
  }

  public Rotation2d getYawForSnap() {
    double yawRad = getYaw().getRadians();
    yawRad = yawRad % (Math.PI * 2);
    if (yawRad < 0) {
      yawRad = Math.PI * 2 + yawRad;
    }

    return new Rotation2d(yawRad);
  }

  public double calculateSnapOutput(Rotation2d setpoint, Rotation2d snapVelocity) {
    double ffOutput = -snapFFModel.calculate(snapVelocity.getRadians());
    double rotation = -snapPIDController.calculate(getYawForSnap().getRadians(), snapSetpoint.getRadians());
    rotation = MathUtil.clamp(rotation, -1, 1);
    rotation /= 2;
    return rotation + ffOutput;
  }

  /**
   * The main function used for driving the robot
   * 
   * @param translation
   * @param rotation
   */
  public void drive(Translation2d translation, double rotation) {
    switch (driveMode) {
      case Snap:
        rotation = calculateSnapOutput(snapSetpoint, snapVelocity);
        break;
      case AutonomousSnap:
        rotation = calculateSnapOutput(snapSetpoint, snapVelocity);
        break;
      default:
        break;
    }
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX() * Constants.SwerveConstants.maxSpeed,
            translation.getY() * Constants.SwerveConstants.maxSpeed,
            rotation * Constants.SwerveConstants.maxAngularVelocity, getYaw()));

    robotSpeed = new Translation2d(translation.getX() * Constants.SwerveConstants.maxSpeed,
        translation.getY() * Constants.SwerveConstants.maxSpeed);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    setStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    double rotation = targetSpeeds.omegaRadiansPerSecond;
    targetSpeeds = new ChassisSpeeds(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond,
        -rotation);

    var currentSpeed = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds.vxMetersPerSecond,
        robotRelativeSpeeds.vyMetersPerSecond,
        robotRelativeSpeeds.omegaRadiansPerSecond, poseEstimator.getEstimatedPosition().getRotation());
    robotSpeed = new Translation2d(currentSpeed.vxMetersPerSecond, currentSpeed.vyMetersPerSecond);

    SwerveModuleState[] targetStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetSpeeds);
    setStates(targetStates);
  }

  public void setStates(SwerveModuleState[] states) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(states[mod.moduleNumber]);
    }
  }

  public void setDriveMode(DriveMode mode) {
    driveMode = mode;
  }

  public DriveMode getDriveMode() {
    return driveMode;
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber]);
    }
  }

  public void resetSnapI() {
    snapPIDController.reset();
  }

  /** Reset the module encoder values */
  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  /**
   * Get the current state of the modules
   * 
   * @return state of the modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }

    return positions;
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return Constants.SwerveConstants.swerveKinematics.toChassisSpeeds(getStates());
  }

  /**
   * get the orientation of the robot
   * 
   * @return the orientation of the robot
   */
  public Rotation2d getYaw() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  /**
   * Get the pitch from gyro
   * 
   * @return the pitch of the robot
   */
  

  public void zeroGyro() {
    gyro.reset();
  }

  public void checkTunableValues() {
    if (!Constants.enableTunableValues)
      return;

    if (angleP.hasChanged() || angleI.hasChanged() || angleD.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.angleController.setP(angleP.get());
        mod.angleController.setI(angleI.get());
        mod.angleController.setD(angleD.get());
      }
    }

    if (driveP.hasChanged() || driveI.hasChanged() || driveD.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.driveController.setP(driveP.get());
        mod.driveController.setI(driveI.get());
        mod.driveController.setD(driveD.get());

        // mod.drivePIDController.setPID(driveP.get(), driveI.get(), driveD.get());
      }
    }
    if (driveS.hasChanged() || driveV.hasChanged() || driveA.hasChanged()) {
      for (SwerveModule mod : mSwerveMods) {
        mod.feedforward = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
      }
    }

    
  }


  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(getYaw(), getPositions(), pose);
  }


  

  public void logValues() {
 

  }

  public void burnToFlash() {
    for (SwerveModule mod : mSwerveMods) {
      mod.burnToFlash();
    }
  }


  @Override
  public void periodic() {
    checkTunableValues();
    for (SwerveModule mod : mSwerveMods) {
      mod.logValues();
    }

    poseEstimator.update(getYaw(), getPositions());

    switch (driveMode) {
      case AutonomousSnap:
        drive(new Translation2d(), 0);
        break;
      default:
        break;
    }

    for (SwerveModule mod : mSwerveMods) {
      mod.periodic();
    }
    logValues();
  }
}