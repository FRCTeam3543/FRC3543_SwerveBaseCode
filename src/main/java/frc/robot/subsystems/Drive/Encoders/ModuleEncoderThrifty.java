package frc.robot.subsystems.Drive.Encoders;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ModuleEncoderThrifty extends SubsystemBase implements ModuleEncoder {
  private final AnalogInput encoder;
  private Rotation2d offset = new Rotation2d();
  private final DoublePublisher pubYaw;
  private final int encoderID;

  public ModuleEncoderThrifty(int id) {
    this.encoder = new AnalogInput(id);
    this.encoderID = id;
    

    // Set up NetworkTables and publisher
    NetworkTable encoderTable = NetworkTableInstance.getDefault().getTable("Encoders");
    pubYaw = encoderTable.getDoubleTopic("Yaw " + id).publish(PubSubOption.periodic(0.02));
  }

  @Override
  public void setOffset(Rotation2d offset) {
    this.offset = offset;
  }

  private double getRawAbsolutePosition() {
    return encoder.getAverageVoltage() / RobotController.getVoltage5V();
  }

  @Override
  public Rotation2d getAbsolutePosition() {
    double angle = Math.toRadians(360.0 * getRawAbsolutePosition()) - offset.getRadians();
    if (angle < 0) {
      angle += Math.PI * 2;
    }
    return Rotation2d.fromRadians(angle);
  }

  public void periodic() {
    // Publish the encoder angle to NetworkTables
    double yaw = getAbsolutePosition().getDegrees();
    pubYaw.accept(yaw);
    
    

  
  }
}
