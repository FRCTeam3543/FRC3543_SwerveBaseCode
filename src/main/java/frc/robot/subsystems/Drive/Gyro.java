package frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro {
    private final ADXRS450_Gyro gyro;
    private double yawOffset = 0;

    public Gyro() {
        gyro = new ADXRS450_Gyro();
        gyro.calibrate();
    }

    public double getYaw() {
        return gyro.getAngle() - yawOffset;
    }

    public void logValues() {
        SmartDashboard.putNumber("Gyro Yaw", getYaw());
    }

    public void home() {
        yawOffset = gyro.getAngle();
    }
}
