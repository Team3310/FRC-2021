package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {
    private TalonFX drive;
    private CANCoder turn;

    private String name;

    public SwerveModule(TalonFX drive, CANCoder turn, String name) {
        this.drive = drive;
        this.name = name;
        this.turn = turn;
    }


    public void setSpeed() {

    }

    public void setAngle(double angle) {

    }

    public double getSpeed(double speed) {
        return speed;
    }

    public double getAngle(double angle){
        return angle;
    }

    public SwerveModuleState getState() {
        SwerveModuleState swerveModuleState = new SwerveModuleState(getSpeed(0.0), Rotation2d.fromDegrees(getAngle(0.0)));
        return swerveModuleState;
    }
}
