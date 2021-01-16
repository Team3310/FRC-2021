package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {
    private TalonFX drive;
    private TalonFX turn;
    private CANCoder turnCoder;

    private String name;

    public SwerveModule(TalonFX drive,TalonFX turn, CANCoder turnCoder, String name) {
        this.drive = drive;
        this.name = name;
        this.turn = turn;
        this.turnCoder = turnCoder;

    }

    public void setDesiredState(SwerveModuleState state) {
        // Calculate the drive output from the drive PID controller.
        final var driveOutput =
                m_drivePIDController.calculate(m_driveEncoder.getRate(), state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final var turnOutput =
                m_turningPIDController.calculate(m_turningEncoder.get(), state.angle.getRadians());

        // Calculate the turning motor output from the turning PID controller.
        drive.set(driveOutput);
        turn.set(turnOutput);
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
