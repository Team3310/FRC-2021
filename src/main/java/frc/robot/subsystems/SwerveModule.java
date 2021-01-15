package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

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
        turn.setPosition(angle);
    }
}
