package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {
    private TalonFX topLeftDrive;
    private TalonFX topRightDrive;
    private TalonFX bottomLeftDrive;
    private TalonFX bottomRightDrive;

    private CANCoder topLeftTurn;
    private CANCoder topRightTurn;
    private CANCoder bottomLeftTurn;
    private CANCoder bottomRightTurn;

    private SwerveModule topLeft;
    private SwerveModule bottomLeft;
    private SwerveModule topRight;
    private SwerveModule bottomRight;

    private PigeonIMU gyro;

    private SwerveDriveOdometry m_odometry;

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            Constants.FRONT_LEFT_WHEEL_LOCATION, Constants.FRONT_RIGHT_WHEEL_LOCATION, Constants.BACK_LEFT_WHEEL_LOCATION, Constants.BACK_RIGHT_WHEEL_LOCATION);

    private final static SwerveDrive INSTANCE = new SwerveDrive();

    /** Creates a new ExampleSubsystem. */
    public SwerveDrive() {
        topLeft = new SwerveModule(topLeftDrive, topLeftTurn, "Top_Left_Module");
        topRight = new SwerveModule(topRightDrive, topRightTurn, "Top_Right_Module");
        bottomLeft = new SwerveModule(bottomLeftDrive, bottomLeftTurn, "Bottom_Left_Module");
        topLeft = new SwerveModule(bottomRightDrive, bottomRightTurn, "bottom_Right_Module");

        gyro = new PigeonIMU(Constants.GYRO_CAN_ID);
        m_odometry = new SwerveDriveOdometry(m_kinematics,Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()));
    }

    public synchronized double getGyroFusedHeadingAngleDeg() {
        return gyro.getFusedHeading();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
