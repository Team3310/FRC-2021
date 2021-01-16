package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.lists;

public class SwerveDrive extends SubsystemBase {

    private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;

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

    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Convert to module states
    private SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    private SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    private SwerveModuleState frontRight = moduleStates[0];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[0];

    // Back right module state
    SwerveModuleState backRight = moduleStates[0];

    var frontLeftOptimized = SwerveModuleState.optimize(frontLeft,
            new Rotation2d(m_turningEncoder.getDistance()));

    private ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0, 0.0, Math.PI / 0.0, Rotation2d.fromDegrees(0.0));

    // Now use this in our kinematics
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

    var frontLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    var frontRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    var backLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    var backRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    // Convert to chassis speeds
    private ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(
            frontLeftState, frontRightState, backLeftState, backRightState);

    // Getting individual speeds
    private double forward = chassisSpeeds.vxMetersPerSecond;
    private double sideways = chassisSpeeds.vyMetersPerSecond;
    private double angular = chassisSpeeds.omegaRadiansPerSecond;

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Get my gyro angle. We are negating the value because gyros return positive
        // values as the robot turns clockwise. This is not standard convention that is
        // used by the WPILib classes.
        var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getAngle());

        // Update the pose
        m_pose = m_odometry.update(gyroAngle, m_frontLeftModule.getState(), m_frontRightModule.getState(),
                m_backLeftModule.getState(), m_backRightModule.getState());
    }
}
