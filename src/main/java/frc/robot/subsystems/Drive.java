package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Arrays;

public class Drive extends SubsystemBase {

    public static enum DriveControlMode {
        JOYSTICK, PATH_FOLLOWING
    }

    private DriveControlMode driveControlMode = DriveControlMode.JOYSTICK;

    private TalonFX topLeftDrive;
    private TalonFX topRightDrive;
    private TalonFX bottomLeftDrive;
    private TalonFX bottomRightDrive;

    private CANCoder topLeftTurn;
    private CANCoder topRightTurn;
    private CANCoder bottomLeftTurn;
    private CANCoder bottomRightTurn;

    private SwerveModule frontLeft;
    private SwerveModule backLeft;
    private SwerveModule frontRight;
    private SwerveModule backRight;

    private PigeonIMU m_gyro;

    private SwerveDriveOdometry m_odometry;

    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            Constants.FRONT_LEFT_WHEEL_LOCATION, Constants.FRONT_RIGHT_WHEEL_LOCATION, Constants.BACK_LEFT_WHEEL_LOCATION, Constants.BACK_RIGHT_WHEEL_LOCATION);

    private final static Drive INSTANCE = new Drive();

    /** Creates a new ExampleSubsystem. */
    public Drive() {
        frontLeft = new SwerveModule(topLeftDrive, topLeftTurn, "Top_Left_Module");
        frontRight = new SwerveModule(topRightDrive, topRightTurn, "Top_Right_Module");
        backLeft = new SwerveModule(bottomLeftDrive, bottomLeftTurn, "Bottom_Left_Module");
        backRight = new SwerveModule(bottomRightDrive, bottomRightTurn, "bottom_Right_Module");

        m_gyro = new PigeonIMU(Constants.GYRO_CAN_ID);
        m_odometry = new SwerveDriveOdometry(m_kinematics,Rotation2d.fromDegrees(getGyroFusedHeadingAngleDeg()));
    }

    public synchronized double getGyroFusedHeadingAngleDeg() {
        return m_gyro.getFusedHeading();
    }

    // Example chassis speeds: 1 meter per second forward, 3 meters
    // per second to the left, and rotation at 1.5 radians per second
    // counterclockwise.
    private ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // Convert to module states
    private SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeftModuleState = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRightModuleState = moduleStates[0];

    // Back left module state
    SwerveModuleState bottomLeftModuleState = moduleStates[0];

    // Back right module state
    SwerveModuleState backRightModuleState = moduleStates[0];


    private ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0, 0.0, Math.PI / 0.0, Rotation2d.fromDegrees(0.0));


    private SwerveModuleState frontLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    private SwerveModuleState frontRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    private SwerveModuleState backLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));
    private SwerveModuleState backRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    // Convert to chassis speeds
    private ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
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
        var gyroAngle = Rotation2d.fromDegrees(-m_gyro.getFusedHeading());

        // Update the pose
        Pose2d m_pose = m_odometry.update(gyroAngle, frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backLeft.getState());
    }
}
