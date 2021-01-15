package frc.robot.auto.routines;


import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.ResetOdometryAuto;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.subsystems.Drive;

public class AutoTest8Feet extends SequentialCommandGroup {
    TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
    Drive mDrive = Drive.getInstance();

    public AutoTest8Feet() {
        addCommands(new SequentialCommandGroup(
                new ResetOdometryAuto(new Pose2d(Units.inchesToMeters(76), Units.inchesToMeters(-155), new Rotation2d(Units.degreesToRadians(0)))),
                new RamseteCommand(
                        mTrajectories.getTestAuton(),
                        mDrive::getPose,
                        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                        new SimpleMotorFeedforward(Constants.ksVolts,
                                Constants.kvVoltSecondsPerMeter,
                                Constants.kaVoltSecondsSquaredPerMeter),
                        Constants.kDriveKinematics,
                        mDrive::getWheelSpeeds,
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                        // RamseteCommand passes volts to the callback
                        mDrive::tankDriveVolts,
                        mDrive),

                new StopTrajectory()
        )
        );
    }
}