package frc.robot.Auto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Shooter.Shooter;

public class Auto {
    private static Drivetrain drivetrain = Drivetrain.getInstance();
    private static Shooter shooter = Shooter.getInstance();
    private static Intake intake = Intake.getInstance();

    public static Command getAuto(){
        return new SequentialCommandGroup(
            drivetrain.drive(new Pose2d(7.7,7.4,Rotation2d.kCCW_90deg)),
            intake.setState(new SwerveModuleState(5,Rotation2d.kCW_90deg)).alongWith(drivetrain.drive(new Pose2d(7.7,4.5,Rotation2d.kCCW_90deg)))
                .andThen(intake.runOnce(intake.RollMotor::stopMotor)),
            drivetrain.drive(new Pose2d(7.7,7.4,Rotation2d.kCCW_Pi_2)),
            drivetrain.drive(new Pose2d(2.7,5.9,Rotation2d.fromDegrees(-45))),
            shooter.shoot(MetersPerSecond.of(5), false),
            drivetrain.drive(new Pose2d(4,7.4,Rotation2d.kCCW_90deg))
        ).repeatedly().withTimeout(Seconds.of(30));
    }
}
