// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Auto.Auto;
import frc.robot.Drivetrain.Constants;
import frc.robot.Drivetrain.Drivetrain;
import frc.robot.Intake.Intake;
import frc.robot.Passer.Passer;
import frc.robot.Shooter.Shooter;

public class RobotContainer {
  public Drivetrain drivetrain = Drivetrain.getInstance();
  public Intake intake = Intake.getInstance();
  public Shooter shooter = Shooter.getInstance();
  public CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    drivetrain.setDefaultCommand(drivetrain.drive(() -> new ChassisSpeeds(
      Constants.MaxDriveVelocity.times(controller.getLeftX()),
      Constants.MaxDriveVelocity.times(controller.getLeftY()),
      Constants.MaxOmega.times(controller.getRightX())
    )));
    configureBindings();
    Passer.getInstance(); //just call it once, it will move only if it needs to move
  }

  private void configureBindings() {
    controller.a().onTrue(intake.setState(new SwerveModuleState(5,Rotation2d.kCW_90deg)));
    controller.b().onTrue(intake.setState(new SwerveModuleState()));
    controller.x().onTrue(drivetrain.drive(new Pose2d(2.7,5.9,Rotation2d.fromDegrees(-45))).andThen(shooter.shoot(MetersPerSecond.of(5), false)));
  }

  public Command getAutonomousCommand() {
    return Auto.getAuto();
  }
}
