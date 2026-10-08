package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static final int MotorIDs[] = {10,11,12,13};
    public static final ClosedLoopConfig DrivePID = new ClosedLoopConfig()
        .pid(0, 0, 0);
    public static final FeedForwardConfig DriveFF = new FeedForwardConfig()
        .sva(0, 0, 0);
    public static final MAXMotionConfig DriveMotion = new MAXMotionConfig()
        .maxAcceleration(0);

    public static final double GearRatio = 10.71;
    public static final Distance TrackWidth = Inches.of(25);
    public static final Distance WheelRadius = Inches.of(3);
    public static final Distance WheelCirc = WheelRadius.times(2).times(Math.PI);
    public static final LinearVelocity MaxDriveVelocity = MetersPerSecond.of(4);
    public static final LinearVelocity TrueMaxVelocity = WheelCirc.per(Seconds).times(5676/60);

    public static final PathConstraints AutoConstaints = new PathConstraints(TrueMaxVelocity, MetersPerSecondPerSecond.of(9.8), RadiansPerSecond.of(3), RadiansPerSecondPerSecond.of(27));

    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(TrackWidth);
    public static final Pose3d StartingPositon = new Pose3d(new Pose2d());
}
