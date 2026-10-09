package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Centimeters;

import com.ctre.phoenix6.swerve.utility.WheelForceCalculator.Feedforwards;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public static final int ShootID = 41;
    public static final double ShootRatio = 1;
    public static final Distance ShootCirc = Centimeters.of(3).times(Math.PI);
    public static final ClosedLoopConfig ShootPID = new ClosedLoopConfig()
        .pid(0, 0, 0)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    public static final FeedForwardConfig ShootFF = new FeedForwardConfig()
        .sva(0, 0, 0);
    public static final MAXMotionConfig ShootMotion = new MAXMotionConfig()
        .maxAcceleration(0);
}
