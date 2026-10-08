package frc.robot.Intake;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;

import edu.wpi.first.units.measure.Distance;

public class Constants {
    public class Pivot {
        public static final int PivotID = 21;
        public static final double PivotRatio = 60;
        public static final ClosedLoopConfig PivotPID = new ClosedLoopConfig()
            .pid(0, 0, 0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        public static final FeedForwardConfig PivotFF = new FeedForwardConfig()
            .svacr(0, 0, 0, 0, 0);
        public static final MAXMotionConfig PivotMotion = new MAXMotionConfig()
            .cruiseVelocity(0)
            .maxAcceleration(0);
        public static final SoftLimitConfig PivotLimit = new SoftLimitConfig()
            .forwardSoftLimit(0)
            .reverseSoftLimit(0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);
    }

    public class Roll{
        public static final int RollID = 22;
        public static final double RollRatio = 1;
        public static final Distance RollRadius = Inches.of(1.125);
        public static final ClosedLoopConfig RollPID = new ClosedLoopConfig()
            .pid(0, 0, 0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        public static final FeedForwardConfig RollFF = new FeedForwardConfig()
            .svacr(0, 0, 0, 0, 0);
        public static final MAXMotionConfig RollMotion = new MAXMotionConfig()
            .cruiseVelocity(0)
            .maxAcceleration(0);

    }
}
