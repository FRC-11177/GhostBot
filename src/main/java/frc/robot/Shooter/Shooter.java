package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Shooter implements Subsystem{
    private static Shooter inst;

    public SparkMax ShootMotor;
    public RelativeEncoder ShootEncoder;
    public SparkClosedLoopController ShootPID;
    private SparkMaxConfig ShootConfig;

    private boolean isIdle;
    private AngularVelocity velSetpoint;

    private Shooter(){
        ShootMotor = new SparkMax(Constants.ShootID, MotorType.kBrushless);
        ShootEncoder = ShootMotor.getEncoder();
        ShootPID = ShootMotor.getClosedLoopController();

        ShootConfig = new SparkMaxConfig();

        ShootConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(40);
        ShootConfig.encoder
            .positionConversionFactor(1/Constants.ShootRatio)
            .velocityConversionFactor(1/Constants.ShootRatio/60);
        
        ShootMotor.configure(ShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        setDefaultCommand(this.shoot(MetersPerSecond.of(1), true));
    }

    public boolean isOK(){
        return !isIdle && RotationsPerSecond.of(ShootPID.getSetpoint()).isNear(velSetpoint, 0.05);
    }

    public Command shoot(LinearVelocity vel, boolean isIdle){
        return runEnd(() ->{
            if(isIdle) return;
            ShootPID.setSetpoint(vel.in(MetersPerSecond)/Constants.ShootCirc.in(Meters), ControlType.kMAXMotionVelocityControl); //actually rot/s 
            velSetpoint = RotationsPerSecond.of(vel.in(MetersPerSecond)/Constants.ShootCirc.in(Meters));
            this.isIdle = false;
        },() -> {
            ShootPID.setSetpoint(3, ControlType.kMAXMotionVelocityControl);
            this.isIdle = true;
        });
    }

    public static Shooter getInstance(){
        inst = inst == null ? new Shooter() : inst;
        return inst;
    }
}
