package frc.robot.Intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Intake.Constants.Pivot;
import frc.robot.Intake.Constants.Roll;

public class Intake implements Subsystem{
    private static Intake inst;

    public SparkMax PivotMotor, RollMotor;
    public RelativeEncoder PivotEncoder, RollEncoder;
    public SparkClosedLoopController PivotPID, RollPID;
    
    private SparkMaxConfig PivotConfig, RollConfig;

    private Intake(){
        PivotMotor = new SparkMax(Pivot.PivotID, MotorType.kBrushless);
        PivotEncoder = PivotMotor.getEncoder();
        PivotPID = PivotMotor.getClosedLoopController();

        RollMotor = new SparkMax(Roll.RollID, MotorType.kBrushless);
        RollEncoder = RollMotor.getEncoder();
        RollPID = RollMotor.getClosedLoopController();

        PivotConfig = new SparkMaxConfig();
        RollConfig = new SparkMaxConfig();

        PivotConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(60)
            .voltageCompensation(12);
        PivotConfig.encoder
            .positionConversionFactor(1/Pivot.PivotRatio)
            .velocityConversionFactor(1/Pivot.PivotRatio/60);
        PivotConfig.closedLoop
            .apply(Pivot.PivotPID).apply(Pivot.PivotFF).apply(Pivot.PivotMotion);
        PivotConfig.apply(Pivot.PivotLimit);

        RollConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .smartCurrentLimit(40)
            .voltageCompensation(12);
        RollConfig.encoder  
            .positionConversionFactor(1/Roll.RollRatio)
            .velocityConversionFactor(1/Roll.RollRatio/60);
        RollConfig.closedLoop
            .apply(Roll.RollPID).apply(Roll.RollFF).apply(Roll.RollMotion);
            
        PivotMotor.configure(PivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RollMotor.configure(RollConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Roll.RollRadius.per(Second).times(2*Math.PI).times(RollEncoder.getVelocity()),
            Rotation2d.fromRotations(PivotEncoder.getPosition())
        );
    }

    public Command setState(SwerveModuleState targetState){
        return run(() -> {
            RollPID.setSetpoint(targetState.speedMetersPerSecond/Roll.RollRadius.times(2*Math.PI).in(Meters), ControlType.kMAXMotionVelocityControl);
            PivotPID.setSetpoint(targetState.angle.getRotations(), ControlType.kMAXMotionPositionControl);
        });
    }

    @Override
    public void periodic(){
        DogLog.log("Intake/State", getState());
    }

    public static Intake getInstance(){
        inst = inst == null ? new Intake() : inst;
        return inst;
    }
}
