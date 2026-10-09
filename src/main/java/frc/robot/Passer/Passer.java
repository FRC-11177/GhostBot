package frc.robot.Passer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Passer.Constants.Pass;
import frc.robot.Shooter.Shooter;

public class Passer implements Subsystem{
    private static Passer inst;
    private Shooter shooter;

    public SparkMax PassMotor;
    public RelativeEncoder PassEncoder;

    private SparkMaxConfig PassConfig;

    private Passer(){
        PassMotor = new SparkMax(Pass.PassID, MotorType.kBrushless);
        PassEncoder = PassMotor.getEncoder();
        shooter = Shooter.getInstance();

        PassConfig = new SparkMaxConfig();

        PassConfig
            .idleMode(IdleMode.kCoast)
            .inverted(false)    
            .voltageCompensation(12)
            .smartCurrentLimit(40);
        PassConfig.encoder
            .positionConversionFactor(1/Pass.PassRatio)
            .velocityConversionFactor(1/Pass.PassRatio);

        PassMotor.configure(PassConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        setDefaultCommand(feed().onlyIf(shooter::isOK));
    }

    public Command feed(){
        return runEnd(() -> PassMotor.set(0.5), PassMotor::stopMotor);
    }

    public static Passer getInstance(){
        inst = inst == null ? new Passer() : inst;
        return inst;
    }
}
