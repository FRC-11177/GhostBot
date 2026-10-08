package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Vision.Vision;

public class Drivetrain implements Subsystem{
    private static Drivetrain inst;
    private Vision vision;

    public SparkMax FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;
    public RelativeEncoder LeftEncoder, RightEncoder;
    public SparkClosedLoopController LeftPID, RightPID;
    public DifferentialDrivePoseEstimator3d PoseEstimator;
    public AHRS gyro;

    public PIDController HeadingPID;

    private SparkMaxConfig FrontLeftConfig, FrontRightConfig, BackLeftConfig, BackRightConfig;

    private Drivetrain(){
        FrontLeftMotor = new SparkMax(Constants.MotorIDs[0], MotorType.kBrushless);
        BackLeftMotor = new SparkMax(Constants.MotorIDs[1], MotorType.kBrushless);
        FrontRightMotor = new SparkMax(Constants.MotorIDs[2], MotorType.kBrushless);
        BackRightMotor = new SparkMax(Constants.MotorIDs[3], MotorType.kBrushless);

        LeftEncoder = FrontLeftMotor.getEncoder();
        RightEncoder = FrontRightMotor.getEncoder();
        LeftPID = FrontLeftMotor.getClosedLoopController();
        RightPID = FrontRightMotor.getClosedLoopController();

        gyro = new AHRS(NavXComType.kMXP_SPI);
        PoseEstimator = new DifferentialDrivePoseEstimator3d(Constants.kinematics, gyro.getRotation3d(), getPositions().leftMeters, getPositions().rightMeters, Constants.StartingPositon);
        vision = Vision.getInstance();
        HeadingPID = new PIDController(0, 0, 0);

        FrontLeftConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(120);
        FrontLeftConfig.encoder
            .positionConversionFactor(1/Constants.GearRatio)
            .velocityConversionFactor(1/Constants.GearRatio/60); //raw unit in rpm
        FrontLeftConfig.closedLoop
            .apply(Constants.DrivePID).apply(Constants.DriveMotion).apply(Constants.DriveFF);

        BackLeftConfig
            .follow(FrontLeftMotor);

        
        FrontRightConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(120);
        FrontRightConfig.encoder
            .positionConversionFactor(1/Constants.GearRatio)
            .velocityConversionFactor(1/Constants.GearRatio/60); //raw unit in rpm
        FrontRightConfig.closedLoop
            .apply(Constants.DrivePID).apply(Constants.DriveMotion).apply(Constants.DriveFF);

        BackRightConfig
            .follow(FrontRightMotor);

        FrontLeftMotor.configure(FrontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        FrontRightMotor.configure(FrontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        BackLeftMotor.configure(BackLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        BackRightMotor.configure(BackRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        AutoInit();
    }

    private DifferentialDriveWheelPositions getPositions(){
        return new DifferentialDriveWheelPositions(
            Constants.WheelCirc.times(LeftEncoder.getPosition()), //encoder position unit in rot
            Constants.WheelCirc.times(RightEncoder.getPosition()));
    }

    private DifferentialDriveWheelSpeeds getVelocity(){
        return new DifferentialDriveWheelSpeeds(
            Constants.WheelCirc.per(Second).times(LeftEncoder.getVelocity()), //encoder veclotiy in rot/sec
            Constants.WheelCirc.per(Seconds).times(LeftEncoder.getVelocity())
        );
    }

    public ChassisSpeeds getSpeeds(){
        return Constants.kinematics.toChassisSpeeds(getVelocity());
    }

    public Pose2d getPose(){
        return PoseEstimator.getEstimatedPosition().toPose2d();
    }

    public Command drive(Pose2d pose){
        return AutoBuilder.pathfindToPose(pose,Constants.AutoConstaints).andThen(turnTo(pose.getRotation()));
    }

    public Command drive(Supplier<ChassisSpeeds> spds){
        return drive(ChassisSpeeds.fromFieldRelativeSpeeds(getSpeeds(), gyro.getRotation2d()));
    }

    public Command drive(ChassisSpeeds robotRelativeSpeeds){
        return run(() -> drive(Constants.kinematics.toWheelSpeeds(robotRelativeSpeeds)));
    }

    public Command turnTo(Rotation2d target){
        return drive(
            new ChassisSpeeds(
                MetersPerSecond.zero(),
                MetersPerSecond.zero(),
                RadiansPerSecond.of(HeadingPID.calculate(getPose().getRotation().getRadians(), target.getRadians())))
        ).until(() -> getPose().getRotation().getMeasure().isNear(target.getMeasure(), 0.05));
    }

    private void drive(DifferentialDriveWheelSpeeds spds){
        spds.desaturate(Constants.MaxDriveVelocity);

        LeftPID.setSetpoint(spds.leftMetersPerSecond/Constants.WheelCirc.in(Meters), ControlType.kMAXMotionVelocityControl);
        RightPID.setSetpoint(spds.rightMetersPerSecond/Constants.WheelCirc.in(Meters), ControlType.kMAXMotionVelocityControl);
    }
    
    @Override
    public void periodic(){
        PoseEstimator.updateWithTime(Microseconds.of(RobotController.getFPGATime()).in(Seconds),gyro.getRotation3d(), getPositions());
        vision.getEstimatedPose().ifPresentOrElse((e) -> {
            if(Seconds.of(e.timestampSeconds).isNear(Microseconds.of(RobotController.getFPGATime()), 0.05))
                PoseEstimator.addVisionMeasurement(e.estimatedPose, e.timestampSeconds);
            DogLog.log("Drivetrain/isVisionUpdate", true);
        }, () -> DogLog.log("Drivetrain/isVisionUpdate", false));

        DogLog.log("Drivetrain/Pose2d", getPose());
        DogLog.log("Drivetrain/CurrentSpeed", getSpeeds());
    }

    private void AutoInit(){
        try{
            AutoBuilder.configure(
                this::getPose, 
                this::resetPose2d, 
                this::getSpeeds, 
                this::drive, 
                new PPLTVController(0.02), 
                RobotConfig.fromGUISettings(), 
                () -> false, 
            this);
        }catch(Exception e){
            DriverStation.reportError(e.getLocalizedMessage(), e.getStackTrace());
        }
    }

    private void resetPose2d(Pose2d pose){
        PoseEstimator.resetPose(new Pose3d(pose));
    }

    public static Drivetrain getInstance(){
        inst = inst == null ? new Drivetrain() : inst;
        return inst;
    }
}
