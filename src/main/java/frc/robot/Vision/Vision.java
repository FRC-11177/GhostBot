package frc.robot.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.Drivetrain;

public class Vision implements Subsystem{
    public PhotonCamera cam;
    public PhotonPoseEstimator PoseEstimator;
    private static Vision inst;

    private Vision(){
        cam = new PhotonCamera(Constants.CameraName);
        PoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark), Constants.RobotToCamera);
    }

    public Optional<EstimatedRobotPose> getEstimatedPose(){
        List<PhotonPipelineResult> res = cam.getAllUnreadResults();
        if(res.isEmpty()) return Optional.empty();
        else return PoseEstimator.estimateLowestAmbiguityPose(res.get(res.size()-1));
        
    }

    @Override
    public void periodic(){
        PoseEstimator.addHeadingData(RobotController.getFPGATime(), Drivetrain.getInstance().gyro.getRotation3d());
    }

    public static Vision getInstance(){
        inst = inst == null ? new Vision() : inst;
        return inst;
    }
}
