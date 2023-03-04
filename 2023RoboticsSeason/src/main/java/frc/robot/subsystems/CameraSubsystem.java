package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase {
    
    /**********************************************/
    //UNUSED CLASS, KEEPING FOR EASY FUTURE REFERENCE
    /**********************************************/

    private PhotonCamera photonCamera;
    private PhotonPoseEstimator photonPoseEstimator;
    public AprilTagFieldLayout aprilTagFieldLayout;
    

    public CameraSubsystem(){
        this.photonCamera = new PhotonCamera(Constants.CameraCharacteristics.photonVisionName);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, 
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, photonCamera, 
            Constants.CameraCharacteristics.robotToCamMeters);
    }

    public PhotonCamera getCamera(){
        return photonCamera;
    }

    public PhotonPoseEstimator getPhotonPoseEstimator(){
        return photonPoseEstimator;
    }
}
