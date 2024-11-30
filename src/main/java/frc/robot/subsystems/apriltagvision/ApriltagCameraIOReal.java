package frc.robot.subsystems.apriltagvision;

import frc.robot.Constants.VisionConstants.CameraInfo;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class ApriltagCameraIOReal implements ApriltagCameraIO {

  PhotonCamera camera;

  public ApriltagCameraIOReal(CameraInfo cameraInfo) {
    camera = new PhotonCamera(cameraInfo.cameraName);
  }

  @Override
  public void updateInputs(AprilTagCameraIOInputs inputs) {
    var result = camera.getLatestResult();
    if (result.hasTargets()) {
      inputs.result = result;
    } else {
      inputs.result = new PhotonPipelineResult();
    }
  }
}
