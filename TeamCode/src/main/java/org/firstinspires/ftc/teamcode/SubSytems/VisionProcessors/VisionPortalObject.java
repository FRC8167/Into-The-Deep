package org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionPortalObject {

    /** WebCam1 Camera Settings **/
    WebcamName camera = null;
    private final long CAMERA_EXPOSURE = 6; //15;
    private final int  CAMERA_GAIN = 245;

    /** The variable to store our instance of the vision portal **/
    private VisionPortal visionPortal = null;
    ArrayList<VisionProcessor> processors;
    private int portalID;

    /**
     * Create a Vision Portal object with the specified camera.
     * @param builder
     * @throws InterruptedException
     */
    public VisionPortalObject(Builder builder) throws InterruptedException {
        this.camera = builder.camera;
        processors  = builder.processors;
        portalID    = builder.portalID;
        buildVisionPortal();
    }


    /*************************** Builder Class ***************************/
    public static class Builder {

        private WebcamName camera;
        private ArrayList<VisionProcessor> processors = new ArrayList<>();
        private int portalID;

        public Builder(WebcamName camera, int ID) {
            this.camera = camera;
            this.portalID = ID;
        }

        public VisionPortalObject.Builder addProcessor(VisionProcessor processor) {
            this.processors.add(processor);
            return this;
        }

        public VisionPortalObject build() throws InterruptedException { return new VisionPortalObject(this); }
    }
    /************************** END CONSTRUCTOR **************************/


    /**
     * Add an April Tag processor to the vision portal.
     *
     * @throws InterruptedException
     */
    public void buildVisionPortal() throws InterruptedException {

        /* Convert list of vision processors to an array */
        VisionProcessor[] processorArray = new VisionProcessor[processors.size()];
        processorArray = processors.toArray(processorArray);


        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .setLiveViewContainerId(portalID)
                .addProcessors(processorArray)
                .setCameraResolution(new Size(640, 480))
                .build();

        /** Pause code execution until camera state is streaming **/
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
        }
        setManualExposure(CAMERA_EXPOSURE, CAMERA_GAIN);
    }

    /**
     * Manually set the camera gain and exposure.
     * This can only be called AFTER calling initAprilTag(), and only works for Webcams;
     */
    public void setManualExposure(long exposureMS, int cameraGain) {

        if (visionPortal == null) {
            return;
        }

        /** Adjust exposure and gain settings to reduce motion blur **/
        ExposureControl exposure = visionPortal.getCameraControl(ExposureControl.class);
        if (exposure.isExposureSupported()) {
            exposure.setMode(ExposureControl.Mode.Manual);
            exposure.setExposure(exposureMS, TimeUnit.MILLISECONDS);

            GainControl gain = visionPortal.getCameraControl(GainControl.class);
            gain.setGain(cameraGain);
        }
    }


    /**
     * Return the frames/second the vision portal is capturing. Can be used to assess cpu memory usage
     */
    public float fps() { return visionPortal.getFps(); }


    /**
     * All subsystems should contain a periodic method to check for any limit exceedances or needs
     * to update anything on a periodic basis, typically once per loop in runOpMode.
     */
    public void periodic() { }

}