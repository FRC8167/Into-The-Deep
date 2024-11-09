package org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class VisionPortalObject {

    /** WebCam1 Camera Settings **/
    WebcamName camera = null;
    private final long CAMERA_EXPOSURE = 6; //15;
    private final int  CAMERA_GAIN = 250;

    /** The variable to store our instance of the vision portal **/
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTagProcessor = null;

    private ColorBlobLocatorProcessor blueColorLocator;
    private ColorBlobLocatorProcessor yellowColorLocator;
    private ColorBlobLocatorProcessor redColorLocator;


    /**
     * Create a Vision Portal object with the specified camera.
     * @param cameraName
     */
    public VisionPortalObject(WebcamName cameraName) throws InterruptedException {
        camera = cameraName;
        buildVisionPortal(aprilTagProcessor);
    }


    /**
     * Add an April Tag processor to the vision portal.
     * @param atproc
     * @throws InterruptedException
     */
    public void buildVisionPortal(AprilTagProcessor atproc) throws InterruptedException {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(889.035, 889.035, 390.3019, 66.3539)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();
        aprilTagProcessor.setDecimation(1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(aprilTagProcessor)
                .addProcessor(redColorLocator)
                .addProcessor(blueColorLocator)
                .addProcessor(yellowColorLocator)
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

    /*************************** Processor Controls ***************************/
    public void enableAprilTagDetection()  { visionPortal.setProcessorEnabled(aprilTagProcessor, true); }
    public void disableAprilTagDetection() { visionPortal.setProcessorEnabled(aprilTagProcessor, false); }


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
