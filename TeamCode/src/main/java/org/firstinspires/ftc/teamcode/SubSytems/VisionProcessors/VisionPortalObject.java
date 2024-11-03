package org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
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
    private AprilTagProcessor aTagP = null;

    private ColorBlobLocatorProcessor blueColorLocator;
    private ColorBlobLocatorProcessor yellowColorLocator;



    /**
     * Create a Vision Portal object with the specified camera.
     * @param cameraName
     */
    public VisionPortalObject(WebcamName cameraName) throws InterruptedException {
        camera = cameraName;
        buildVisionPortal(aTagP);
    }


    /**
     * Add an April Tag processor to the vision portal.
     * @param atproc
     * @throws InterruptedException
     */
    public void buildVisionPortal(AprilTagProcessor atproc) throws InterruptedException {

        //aTagP = atproc;

        blueColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        yellowColorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.YELLOW)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

      visionPortal = new VisionPortal.Builder()
                .setCamera(camera)
                .addProcessor(blueColorLocator)
                .addProcessor(yellowColorLocator)
                .setCameraResolution(new Size(640, 480))
                .build();




   visionPortal.setProcessorEnabled(yellowColorLocator, true);
   visionPortal.setProcessorEnabled(blueColorLocator, true);


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
    public void enableAprilTagDetection()  { visionPortal.setProcessorEnabled(aTagP, true); }
    public void disableAprilTagDetection() { visionPortal.setProcessorEnabled(aTagP, false); }


    /**
     * Return the frames/second the vision portal is capturing. Can be used to assess cpu memory usage
     */
    public float fps() { return visionPortal.getFps(); }


    /**
     * All subsystems should contain a periodic method to check for any limit exceedances or needs
     * to update anything on a periodic basis, typically once per loop in runOpMode.
     */
    public void periodic() { }

    public List<ColorBlobLocatorProcessor.Blob> blueBlobs() {
        List<ColorBlobLocatorProcessor.Blob> blueBlobs = blueColorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blueBlobs);
        return blueBlobs;
    }

    public List<ColorBlobLocatorProcessor.Blob> yellowBlobs() {
        List<ColorBlobLocatorProcessor.Blob> yellowBlobs = yellowColorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, yellowBlobs);
        return yellowBlobs;
    }



}
