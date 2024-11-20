package org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagProcessorObject {

    /** Season Tag IDs **/
    public final int BLUE_AUDIENCE_WALL = 11;
    public final int BLUE_SEAM_3 = 12;
    public final int BLUE_REAR_WALL = 13;
    public final int RED_AUDIENCE_WALL = 14;
    public final int RED_SEAM_3 = 15;
    public final int RED_REAR_WALL = 16;

    /** The variable to store our instance of the AprilTag processor. **/
    private AprilTagProcessor aprilTagProc = null;

    private List<AprilTagDetection> currentDetections = null;

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // eg: Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    private final int decimationRate = 1;


    /* Uncomment setLensIntrinsics line in the constructor if used */
    private final double LENS_INTRINSICS_FX = 1426.86;//889.035;
    private final double LENS_INTRINSICS_FY = 1426.86;//889.035;
    private final double LENS_INTRINSICS_CX = 542.206;//390.3019;
    private final double LENS_INTRINSICS_CY = 407.087;//66.3539;


    /**
     * Create an processor for April Tag detections
     * @throws InterruptedException
     */
    public AprilTagProcessorObject()  {

        aprilTagProc = new AprilTagProcessor.Builder()
                // The following default settings are available to un-comment and edit as needed.
                //.setLensIntrinsics(LENS_INTRINSICS_FX, LENS_INTRINSICS_FY, LENS_INTRINSICS_CX, LENS_INTRINSICS_CY)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawTagID(false)
                .setDrawTagOutline(false)    // Change these options to ture if needed for debugging
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        aprilTagProc.setDecimation(decimationRate);
    }


    /**
     * Returns the current April Tag Processor
     * @return
     */
    public AprilTagProcessor getProcessor() { return aprilTagProc; }


    /**
     * Must be called prior to any other April Tag Functions
     */
    public void scanForAprilTags() { currentDetections = aprilTagProc.getDetections(); }


    /**
     * Returns all the April Tag objects detected during the scanForAprilTags() call.
     * @return
     */
//    public List<AprilTagDetection> allAprilTagsDetected() { return aprilTag.getDetections(); }
    public List<AprilTagDetection> allAprilTagsDetected() { return currentDetections; }


    /**
     * Returns a boolean indicating if any tags were found.
     * @return
     */
    public boolean aprilTagsDetected() {
        return currentDetections.size() > 0;
    }


    /**
     * Return if the requested April Tag was found in the scan.
     * @param tagId
     * @return
     */
    public boolean aprilTagAvailable(int tagId) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagId) {
                return true;
            }
        }
        return false;
    }


    /**
     * Return the number of April Tags detected in the scan.
     * @return
     */
    public int aprilTagsDetectedCount() { return currentDetections.size(); }


    /**
     * Return the data of the requested April Tag.
     * @param tagId
     * @return
     */
    public AprilTagDetection getTagData(int tagId) {
        AprilTagDetection tag = null;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && detection.id == tagId) {
                tag = detection;
                break;  // don't look any further.
            }
        }
        return tag;
    }


}
