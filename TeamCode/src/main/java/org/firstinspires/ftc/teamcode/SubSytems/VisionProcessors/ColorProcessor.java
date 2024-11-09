package org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

public class ColorProcessor {

    enum Filter {NONE, AREA, ASPECT};
//    Filter filter = Filter.NONE;
    ColorBlobLocatorProcessor colorProcessor;
    ColorRange color;

    public ColorProcessor(ColorRange color) {
        this.color = color;
        this.colorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
//              .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
    }


    public ColorBlobLocatorProcessor colorProcessor() {
        return colorProcessor;
    }


    private List<ColorBlobLocatorProcessor.Blob> getBlobs() {
        return colorProcessor.getBlobs();
    }


    public org.opencv.core.RotatedRect blobData(Filter filter) {

        RotatedRect boxFit = new RotatedRect();
        List<ColorBlobLocatorProcessor.Blob> blobs = getBlobs();

        switch(filter) {
            case NONE:
                break;

            case AREA:
                ColorBlobLocatorProcessor.Util.filterByArea(10000, 20000, blobs);
                break;
            case ASPECT:
                ColorBlobLocatorProcessor.Util.filterByAspectRatio(1.8, 3.2, blobs);
                break;
        }

        /* Do we have to loop through all the blobs? Is the largest the last blob since that is
        what sets the size (list order)? */
        if(!blobs.isEmpty()) {
            for (ColorBlobLocatorProcessor.Blob b : getBlobs()) {
                boxFit = b.getBoxFit();
            }
        } else boxFit = new RotatedRect();

        return boxFit;
    }

}
