package org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors;

import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.List;

public class ColorProcessor {

    public enum Filter {NONE, AREA, ASPECT}
    ColorBlobLocatorProcessor colorProcessor;
    ColorRange color;
    double alpha;
    double height;
    double width;
    Point[] corners = new Point[4];

    public ColorProcessor(ColorRange color) {
        this.color = color;
        this.colorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
//              .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5))  // search central 1/4 of camera view
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
    }


    public ColorBlobLocatorProcessor colorProcessor() {
        return colorProcessor;
    }


    public List<ColorBlobLocatorProcessor.Blob> getBlobs() {
        return colorProcessor.getBlobs();
    }


    public org.opencv.core.RotatedRect blobData(Filter filter) {

        RotatedRect boxFit = new RotatedRect();
        List<ColorBlobLocatorProcessor.Blob> blobs = getBlobs();

        switch(filter) {
            case NONE:
                break;

            case AREA:
                ColorBlobLocatorProcessor.Util.filterByArea(1000, 20000, blobs);  //was 10000
                break;
            case ASPECT:
                ColorBlobLocatorProcessor.Util.filterByAspectRatio(1.8, 3.2, blobs);
                break;
        }

        /* Do we have to loop through all the blobs? Is the largest the last blob since that is
        what sets the size (list order)? */
        if(!blobs.isEmpty()) {
//            for (ColorBlobLocatorProcessor.Blob b : getBlobs()) {
//                boxFit = b.getBoxFit();
//            }
            boxFit = getBlobs().get(0).getBoxFit();
        } else boxFit = new RotatedRect();


//        height = boxFit.boundingRect().height;
//        width = boxFit.boundingRect().width;
        boxFit.points(corners);

        height = Math.hypot(corners[1].x - corners[0].x, corners[1].y - corners[0].y);
        width = Math.hypot(corners[2].x - corners[1].x, corners[2].y - corners[1].y);


        alpha = boxFit.angle;

        return boxFit;
    }

    public Boolean ReadyForPickup()
    {
        blobData(ColorProcessor.Filter.NONE);
        return (!(height > width)) && (!(width > height));
    }


    public double CalcWristAngleDegrees()
    {
        blobData(ColorProcessor.Filter.NONE);

        if (height > width)
        {
            return (90 - alpha);
        }
        else if (width > height) {
            return (180 - alpha);
        }
        else {return 90.0;}
    }


    public double getAlpha()
    {
        return alpha;
    }


    public double getHeight()
    {
        return height;
    }


    public double getWidth()
    {
        return width;
    }
    public Point[] getPoint()
    {
        return corners;
    }


}