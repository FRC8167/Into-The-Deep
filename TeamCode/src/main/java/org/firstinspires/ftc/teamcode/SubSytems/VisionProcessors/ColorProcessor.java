package org.firstinspires.ftc.teamcode.SubSytems.VisionProcessors;

import org.firstinspires.ftc.teamcode.Robot.TeamConstants;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class ColorProcessor implements TeamConstants {

    public enum Filter {NONE, AREA, ASPECT}
    ColorBlobLocatorProcessor colorProcessor;
    ColorRange color;
    double alpha;
    double height;
    double width;
    Point center;
    Point[] corners = new Point[4];
    private static final int NUM_READINGS = 20;
    private final Queue<Double> angleReadings = new LinkedList<>();
    private double sumAngles = 0;


    public void clearAngles(){
        angleReadings.clear();
        sumAngles = 0;
    }



    public double GetAvgWristAngle() {
        double newAngle = CalcWristAngleDegrees();

        // FIFO set to 20 readings
        if (angleReadings.size() >= NUM_READINGS) {
            Double removedValue = angleReadings.poll();
            if (removedValue == null) removedValue = 0.0;
            sumAngles -= removedValue;
        }

        angleReadings.add(newAngle);
        sumAngles += newAngle;

        // Return the average of values in queue
        return sumAngles / angleReadings.size();
    }

    public ColorProcessor(ColorRange color) {
        this.color = color;
        this.colorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(color)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.entireFrame())
//              .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))  // search central 1/4 of camera view
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(false)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();
    }


    public ColorBlobLocatorProcessor colorProcessor() {
        return colorProcessor;
    }


    public List<ColorBlobLocatorProcessor.Blob> getBlobs() {
        return colorProcessor.getBlobs();
    }

    public double getCenterOffsetInchesX(double InchesPerPixel) {
        if (center != null) {
            return (center.x - (FRAME_WIDTH / 2.0)) * InchesPerPixel;
        }
        else {
            return Double.NaN;
        }
    }

    public double getCenterOffsetInchesY(double InchesPerPixel) {
        if (center != null) {
            return ((FRAME_HEIGHT / 2.0) - center.y) * InchesPerPixel;
        }
        return Double.NaN;
    }

    public double calcInchPerPixelHorizontal(double cameraHeight) {
        double base;
        base = 2*(cameraHeight*Math.tan(Math.toRadians(45.2189284116/2)));
        return base/640;
    }

    public double calcInchPerPixelVertical(double cameraHeight) {
        double base;
        base = 2*(cameraHeight*Math.tan(Math.toRadians(34.6913691068/2)));
        return base/480;
    }

    public ColorBlobLocatorProcessor.Blob getClosestBlobToCenter() {
        List<ColorBlobLocatorProcessor.Blob> blobs = getBlobs();
        if (blobs.isEmpty()) return null;

        Point imageCenter = new Point(FRAME_WIDTH / 2.0, FRAME_HEIGHT / 2.0);
        ColorBlobLocatorProcessor.Blob closestBlob = blobs.get(0);
        double minDistance = distance(closestBlob.getBoxFit().center, imageCenter);

        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            //new DMW
            double blobArea = blob.getBoxFit().size.area();
            if (blobArea < 20000 || blobArea > 40000) continue;  //min 12000

            double blobDistance = distance(blob.getBoxFit().center, imageCenter);
            if (blobDistance < minDistance) {
                closestBlob = blob;
                minDistance = blobDistance;
            }
        }
        return closestBlob;
    }

    private double distance(Point p1, Point p2) {
        return Math.hypot(p1.x - p2.x, p1.y - p2.y);
    }


    public void blobData(Filter filter) {

        RotatedRect boxFit = new RotatedRect();
        List<ColorBlobLocatorProcessor.Blob> blobs = getBlobs();

        switch(filter) {
            case NONE:
                break;

            case AREA:
                ColorBlobLocatorProcessor.Util.filterByArea(12000, 40000, blobs);  //was 10000
                break;
            case ASPECT:
                ColorBlobLocatorProcessor.Util.filterByAspectRatio(2.8, 3.2, blobs);
                break;
        }

        /* Do we have to loop through all the blobs? Is the largest the last blob since that is
        what sets the size (list order)? */
        if(!blobs.isEmpty()) {
//            for (ColorBlobLocatorProcessor.Blob b : getBlobs()) {
//                boxFit = b.getBoxFit();
//            }
//NEW to replace get(0)
            ColorBlobLocatorProcessor.Blob targetBlob = getClosestBlobToCenter();
            if (targetBlob != null) {
                boxFit = targetBlob.getBoxFit();
            }
//            boxFit = getBlobs().get(0).getBoxFit();
        } else boxFit = new RotatedRect();


//        height = boxFit.boundingRect().height;
//        width = boxFit.boundingRect().width;
        boxFit.points(corners);
        center = boxFit.center;

        height = Math.hypot(corners[1].x - corners[0].x, corners[1].y - corners[0].y);
        width  = Math.hypot(corners[2].x - corners[1].x, corners[2].y - corners[1].y);


        alpha = boxFit.angle;

    }

    public Boolean ReadyForPickup()
    {
        blobData(ColorProcessor.Filter.AREA);  //was none
        return (!(height > width)) && (!(width > height));
    }

//NEW DMW
public double CalcWristAngleDegrees() {
    double angle = 90;

    ColorBlobLocatorProcessor.Blob targetBlob = getClosestBlobToCenter();
    blobData(Filter.AREA);  //added DMW

    if (targetBlob != null) {
        RotatedRect boxFit = targetBlob.getBoxFit();
        Point[] corners = new Point[4];
        boxFit.points(corners);

        // Compute height and width
        double height = Math.hypot(corners[1].x - corners[0].x, corners[1].y - corners[0].y);
        double width  = Math.hypot(corners[2].x - corners[1].x, corners[2].y - corners[1].y);
        double alpha = boxFit.angle;

        // Determine the correct wrist angle based on the object's orientation
        if (height > width) {
            angle = (90 - alpha);
        } else if (width > height) {
            angle = (180 - alpha);
        }
    }

    return angle;
}

//    public double CalcWristAngleDegrees() {
//
//        double angle = 90;
//        blobData(Filter.NONE);
//
//        if(getBlobs().size() > 0) {
//            if (height > width) {
//                angle = (90 - alpha);
//            } else if (width > height) {
//                angle = (180 - alpha);
//            }
//        }
//        else { angle =  90.0; }
//
//        return angle;
//    }


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


    public double getArea() {
        return height * width;
    }
}