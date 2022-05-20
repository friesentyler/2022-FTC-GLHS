/* You will not need to mess with this file, this is what does all the processing on the
    backend to find the object coordinates
 */

package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class SamplePipeline extends OpenCvPipeline
{
    boolean viewportPaused;
    // Mats are the objects in the OpenCV library that contain images
    Mat mat = new Mat();
    // just a blank variable of the Rectangle class
    Rect rectangle;
    /* This most likely will not be relevant if you are just playing with the code.
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    Telemetry telemetry;

    public SamplePipeline(Telemetry t) {
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input)
    {
        /*
         * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
         * will only dereference to the same image for the duration of this particular
         * invocation of this method. That is, if for some reason you'd like to save a copy
         * of this particular frame for later use, you will need to either clone it or copy
         * it to another Mat.
         */

        // converts image to HSV format
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // filters for the color yellow
        Scalar lowHSV = new Scalar(23, 50, 70);
        Scalar highHSV = new Scalar(32, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);

        // converts to RGB format from HSV
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        // convert to greyscale from RGB format
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2GRAY);
        rectangle = Imgproc.boundingRect(mat);
        Imgproc.rectangle(mat, new Point(rectangle.x, rectangle.y), new Point(rectangle.x + rectangle.width, rectangle.y + rectangle.height), new Scalar(255, 255, 0), 5);
        

        telemetry.update();
        return mat;
    }

    // This function gets called in Webcam example to get the coordinates of the object in the frame.
    public Rect getRect() {
        return rectangle;
    }

    /*@Override
    public void onViewportTapped()
    {
        /*
         * The viewport (if one was specified in the constructor) can also be dynamically "paused"
         * and "resumed". The primary use case of this is to reduce CPU, memory, and power load
         * when you need your vision pipeline running, but do not require a live preview on the
         * robot controller screen. For instance, this could be useful if you wish to see the live
         * camera preview as you are initializing your robot, but you no longer require the live
         * preview after you have finished your initialization process; pausing the viewport does
         * not stop running your pipeline.
         *
         * Here we demonstrate dynamically pausing/resuming the viewport when the user taps it
         */

        /*viewportPaused = !viewportPaused;

        if(viewportPaused)
        {
            webcam.pauseViewport();
        }
        else
        {
            webcam.resumeViewport();
        }
    }*/
}