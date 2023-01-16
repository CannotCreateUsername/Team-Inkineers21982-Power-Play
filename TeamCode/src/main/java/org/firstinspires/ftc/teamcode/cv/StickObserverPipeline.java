package org.firstinspires.ftc.teamcode.cv;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
/*@Config*/
public class StickObserverPipeline extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 140;
    public static double strictHighS = 255;

    // this is the rectangle of the widest (assuming to be most close) yellow junction
    private Rect maxRect;
    public Rect getStickRect(){
        return maxRect;
    }


    public StickObserverPipeline() {
        frameList = new ArrayList<>();
    }

    /**
     *  Mat: Matrix class from open CV -
     *  https://docs.opencv.org/3.4/d3/d63/classcv_1_1Mat.html
     * @param input
     * @return
     */
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        // lenient bounds will filter out near yellow, this should filter out all near yellow things(tune this if needed)
        Scalar lowHSV = new Scalar(20, 70, 80); // new  Scalar(23,50,70); // new Scalar(20, 70, 80); // lenient lower bound HSV for yellow
        Scalar highHSV = new Scalar(32, 255, 255); // new Scalar(32,255,255); // new Scalar(32, 255, 255); // lenient higher bound HSV for yellow

        Mat thresh = new Mat();

        // Get a black and white image of yellow objects
        // ref: https://www.reddit.com/r/FTC/comments/u71pzb/easyopencv_scalar_color_question/
        /*
        Scalars don't have an inherent color space to them. The InRange function just takes the image matrix and compares it directly to the scalars, so in that context they are effectively in the same color space as the image being compared.
        For example, InRange(RGBImage, lowHSV, highHSV) would treat them as RGB values,
        while InRange(HSVImage, lowHSV, highHSV) would treat them like HSV values
         */
        Core.inRange(mat, lowHSV, highHSV, thresh);

        Mat masked = new Mat();
        //color the white portion of thresh in with HSV from mat
        //output into masked
        Core.bitwise_and(mat, mat, masked, thresh);
        //calculate average HSV values of the white thresh values
        Scalar average = Core.mean(masked, thresh);

        Mat scaledMask = new Mat();
        //scale the average saturation to 150
        masked.convertTo(scaledMask, -1, 150 / average.val[1], 0);


        Mat scaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(scaledMask, strictLowHSV, strictHighHSV, scaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, scaledThresh);

        Mat edges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(scaledThresh, edges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(scaledThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);


        /**drawing contours to ret in green**/
        Imgproc.drawContours(input, contours, -1, new Scalar(0.0, 255.0, 0.0), 3);

        // inspired by the logic fromm 2021-2022 Ultmiate goal
        // ref: https://docs.ftclib.org/ftclib/vision/ring-stack-detection
        int maxWidth = 0;
        maxRect = new Rect();
        for (MatOfPoint c : contours) {
            MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
            Rect rect = Imgproc.boundingRect(copy);

            int w = rect.width;
            // checking if the rectangle is below the horizon
            if (w > maxWidth) {
                maxWidth = w;
                maxRect = rect;
            }
            c.release(); // releasing the buffer of the contour, since after use, it is no longer needed
            copy.release(); // releasing the buffer of the copy of the contour, since after use, it is no longer needed
        }

        /**drawing target rectange in red **/
        Imgproc.rectangle(input, maxRect, new Scalar(255.0, 0, 0.0), 5 );

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }




        //release all the data
        // input.release(); // release original input before reassigning, otherwise, there can be meomory leak
        // mat.copyTo(input); // for bebug only - RBG to HSV
        // thresh.copyTo(input); // for debug only - first HSV rough filter
        // masked.copyTo(input); // for debug only - a masked
        // scaledMask.copyTo(input); // for debug only - weighted mask
        // scaledThresh.copyTo(input);  // final result
        // finalMask.copyTo(input);
        // edges.copyTo(input);

        // release all other temporary mat
        mat.release();
        thresh.release();
        masked.release();
        scaledMask.release();
        scaledThresh.release();
        finalMask.release();
        edges.release();

        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return input;
    }


}