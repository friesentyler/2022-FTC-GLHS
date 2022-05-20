/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* This is the main file you'll want to modify, skip to line 232 for the real meat of the code,
* everything else is mostly boilerplate code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.*;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.SamplePipeline;

@TeleOp
public class WebcamExample extends LinearOpMode
{
    // just variable declarations, ignore these
    OpenCvWebcam webcam;

    private Blinker control_Hub;
    private TouchSensor emergency_;
    private TouchSensor emergency_Stop;
    private DcMotor flywheel1;
    private DcMotor flywheel2;
    private DcMotor lift1;
    private DcMotor lift2;
    private HardwareDevice webcam_1;
    private ColorSensor color;
    private Servo left_claw;
    private Servo right_claw;
    private Servo test;
    private Servo trigger;

    private Button buttonA;
    private Button DpadUp;
    private Button DpadDown;
    private Button DpadRight;
    private Button DpadLeft;
    double Count = 0;
    double Angle = 0;

    private DigitalChannel in1;
    private DigitalChannel out1;
    private ElapsedTime timer = new ElapsedTime();

    // define class variables.
    // all variables defined at class scope so that the methods can access them.
    private double lastTime = 0.0;
    private double timeNow = 0.0;
    private boolean lastState = false;
    private double measureTime = 0.0;
    private double difference = 0.0;

    private double period = 1000;
    private byte txBuffer = 0;
    private byte rxBuffer = 0;

    private int txSeq = 0;
    private int rxSeq = 0;

    private double rxLastTime = 0.0;
    private double txLastTime = 0;

    private boolean txFlag = false;
    private boolean rxFlag = false;

    private double txPeriod = period;
    private double rxPeriod = period;
    private int txSeqCount = 0;

    private byte rxBit = 0;
    private byte txBit = 0;
    private byte rxAddValue = 0;

    private byte txIndex = 0;
    private byte rxIndex = 0;

    @Override
    public void runOpMode()
    {
        // hardware map so the code knows which part is which, ignore this.
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        flywheel1 = hardwareMap.get(DcMotor.class, "Flywheel1");
        flywheel2 = hardwareMap.get(DcMotor.class, "Flywheel2");
        lift1 = hardwareMap.get(DcMotor.class, "Lift1");
        lift2 = hardwareMap.get(DcMotor.class, "Lift2");
        webcam_1 = hardwareMap.get(HardwareDevice.class, "Webcam 1");
        color = hardwareMap.get(ColorSensor.class, "color");
        left_claw = hardwareMap.get(Servo.class, "left claw");
        right_claw = hardwareMap.get(Servo.class, "right claw");
        test = hardwareMap.get(Servo.class, "test");
        trigger = hardwareMap.get(Servo.class, "trigger");
        in1 = hardwareMap.get(DigitalChannel.class, "in1");
        out1 = hardwareMap.get(DigitalChannel.class, "out1");
        buttonA = new Button();
        DpadUp = new Button();
        DpadDown = new Button();
        DpadRight = new Button();
        DpadLeft = new Button();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        lift1.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        lift1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift1.setTargetPosition(0);
        lift2.setTargetPosition(0);
        lift1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx) lift1).setVelocity(200);
        ((DcMotorEx) lift2).setVelocity(200);
        waitForStart();


        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * In this sample, we're using a webcam. Note that you will need to
         * make sure you have added the webcam to your configuration file and
         * adjusted the name here to match what you named it in said config file.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone). If no camera monitor is desired, use the alternate
         * single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera. Note that switching pipelines on-the-fly
         * (while a streaming session is in flight) *IS* supported.
         */
        SamplePipeline pipeline = new SamplePipeline(telemetry);
        webcam.setPipeline(pipeline);
        //webcam.setPipeline(new SamplePipeline(telemetry));

        /*
         * Open the connection to the camera device. New in v1.4.0 is the ability
         * to open the camera asynchronously, and this is now the recommended way
         * to do it. The benefits of opening async include faster init time, and
         * better behavior when pressing stop during init (i.e. less of a chance
         * of tripping the stuck watchdog)
         *
         * If you really want to open synchronously, the old method is still available.
         */
        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();

        /* inside this loop is where the actually interesting stuff happens. */
        while (opModeIsActive())
        {
            /*
             * Send some stats to the telemetry
             */
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            telemetry.update();

            if(gamepad1.a)
            {
                webcam.stopStreaming();
                //webcam.closeCameraDevice();
            }
            // this gets the object coordinates from the pipeline and stores them as a Rect object
            Rect result = pipeline.getRect();
            // this just prints out telemetry data of the rectangle to the driver hub
            telemetry.addData("boundingRect", result);

            // this code is what adjusts the motor position to be inline with the object it detects.
            // try catch blocks catch the NullPointerException that gets thrown when the Rect object
            // returns empty. We just set the target position to the current position if that happens
            // otherwise we use the AngleToCounts() function.
            try {
                // result.y gives us the y coordinate of the detected object. We subtract 120 because
                // that is the middle of the frame on the y axis. We then multiply it by 0.235 to convert
                // from pixels to degrees. That measurement was gotten by dividing a certain centimeter
                // amount by the pixel count. cm/pixel. We add 10 degrees because the camera is just
                // above the flywheels to account for the height of the camera. I recommend playing with
                // this added degree amount if you want to make it more accurate.
                // See AngleToCounts() method below for more details.
                AngleToCounts(((result.y - 120) * 0.235)+10);
            } catch (NullPointerException n) {
                System.out.println("oopsie daisy");
                lift1.setTargetPosition(lift1.getCurrentPosition());
                lift2.setTargetPosition(lift2.getCurrentPosition());
            }

            // This code turns the flywheels on if it detects the object within a certain range of
            // the center of the frame.
            // This also needs a try catch block. basically we need them anytime we use the "result" object
            // since sometimes it returns empty. The if condition checks if the y value of the object is within
            // 75 pixels on either side of the object. For some reason this doesn't work like I think it should.
            // You can play with this and try to see if you can get the flywheels to turn on accurately.
            try {
                if ((result.y - (result.height/2)) - 120 <= 75 && (result.y - (result.height/2)) - 120 >= -75) {
                    flywheel1.setPower(-.7);
                    flywheel2.setPower(.7);
                    sleep(100);
                    flywheel2.setPower(0);
                    flywheel1.setPower(0);
                }
            } catch (NullPointerException n) {
                // yeet since I don't know what to do with the exception.
                System.out.println("yeet");
            }

            // This is all your guys' serial communication protocol that you built with Mr. A.
            // You can uncomment this if you want to try to fix this beast, good luck! You'll need it XD
            /*switch(txSeq){
                case 0:
                    txIdle();
                    break;
                case 1:
                    txStartSignal();
                    break;
                case 2:
                    txActive();
                    break;
                default:
                    telemetry.addData("Error", "!tx switch error!" );
            }
            switch(rxSeq){
                case 0:
                    rxIdle();
                    break;
                case 1:
                    rxSetTiming();
                    break;
                case 2:
                    rxActive();
                    break;
                default:
                    telemetry.addData("Error", "!rx switch error!" );
            }


            telemetry.addData("Tx Index",txIndex);
            telemetry.addData("Tx bit", txBit);
            telemetry.addData("Rx Index", rxIndex);
            telemetry.addData("Rx bit", rxBit);
            telemetry.addData("Rx add value", rxAddValue);
            telemetry.addData("Rx Buffer",rxBuffer );
            telemetry.addData("in1 value", in1.getState());
            telemetry.addData("Status", "Running");
            telemetry.update();*/


        }
    }

    // This is your AngleToCounts() function. I modified it a little so that it just adjusts the
    // target position directly instead of returning a motor tick value.
    private void AngleToCounts(double Angle){

        double VarCon = 768.0 / 360.0;
        int counts = (int)VarCon * (int)Angle;
        lift1.setTargetPosition(counts);
        lift2.setTargetPosition(counts);

    }

    // more of the serial protocol
    /*private void txIdle(){
        if (txSeq == 0 && gamepad1.a){
            txBuffer = -100;
            txSeq++;
        }

    }

    private void txStartSignal(){
        // On the first time through:
        // Set the output HIGH, reset the time reference, and set the period to 3P
        if(txSeqCount == 0){
            out1.setState(true);
            txLastTime = timer.milliseconds();
            txPeriod = 3*period;
            txSeqCount++;
        }
        // Wait for the period to expire. There will be two repeats with different periods.
        if(timer.milliseconds() - txLastTime > txPeriod){
            // The First time, 3P, set the output LOW and reset period
            if(txSeqCount == 1){
                out1.setState(false);
                txPeriod = period;
                txSeqCount++;
            }
            // The Second time, 1P, is just to delay 1P before transmitting for timing.
            else{
                out1.setState(false);
                txSeqCount = 0;
                txSeq++;
                return;
            }
            // must update the time reference each time through.
            txLastTime = timer.milliseconds();
        }
    }

    private void txActive(){
        if(timer.milliseconds() - txLastTime > txPeriod){

            // If the index is 8, all data has been sent.
            if (txIndex == 8){
                out1.setState(false);
                txIndex = 0;
                txSeq = 0;
                return;
            }

            // Select the next bit
            // right shift the buffer and boolean AND with 1 to get the next bit
            txBit =(byte)((txBuffer>>txIndex)&1);
            if (txBit == 1){
                out1.setState(true);
            }else if (txBit == 0){
                out1.setState(false);
            }

            // Increment the index.
            txIndex++;

            // must update the time reference each time through.
            txLastTime = timer.milliseconds();
        }
    }

    private void rxIdle(){
        // if the input goes high and the Flag is not set,
        // update the time reference, set the Flag (start detected)
        if(in1.getState() && !rxFlag){
            rxLastTime = timer.milliseconds();
            rxFlag = true;
            rxPeriod = 2.5* period;
        }
        // if the Flag is set (start detected) and the input goes LOW, RESET
        if(rxFlag && !in1.getState()){
            rxFlag = false;
        }
        // if the Flag is set (start detected) for more then the defined time,
        // then, trigger the start of the rx sequence.
        if(rxFlag && timer.milliseconds()-rxLastTime > rxPeriod){
            rxIndex = 0;
            rxPeriod = 0.5*period;
            rxLastTime = timer.milliseconds();
            rxFlag = false;
            rxBuffer = 0;
            rxSeq++;
        }

    }

    private void rxSetTiming(){
        // wait until the input goes LOW and set the time reference.
        if(!in1.getState() && !rxFlag){
            rxLastTime = timer.milliseconds();
            rxFlag = true;
        }
        if(timer.milliseconds() - rxLastTime > rxPeriod){
            rxPeriod = period;
            rxLastTime = timer.milliseconds();
            rxSeq++;
        }
    }

    private void rxActive(){
        // When each period elapses, read the next bit
        if(timer.milliseconds()-rxLastTime > rxPeriod){

            // reset the rx Sequence when 8 bits have been received.
            if(rxIndex == 8){
                rxSeq = 0;
                rxIndex = 0;
                return;
            }
            if(in1.getState()){
                rxBit = 1;
            }else{
                rxBit = 0;
            }
            // shift the bit left by the Index
            rxAddValue = (byte)(rxBit<<rxIndex);
            // boolean AND the new bit to the buffer.
            rxBuffer = (byte)(rxBuffer | rxAddValue);
            // increment the index and update the time reference.
            rxIndex++;
            rxLastTime = timer.milliseconds();
        }

    }*/


}
