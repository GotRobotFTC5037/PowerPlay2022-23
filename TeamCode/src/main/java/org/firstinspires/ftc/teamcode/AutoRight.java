package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Right", group = "auto")
public class AutoRight extends LinearOpMode {
    Hardware robot = new Hardware();
    AutoDetectionJunction.JunctionAnalysisPipeline junctionPipeline;
    AutoDetectionRight.SignalDeterminationPipeline signalPipeline;
    //28 * 20 / (2ppi * 4.125)
    Double width = 16.0; //inches
    double cpr = 751.8; //counts per rotation
    double gearratio = 1;
    Double diameter = 3.77953;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.714;
    //
    double wrist_UP = 0.1;
    double wrist_MID = 0.625;
    double wrist_DOWN = 1;

    double grip_OPEN = 0;
    double grip_CLOSED = 0.4;
    //
    Double conversion = cpi * bias;
    Boolean exit = false;
    //
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    public void runOpMode() {

        robot.init(hardwareMap);

        initGyro();

        robot.frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        robot.backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        robot.gripper.setPosition(grip_OPEN);
        robot.wrist.setPosition(wrist_UP);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.upperLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        signalPipeline = new AutoDetectionRight.SignalDeterminationPipeline();
        robot.webcam.setPipeline(signalPipeline);
        robot.webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        robot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
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
                robot.webcam.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("Camera unable to open,", "will run left");
                telemetry.update();
            }
        });

        robot.webcam2.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                robot.webcam2.startStreaming(800, 600, OpenCvCameraRotation.UPRIGHT);

                junctionPipeline = new AutoDetectionJunction.JunctionAnalysisPipeline();
                robot.webcam2.setPipeline(junctionPipeline);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("Camera unable to open,", "will run left");
                telemetry.update();
            }
        });


        // Tell telemetry to update faster than the default 250ms period :)
        telemetry.setMsTransmissionInterval(20);

        telemetry.addData("Status", "Ready to run");
        while (!isStarted()) {
            if (isStopRequested()) return;
            {
                telemetry.addData("Position", signalPipeline.getAnalysis());
                telemetry.addData("FPS", String.format("%.2f", robot.webcam.getFps()));
                telemetry.addData("Pipeline time ms", robot.webcam.getPipelineTimeMs());
                telemetry.update();
            }
        }

        waitForStart();

        robot.webcam.closeCameraDevice();
        telemetry.addData("Running", signalPipeline.getAnalysis());
        telemetry.update();

        robot.wrist.setPosition(wrist_MID);
        sleep(200);
        robot.gripper.setPosition(grip_CLOSED);
        sleep(100);
        robot.gripperWheel.setPower(1);
        sleep(200);
        robot.gripperWheel.setPower(.075);





        moveToPosition(2,.3);
        strafeToPosition(-2, .3);
        robot.upperLift.setTargetPosition(2940);
        robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperLift.setPower(.9);
        robot.lift.setTargetPosition(600);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(.9);
        turnWithGyro(90,.3);
        brake();
        strafeToPosition(-30, .5);
        sleep(150);
        getJunctionPosition(.3);
        robot.upperLift.setTargetPosition(2040);
        robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperLift.setPower(.9);
        sleep(300);
        robot.gripper.setPosition(grip_OPEN);
        sleep(200);
        robot.upperLift.setTargetPosition(2940);
        robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperLift.setPower(.9);
        sleep(300);
        robot.wrist.setPosition(wrist_UP);
        strafeToPosition(-12,.5);
        moveToPosition(10,.5);
        robot.upperLift.setTargetPosition(1940);
        robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperLift.setPower(.9);
        robot.lift.setTargetPosition(0);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(.9);
        lineUp();
        robot.wrist.setPosition(wrist_MID);
        robot.gripper.setPosition(grip_OPEN);
        moveToPosition(15,.5);
        robot.gripper.setPosition(grip_CLOSED);
        sleep(500);
        robot.upperLift.setTargetPosition(2940);
        robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperLift.setPower(.85);
        robot.lift.setTargetPosition(2080);
        robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lift.setPower(.85);
        moveToPosition(-20,.5);
        turnWithGyro(90,.33);
        brake();
        moveToPosition(-1,.3);
        strafeToPosition(18,.5);
        brake();
        sleep(200);
        getJunctionPosition(.3);
        sleep(300);
        robot.upperLift.setTargetPosition(2040);
        robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperLift.setPower(.9);
        sleep(300);
        robot.gripper.setPosition(grip_OPEN);
        sleep(300);
        robot.upperLift.setTargetPosition(2940);
        robot.upperLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.upperLift.setPower(.9);
        sleep(300);
        robot.wrist.setPosition(wrist_UP);
        sleep(200);
        turnWithGyro(90,-.33);
        moveToPosition(25,.5);
        lineUp();


        switch (signalPipeline.getAnalysis()) {
            case LEFT: {

                return;
            }
            case CENTER: {


                return;
            }
            case RIGHT: {

                return;
            }
        }

    }

    public void brake() {
        robot.frontLeftDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        robot.frontRightDrive.setPower(0);
        robot.backRightDrive.setPower(0);
    }

    public void lineUp() {
        while (robot.leftColor.red() < 75) {
        strafeToPositionNoBrake(.5,1);
        }
        brake();
        strafeToPosition(-1,.3);
    }

    public void getJunctionPosition(double speed) {
        double junctionWidth = 1;
        double junctionPosition = 0;
        double pointsPerInch;
        ArrayList<AutoDetectionJunction.JunctionAnalysisPipeline.AnalyzedJunction> junctions = junctionPipeline.getDetectedStones();

        for (AutoDetectionJunction.JunctionAnalysisPipeline.AnalyzedJunction junction : junctions) {
            if (junction.area > junctionPosition) {
                junctionPosition = junction.position;
                junctionWidth = junction.width;
            }
        }
        telemetry.addData("Position", junctionPosition);
        telemetry.addData("Width", junctionWidth);
        telemetry.addData("Distance to move", (junctionPosition - 1075) / junctionWidth);
        telemetry.update();
        strafeToPosition((junctionPosition - 820) / 100, speed);
    }

    public void moveToPositionWhile(double inches, double speed, boolean If) {

        int move = (int) (Math.round(inches * conversion));

        robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() + move);
        robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + move);
        robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() + move);
        robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() + move);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.backRightDrive.setPower(speed);

        while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy()) {
            if (exit || If) {
                robot.frontRightDrive.setPower(0);
                robot.frontLeftDrive.setPower(0);
                robot.backRightDrive.setPower(0);
                robot.backLeftDrive.setPower(0);
                return;
            }
        }
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        return;
    }

    public void moveToPosition(double inches, double speed) {

        int move = (int) (Math.round(inches * conversion));

        robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() + move);
        robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + move);
        robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() + move);
        robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() + move);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(speed);
        robot.backLeftDrive.setPower(speed);
        robot.frontRightDrive.setPower(speed);
        robot.backRightDrive.setPower(speed);

        while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy()) {
            if (exit) {
                robot.frontRightDrive.setPower(0);
                robot.frontLeftDrive.setPower(0);
                robot.backRightDrive.setPower(0);
                robot.backLeftDrive.setPower(0);
                return;
            }
        }
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        return;
    }

    //
    /*
    This function uses the Expansion Hub IMU Integrated Gyro to turn a precise number of degrees (+/- 5).
    Degrees should always be positive, make speedDirection negative to turn left.
     */
    public void turnWithGyro(double degrees, double speedDirection) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;//make this negative
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();

        double first;
        double second;
        if (speedDirection > 0) {//set target positions
            //turn right
            if (degrees > 10) {
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            } else {
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
        } else {
            //turn left
            if (degrees > 10) {
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            } else {
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
        }
        //go to position
        double firsta = convertify(first - 5);//175
        double firstb = convertify(first + 5);//-175

        turnWithEncoder(speedDirection);

        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        } else {

            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }

        double seconda = convertify(second - 5);//175
        double secondb = convertify(second + 5);//-175

        turnWithEncoder(speedDirection / 3);

        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
        }

        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed) {

        int move = (int) (Math.round(inches * conversion));

        robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() - move);
        robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + move);
        robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() + move);
        robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() - move);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(speed);
        robot.backLeftDrive.setPower(-speed);
        robot.frontRightDrive.setPower(-speed);
        robot.backRightDrive.setPower(speed);

        while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy()) {
            if (exit) {
                robot.frontRightDrive.setPower(0);
                robot.frontLeftDrive.setPower(0);
                robot.backRightDrive.setPower(0);
                robot.backLeftDrive.setPower(0);
                return;
            }
        }
        robot.frontRightDrive.setPower(0);
        robot.frontLeftDrive.setPower(0);
        robot.backRightDrive.setPower(0);
        robot.backLeftDrive.setPower(0);
        return;
    }

    public void strafeToPositionNoBrake(double inches, double speed) {

        int move = (int) (Math.round(inches * conversion));

        robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() - move);
        robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + move);
        robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() + move);
        robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() - move);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontLeftDrive.setPower(speed);
        robot.backLeftDrive.setPower(-speed);
        robot.frontRightDrive.setPower(-speed);
        robot.backRightDrive.setPower(speed);

        while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy()) {
            if (exit) {
                return;
            }
        }
        return;
    }

    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }

    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
    public void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        //
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input) {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        robot.frontLeftDrive.setPower(input);
        robot.backLeftDrive.setPower(input);
        robot.frontRightDrive.setPower(-input);
        robot.backRightDrive.setPower(-input);
    }
    //
}
