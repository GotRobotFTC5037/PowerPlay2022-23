package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="Calibrate", group="chad")
public class Calibrate extends LinearOpMode {
    Hardware robot = new Hardware();
    //
    //Calculate encoder conversion
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 1;
    Double diameter = 3.77953;
    Double cpi = (cpr * gearratio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference
    Double bias = 0.714;//adjust until your robot goes 20 inches
    //
    Double conversion = cpi * bias;
    //
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStartify();

        moveToPosition(24, .2);//Don't change this line, unless you want to calibrate with different speeds

    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        //
        if (inches < 5) {
            int move = (int) (Math.round(inches * conversion));
            //
            robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + move);
            robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() + move);
            robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() + move);
            robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() + move);
            //
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            robot.frontLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(speed);
            robot.backLeftDrive.setPower(speed);
            robot.backRightDrive.setPower(speed);
            //
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy()) {
            }
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
        } else {
            int move1 = (int) (Math.round((inches - 5) * conversion));
            int movefl2 = robot.frontLeftDrive.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movefr2 = robot.frontRightDrive.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebl2 = robot.backLeftDrive.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebr2 = robot.backRightDrive.getCurrentPosition() + (int) (Math.round(inches * conversion));
            //
            robot.frontLeftDrive.setTargetPosition(robot.frontLeftDrive.getCurrentPosition() + move1);
            robot.frontRightDrive.setTargetPosition(robot.frontRightDrive.getCurrentPosition() + move1);
            robot.backLeftDrive.setTargetPosition(robot.backLeftDrive.getCurrentPosition() + move1);
            robot.backRightDrive.setTargetPosition(robot.backRightDrive.getCurrentPosition() + move1);
            //
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            robot.frontLeftDrive.setPower(speed);
            robot.frontRightDrive.setPower(speed);
            robot.backLeftDrive.setPower(speed);
            robot.backRightDrive.setPower(speed);
            //
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy()) {
            }
            //
            robot.frontLeftDrive.setTargetPosition(movefl2);
            robot.frontRightDrive.setTargetPosition(movefr2);
            robot.backLeftDrive.setTargetPosition(movebl2);
            robot.backRightDrive.setTargetPosition(movebr2);
            //
            robot.frontLeftDrive.setPower(.1);
            robot.frontRightDrive.setPower(.1);
            robot.backLeftDrive.setPower(.1);
            robot.backRightDrive.setPower(.1);
            //
            while (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy() && robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy()) {
            }
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);
        }
        return;
    }
    /*
    A tradition within the Thunder Pengwins code, we always start programs with waitForStartify,
    our way of adding personality to our programs.
     */
    public void waitForStartify() {
        waitForStart();
    }
}
