package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.function.Function;


@Autonomous(name = "Path Correction", group = "MecanumBot")
public class MecanumAutonPathCorrection extends LinearOpMode {

    DcMotorEx FL , FR , BL , BR , odoR , odoL , odoX;
    MecOdo odo;
    BNO055IMU IMU;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    double velocity , lastX , lastY , lastT , t = 0; // The magnitude of the robot's calculated velocity based on deadwheels
    double veloSum , veloCount;
    final float sqrt2 = 1.41421356237f;
    final float vMAX = 29.2f;
    final float vMAXCoeff = 1.0f; // The correlation between velocity and distance; 1 means 1 foot from the expected point results in a velocity equal to that of the current towards the expected

    public void runOpMode(){
        FL = hardwareMap.get(DcMotorEx.class , "front_left_motor");
        FR = hardwareMap.get(DcMotorEx.class , "front_right_motor");
        BL = hardwareMap.get(DcMotorEx.class , "back_left_motor");
        BR = hardwareMap.get(DcMotorEx.class , "back_right_motor");

        odoR = hardwareMap.get(DcMotorEx.class , "enc_right");
        odoL = hardwareMap.get(DcMotorEx.class , "enc_left");
        odoX = hardwareMap.get(DcMotorEx.class , "enc_x");
        odo  = new MecOdo(new DcMotorEx[]{odoR , odoL , odoX});
        odo.resetOdometry(0, 0, 0); // getPose() -> [x , y , h]

        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");

        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "Who cares.";
        IMU.initialize(parameters);

        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        telemetry.addData("Press Start When Ready","");
        telemetry.update();

        waitForStart();

        // getRuntime() returns seconds opModes have been running
        while (opModeIsActive()){
            t = getRuntime();
            odo.updateOdometry();

            Function<Double , Double> first = new Function<Double, Double>() {
                @Override
                public Double apply(Double aDouble) {
                    return 3 * (aDouble - 2) * (aDouble - 2);
                }
            };

            moveOff();
            // moveDeg(Math.toDegrees(Math.atan2(3 * (t - 2) * (t - 2) , 1) - (getAngle() - Math.atan2(3 * (t-2) * (t - 2) , 1) + Math.PI)), 1 , 1);
            // moveXY(1 , 3 * (t-2) * (t - 2) , 1 , 1 );

            calcMotionConstants(t);

            telemetry(getAngle());
        }

        telemetry(getAngle());
        moveOff();
    }

    private void resetAngle()
    {
        lastAngles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the IMU works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Prints telemetry information
     */
    private void telemetry(double angle) {
        telemetry.addData("Odometry", "\nx: " + odo.getPose()[0] + "\ny: " + odo.getPose()[1] + "\nh: " + Math.toDegrees(odo.getPose()[2]));
        telemetry.addData("Angle", Math.toDegrees(angle));
        telemetry.addData("AngleM", getAngle());

//        telemetry.addData("Velocity" , velocity);
//        telemetry.addData("X diff" , odo.getPose()[0] - lastX);
//        telemetry.addData("lastX" , lastX);
//        telemetry.addData("Y diff" , odo.getPose()[1] - lastY);
//        telemetry.addData("lastY" , lastY);
//        telemetry.addData("time diff" , t - lastT);
//        telemetry.addData("lastT" , lastT);


        telemetry.addData("FL Power", FL.getPower());
        // telemetry.addData("FL Raw" , (Math.sin(angle) - Math.cos(angle)) / sqrt2);

        telemetry.addData("FR Power", FR.getPower());
        // telemetry.addData("FR Raw" , (-Math.sin(angle) - Math.cos(angle)) / sqrt2);

        telemetry.addData("BL Power", BL.getPower());
        // telemetry.addData("BL Raw" , (Math.sin(angle) + Math.cos(angle)) / sqrt2);

        telemetry.addData("BR Power", BR.getPower());
        // telemetry.addData("BR Raw" , (-Math.sin(angle) + Math.cos(angle)) / sqrt2);

        telemetry.addData("Runtime" , getRuntime());

        telemetry.update();
    }

    // Rotational motion + function path is SUPER scuffed
    private void moveXY(double x , double y , double rx) {
        y *= -1;
        double transform = Math.sqrt(x * x + y * y);
        double angle = Math.atan2(y , x) + getAngle();
        double[] power = {
                (transform * ((Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2
        };

        double max = maxAbs(power);
        if(transform >= 1 || max > 1) {
            for (int i = 0; i < power.length; i++)
                power[i] = power[i] / max;
        }

        FL.setPower(power[0]);
        FR.setPower(power[1]);
        BL.setPower(power[2]);
        BR.setPower(power[3]);
    }

    // fs should never be greater than 1
    private double[] getMoveXY(double x , double y , double rx , double fs) {
        y *= -1;
        if(fs > 1) fs = 1;

        double angle = Math.atan2(y , x) + getAngle();
        double[] power = {
                ((fs * (Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                ((fs * (-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                ((fs * (Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2,
                ((fs * (-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2
        };

        double max = maxAbs(power);
        if(fs >= 1 || max > 1)
            for (int i = 0; i < power.length; i++)
                power[i] = power[i] / max;

        return power;
    }

    /**
     * Calculates and returns the power for each motor to move at a certain heading, speed, and angular velocity
     *
     * @param degrees   The heading
     * @param speed     The relative linear velocity (max 1)
     * @param rx        The relative angular velocity (max 1)
     * @return          An array of motor powers [FL , FR , BL , BR]
     */
    private double[] getMoveDeg(double degrees , double speed , double rx) {
        double x = speed * Math.cos(Math.toRadians(degrees));
        double y = speed * Math.sin(Math.toRadians(degrees));
        // Change angle back to positive if it starts being weird
        double angle = -Math.toRadians(degrees) + getAngle();
        double[] power = {
                (speed * ((Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (speed * ((-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (speed * ((Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2,
                (speed * ((-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2
        };

        double max = maxAbs(power);
        if(speed >= 1 || max > 1) {
            for (int i = 0; i < power.length; i++)
                power[i] = power[i] / max;
        }

//        FL.setPower(power[0]);
//        FR.setPower(power[1]);
//        BL.setPower(power[2]);
//        BR.setPower(power[3]);
        return power;
    }

    private void moveDeg(double degrees , double speed , double rx , Function<Double , Double> func) {
        // Change angle back to positive if it starts being weird
        double angle = -Math.toRadians(degrees) + getAngle();
        double[] power = {
                ((speed * ((Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2) + pathCorrection(func , speed , rx)[0],
                ((speed * ((-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2) + pathCorrection(func , speed , rx)[1],
                ((speed * ((Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2) + pathCorrection(func , speed , rx)[2],
                ((speed * ((-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2) + pathCorrection(func , speed , rx)[3]
        };

        double max = maxAbs(power);
        if(speed >= 1 || max > 1) {
            for (int i = 0; i < power.length; i++)
                power[i] = power[i] / max;
        }

        FL.setPower(power[0]);
        FR.setPower(power[1]);
        BL.setPower(power[2]);
        BR.setPower(power[3]);
    }

    private void moveOff() {
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);
    }

    private double maxAbs(double[] nums) {
        double max = Math.abs(nums[0]);
        for(int i = 1 ; i < nums.length ; i++)
            if(Math.abs(nums[i]) > max)
                max = Math.abs(nums[i]);
        return max;
    }

    private double ddx(Function<Double , Double> func, double x) {
        return (func.apply(x + 0.0001) + func.apply(x - 0.0001)) / 0.0002;
    }

    /**
     * Updates lastX, lastY, velocity, and lastT
     *
     * @param t     Time
     */
    private void calcMotionConstants(double t) {
        if(!(Math.floor(t * 1000) % 10 == 0)) {
            return;
        }

        double tempV = (odo.getPose()[0] - lastX) * (odo.getPose()[0] - lastX);
        tempV += (odo.getPose()[1] - lastY) * (odo.getPose()[1] - lastY);
        tempV = Math.sqrt(tempV) / (t - lastT);
        veloSum += tempV;
        veloCount++;

        if(veloCount > 9) {
            velocity = veloSum / veloCount;
            veloSum = 0;
            veloCount = 0;
        }

        if(FL.getPower() != 0 ||
            FR.getPower() != 0 ||
            BL.getPower() != 0 ||
            BR.getPower() != 0) {
            lastX = lastX != odo.getPose()[0] ? odo.getPose()[0] - 0.0001 : lastX - 0.0001;
            lastY = lastY != odo.getPose()[1] ? odo.getPose()[1] - 0.0001 : lastY - 0.0001;
        }else {
            lastX = odo.getPose()[0];
            lastY = odo.getPose()[1];
        }

        lastT = t != lastT ? t - 0.0001 : lastT - 0.0001;
    }

    /**
     * NEED TO KEEP TRACK OF WHERE THE FUNCTION WAS STARTED AND NORMALIZE THE POSITION TO 0
     * (Doens't matter for this trial since odometry always normalizes to 0 at the beginning of
     * the code, but will matter A LOT when adding more than 1 path)
     */
    private double[] pathCorrection(Function<Double , Double> func , double speed , double rx) {
        double y = func.apply(t);
        double angle = (y - odo.getPose()[1]) / (t - odo.getPose()[0]);
        angle = Math.atan2(angle , 1);
        double transform = speed * (Math.sqrt((y - odo.getPose()[1]) * (y - odo.getPose()[1]) + (t - odo.getPose()[0]) * (t - odo.getPose()[0])) / vMAXCoeff);
        return new double[]{
                (transform * ((Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2
        };
    }
}
