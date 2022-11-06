package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;


@Autonomous(name = "Mecanum Auton", group = "MecanumBot")
public class MecanumAutonTesting extends LinearOpMode {

    DcMotorEx FL , FR , BL , BR , odoR , odoL , odoX;
    MecOdo odo;
    BNO055IMU IMU;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    final float sqrt2 = 1.41421356237f;

    public void runOpMode(){
        FL = hardwareMap.get(DcMotorEx.class , "front_left_motor");
        FR = hardwareMap.get(DcMotorEx.class , "front_right_motor");
        BL = hardwareMap.get(DcMotorEx.class , "back_left_motor");
        BR = hardwareMap.get(DcMotorEx.class , "back_right_motor");

        odoR = hardwareMap.get(DcMotorEx.class , "enc_right");
        odoL = hardwareMap.get(DcMotorEx.class , "enc_left");
        odoX = hardwareMap.get(DcMotorEx.class , "enc_x");
        odo  = new MecOdo(new DcMotorEx[]{odoR , odoL , odoX});
        odo.resetOdometry(0, 0, 0);

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
            odo.updateOdometry();

            moveDeg(-45 , 0.5 , 0.4);

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

        telemetry.addData("FL Power", FL.getPower());
        telemetry.addData("FL Raw" , (Math.sin(angle) - Math.cos(angle)) / sqrt2);

        telemetry.addData("FR Power", FR.getPower());
        telemetry.addData("FR Raw" , (-Math.sin(angle) - Math.cos(angle)) / sqrt2);

        telemetry.addData("BL Power", BL.getPower());
        telemetry.addData("BL Raw" , (Math.sin(angle) + Math.cos(angle)) / sqrt2);

        telemetry.addData("BR Power", BR.getPower());
        telemetry.addData("BR Raw" , (-Math.sin(angle) + Math.cos(angle)) / sqrt2);

        telemetry.update();
    }

    private void moveXY(double x , double y , double rx) {
        double transform = Math.sqrt(x * x + y * y);
        double angle = Math.atan2(y , x) + getAngle();
        double[] power = {
                (transform * ((Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2
        };

        /*
        double max = max(power);
        if(transform >= 1 || max > 1) {
            for (int i = 0; i < power.length; i++)
                power[i] = power[i] / max;
        }
         */

        FL.setPower(power[0]);
        FR.setPower(power[1]);
        BL.setPower(power[2]);
        BR.setPower(power[3]);
    }

    private void moveDeg(double degrees , double speed , double rx) {
        double x = speed * Math.cos(Math.toRadians(degrees));
        double y = speed * Math.sin(Math.toRadians(degrees));
        double transform = Math.sqrt(x * x + y * y);
        double angle = Math.toRadians(degrees) + getAngle();
        double[] power = {
                (transform * ((Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2,
                (transform * ((-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2
        };
        /*
        double max = max(power);
        if(transform >= 1 || max > 1) {
            for (int i = 0; i < power.length; i++)
                power[i] = power[i] / max;
        }
         */

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

    private double max(double[] nums) {
        double max = nums[0];
        for(int i = 1 ; i < nums.length ; i++)
            if(nums[i] > max)
                max = nums[i];
        return max;
    }
}
