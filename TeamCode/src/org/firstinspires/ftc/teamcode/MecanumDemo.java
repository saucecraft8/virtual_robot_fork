package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Example OpMode. Demonstrates use of gyro, color sensor, encoders, and telemetry.
 *
 */
@TeleOp(name = "Mecanum TeleOp", group = "MecanumBot")
public class MecanumDemo extends LinearOpMode {

    DcMotorEx FL , BL , FR , BR;
    BNO055IMU IMU;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    final float sqrt2 = 1.41421356237f;

    public void runOpMode(){
        BL = hardwareMap.get(DcMotorEx.class, "back_left_motor");
        FL = hardwareMap.get(DcMotorEx.class, "front_left_motor");
        FR = hardwareMap.get(DcMotorEx.class, "front_right_motor");
        BR = hardwareMap.get(DcMotorEx.class, "back_right_motor");

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
        while (opModeIsActive()){


            telemetry.addData("Status", "Running");

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double angle = Math.atan2(y , x) + getAngle();
            double transform = Math.sqrt(x * x + y * y);

            telemetry.addData("transform", transform);

            FL.setPower((transform * ((Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2);
            FR.setPower((transform * ((-Math.sin(angle) - Math.cos(angle)) / sqrt2) - rx) / 2);
            BL.setPower((transform * ((Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2);
            BR.setPower((transform * ((-Math.sin(angle) + Math.cos(angle)) / sqrt2) - rx) / 2);

            telemetry(angle);
        }
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
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
        telemetry.addData("FL Power", FL.getPower());
        telemetry.addData("FL Raw" , (Math.sin(angle) - Math.cos(angle)) / sqrt2);

        telemetry.addData("FR Power", FR.getPower());
        telemetry.addData("FR Raw" , (-Math.sin(angle) - Math.cos(angle)) / sqrt2);

        telemetry.addData("BL Power", BL.getPower());
        telemetry.addData("BL Raw" , (Math.sin(angle) + Math.cos(angle)) / sqrt2);

        telemetry.addData("BR Power", BR.getPower());
        telemetry.addData("BR Raw" , (-Math.sin(angle) + Math.cos(angle)) / sqrt2);


        telemetry.addData("Angle", Math.toDegrees(angle));
        telemetry.addData("AngleM", getAngle());

        telemetry.update();
    }
}
