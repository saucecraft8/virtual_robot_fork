package teamcode;

import virtual_robot.hardware.DCMotor;
import virtual_robot.controller.LinearOpMode;
import virtual_robot.time.ElapsedTime;

/**
 * Example OpMode. Controls robot using left joystick, with arcade drive.
 */
public class ArcadeDrive extends LinearOpMode {

    public void runOpMode(){
        DCMotor left = hardwareMap.dcMotor.get("left_motor");
        DCMotor right = hardwareMap.dcMotor.get("right_motor");
        left.setDirection(DCMotor.Direction.REVERSE);
        telemetry.addData("Press Start When Ready","");
        telemetry.update();
        waitForStart();
        ElapsedTime et = new ElapsedTime();
        double oldMillis = 0.0;
        double maxLoopMillis = 0.0;
        while (opModeIsActive()){
            double millis = et.milliseconds();
            double loopMillis = millis - oldMillis;
            if (loopMillis > maxLoopMillis){
                System.out.println("Max Loop Millis: " + loopMillis);
                maxLoopMillis = loopMillis;
            }
            oldMillis = millis;
            float fwd = -gamePad1.left_stick_y;
            float steer = gamePad1.left_stick_x;
            left.setPower(.5 * fwd + .2 * steer);
            right.setPower(0.5 * fwd - .2 * steer);
            }
        left.setPower(0);
        right.setPower(0);
    }
}
