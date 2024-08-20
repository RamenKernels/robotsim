package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class PhredBot {
    DcMotor front_left_motor;
    DcMotor front_right_motor;
    DcMotor back_left_motor;
    DcMotor back_right_motor;
    DcMotor arm_motor;

    Servo hand_servo;

    BNO055IMU imu;

    double adjusted_yaw;
    double zeroed_yaw;
    double initial_yaw;

    public void initialize(HardwareMap hardwareMap) {
        front_left_motor = hardwareMap.dcMotor.get("front_left_motor");
        front_right_motor = hardwareMap.dcMotor.get("front_right_motor");
        back_left_motor = hardwareMap.dcMotor.get("back_left_motor");
        back_right_motor = hardwareMap.dcMotor.get("back_right_motor");
        arm_motor = hardwareMap.dcMotor.get("arm_motor");

        hand_servo = hardwareMap.servo.get("hand_servo");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelerationIntegrationAlgorithm = null;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationData = null;
        parameters.calibrationDataFile = "";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        resetYaw();
    }

    public void drive(double drive, double strafe, double turn) {
        front_left_motor.setPower(drive - turn - strafe);
        front_right_motor.setPower(-(drive + turn + strafe));
        back_left_motor.setPower(drive - turn + strafe);
        back_right_motor.setPower(-(drive + turn - strafe));
    }

    public void driveOriented(double drive, double strafe, double turn) {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // get orientation relative to initial position
        adjusted_yaw = orientation.firstAngle-initial_yaw;
        zeroed_yaw = -initial_yaw+orientation.firstAngle;

        turn = -turn; // turn needs to be reversed idk

        double theta = Math.atan2(strafe, drive) * 180/Math.PI; // the angle of the joystick -180 -> 180
        double real_theta = (360 - zeroed_yaw) + theta; // the angle of the joystick 360 degrees

        double power = Math.hypot(drive, strafe);

        double sin = Math.sin((real_theta * (Math.PI / 180)) - (Math.PI / 4)); // the real directions to give to the motors
        double cos = Math.cos((real_theta * (Math.PI / 180)) - (Math.PI / 4)); // this too

        double max_sin_cos = Math.max(Math.abs(sin), Math.abs(cos));

        double front_left = (power * cos / max_sin_cos + turn);
        double front_right = (power * sin / max_sin_cos - turn);
        double back_left = (power * sin / max_sin_cos + turn);
        double back_right = (power * cos / max_sin_cos - turn);

        front_left_motor.setPower(front_left);
        front_right_motor.setPower(-front_right);
        back_left_motor.setPower(back_left);
        back_right_motor.setPower(-back_right);
    }

    public void moveArm(double speed) {
        arm_motor.setPower(speed);
    }

    public void openHand() {
        hand_servo.setPosition(0);
    }

    public void closeHand() {
        hand_servo.setPosition(1);
    }

    public void resetYaw() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        initial_yaw = orientation.firstAngle + 90;
    }

    public double getYaw() {
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return orientation.firstAngle;
    }

    public double getInitialYaw() {
        return initial_yaw;
    }
}
