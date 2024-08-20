package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Main TeleOp", group =  "Main")
public class MainTeleOp extends LinearOpMode {
    boolean is_field_oriented = false;

    boolean previousStart = false; // Flag for if gamepad1.start was pressed on the last frame; for rising edge detector
    boolean startToggle = false; // The actual toggle output of the start button

    DcMotor front_left_motor;
    DcMotor front_right_motor;
    DcMotor back_left_motor;
    DcMotor back_right_motor;

    DcMotor arm_motor;
    Servo hand_servo;

    PhredBot bot = new PhredBot();

    public void runOpMode() {
        bot.initialize(hardwareMap);

        front_left_motor = hardwareMap.dcMotor.get("front_left_motor");
        front_right_motor = hardwareMap.dcMotor.get("front_right_motor");
        back_left_motor = hardwareMap.dcMotor.get("back_left_motor");
        back_right_motor = hardwareMap.dcMotor.get("back_right_motor");

        arm_motor = hardwareMap.dcMotor.get("arm_motor");
        hand_servo = hardwareMap.servo.get("hand_servo");

        waitForStart();

        while (opModeIsActive()) {
            // This will make it easier to make small adjustments in position and rotation
            double adjusted_drive = Math.abs(gamepad1.left_stick_y) * gamepad1.left_stick_y;
            double adjusted_turn = Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x;
            double adjusted_strafe = Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x;

            if (gamepad1.start && !previousStart) {
                startToggle = !startToggle;
            }
            previousStart = gamepad1.start;

            is_field_oriented = startToggle;

            if (is_field_oriented) {
                bot.driveOriented(adjusted_drive, adjusted_strafe, adjusted_turn);
            } else {
                bot.drive(adjusted_drive, adjusted_strafe, adjusted_turn);
            }

            if (gamepad1.y) {
                bot.resetYaw();
            }

            bot.moveArm(gamepad1.left_trigger - gamepad1.right_trigger);
            if (gamepad1.dpad_left) {
                bot.closeHand();
            } else if (gamepad1.dpad_right) {
                bot.openHand();
            }

            telemetry();
        }
    }

    public void telemetry() {
        telemetry.addData("Field Oriented: ", is_field_oriented);
        telemetry.addData("Yaw: ", bot.getYaw());
        telemetry.addData("InitialYaw: ", bot.getInitialYaw());
        telemetry.update();
    }
}
