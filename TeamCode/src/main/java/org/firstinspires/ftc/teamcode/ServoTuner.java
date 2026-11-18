package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * Find the right positions for the flower bar and the stopper.
 *   D-pad left/right : choose what to move (flower f1+f2, f1 only, f2 only, stopper ss)
 *   D-pad up/down    : +/- 0.01        bumpers: +/- 0.10
 * A servo is not sent anything until you nudge it, so nothing jumps at start.
 * Directions match HamzaTeli2 (f2 REVERSE), so the numbers shown can be copied straight into it.
 * Careful with "f1 only"/"f2 only": the two flower servos share one bar, so use small steps.
 */
@TeleOp(name = "ServoTuner", group = "Test")
public class ServoTuner extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo[] servos = {
                hardwareMap.get(Servo.class, "f1"),
                hardwareMap.get(Servo.class, "f2"),
                hardwareMap.get(Servo.class, "ss")};
        servos[0].setDirection(Servo.Direction.FORWARD);
        servos[1].setDirection(Servo.Direction.REVERSE);
        servos[2].setDirection(Servo.Direction.FORWARD);

        String[] modes = {"FLOWER (f1+f2 together)", "f1 only", "f2 only", "STOPPER ss"};
        int[][] servosInMode = {{0, 1}, {0}, {1}, {2}};
        double[] pos = {1.0, 1.0, 0.5};      // starting guesses: flower up, stopper closed (blocking)
        boolean[] sent = new boolean[3];
        int mode = 0;

        Gamepad prev = new Gamepad();
        telemetry.addLine("D-pad L/R choose, D-pad U/D +/-0.01, bumpers +/-0.10. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            Gamepad g = gamepad1;
            if (g.dpad_right && !prev.dpad_right) mode = (mode + 1) % modes.length;
            if (g.dpad_left && !prev.dpad_left)   mode = (mode + modes.length - 1) % modes.length;

            double step = 0;
            if (g.dpad_up && !prev.dpad_up)           step += 0.01;
            if (g.dpad_down && !prev.dpad_down)       step -= 0.01;
            if (g.right_bumper && !prev.right_bumper) step += 0.10;
            if (g.left_bumper && !prev.left_bumper)   step -= 0.10;
            prev.copy(g);

            if (step != 0) {
                int first = servosInMode[mode][0];
                double target = Range.clip(pos[first] + step, 0, 1);
                for (int i : servosInMode[mode]) {
                    pos[i] = target;
                    servos[i].setPosition(target);
                    sent[i] = true;
                }
            }

            telemetry.addData("Moving", modes[mode]);
            telemetry.addData("f1", sent[0] ? String.format("%.2f", pos[0]) : "not sent yet");
            telemetry.addData("f2", sent[1] ? String.format("%.2f", pos[1]) : "not sent yet");
            telemetry.addData("ss", sent[2] ? String.format("%.2f", pos[2]) : "not sent yet");
            telemetry.addLine("Write down: flower UP, flower DOWN, stopper BLOCKING, stopper LETS BALL THROUGH");
            telemetry.update();
        }
    }
}
