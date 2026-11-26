package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * AUTON SHOOT TEST: the robot does NOT drive. Put it at the shooting spot, load 4 balls, press a button.
 * Each test works like auton: the flywheel starts from a stop, shoots, then turns off.
 *
 *   CROSS    = A: PedroV3 shooting (open, transfer full power for 1.5 s)
 *   CIRCLE   = B: same, but feed for 3 s       (balls 3-4 just didn't get there in time?)
 *   SQUARE   = C: slow feed: 0.5 power for 3 s (spaces the balls out so the flywheel recovers)
 *   TRIANGLE = D: pulses: 4 x (wait for speed, open, feed FEED_MS, close, wait 300 ms)
 *
 *   D-pad UP / DOWN    = flywheel target +/- 50
 *   D-pad RIGHT / LEFT = FEED_MS for test D +/- 50
 *   Hold L2            = intake (to load balls), hold L1 = transfer backwards
 *
 * Write down for each test: how many balls scored, and the speeds on the screen.
 * "Speed when opened" low on later balls = the flywheel hadn't recovered.
 */
@TeleOp(name = "AutonShootTest", group = "Test")
public class AutonShootTest extends LinearOpMode {

    private static final double TRANSFER_POWER = -1.0;   // same as PedroV3
    private static final double SLOW_POWER     = -0.5;   // test C
    private static final double VELOCITY_TOLERANCE = 100;
    private static final double STEADY_MS = 150;
    private static final double SPINUP_TIMEOUT_MS = 2000;
    private static final double MIN_CLOSED_MS = 300;     // test D
    private static final double STOPPER_CLOSED_POS = 0.5;   // same as PedroV3 (the auton that works)
    private static final double STOPPER_OPEN_POS   = 0.0;

    private double target = 1350;    // PedroV3's speed
    private double feedMs = 250;     // test D: one pulse

    private DcMotor transferLeft, transferRight;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo stopperServo;
    private final ElapsedTime steadyTimer = new ElapsedTime();

    // results of the last test, shown on the screen
    private String lastTest = "none yet";
    private double minSpeed;
    private String openSpeeds = "";

    @Override
    public void runOpMode() {
        transferLeft  = hardwareMap.get(DcMotor.class, "tl");
        transferRight = hardwareMap.get(DcMotor.class, "tr");
        shooterLeft   = hardwareMap.get(DcMotorEx.class, "sl");
        shooterRight  = hardwareMap.get(DcMotorEx.class, "sr");
        stopperServo  = hardwareMap.get(Servo.class, "ss");
        Servo flower1 = hardwareMap.get(Servo.class, "f1");
        Servo flower2 = hardwareMap.get(Servo.class, "f2");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flower1.setDirection(Servo.Direction.FORWARD);
        flower2.setDirection(Servo.Direction.REVERSE);
        stopperServo.setDirection(Servo.Direction.FORWARD);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopperServo.setPosition(STOPPER_CLOSED_POS);
        flower1.setPosition(1.0);   // flower bar up, same as auton
        flower2.setPosition(1.0);

        telemetry.addLine("Ready. Press START, then a test button.");
        telemetry.update();
        waitForStart();

        boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;
        while (opModeIsActive()) {
            if (gamepad1.cross) {
                startTest("A: feed 1.5 s full power");
                feedOpen(TRANSFER_POWER, 1500);
                endTest();
            } else if (gamepad1.circle) {
                startTest("B: feed 3 s full power");
                feedOpen(TRANSFER_POWER, 3000);
                endTest();
            } else if (gamepad1.square) {
                startTest("C: feed 3 s at 0.5 power");
                feedOpen(SLOW_POWER, 3000);
                endTest();
            } else if (gamepad1.triangle) {
                startTest(String.format("D: 4 pulses of %.0f ms", feedMs));
                for (int i = 0; i < 4 && opModeIsActive(); i++) {
                    waitForSpeed(SPINUP_TIMEOUT_MS);
                    openSpeeds += String.format("%.0f ", shooterSpeed());
                    stopperServo.setPosition(STOPPER_OPEN_POS);
                    setTransfer(TRANSFER_POWER);
                    pause(feedMs);
                    setTransfer(0);
                    stopperServo.setPosition(STOPPER_CLOSED_POS);
                    pause(MIN_CLOSED_MS);
                }
                endTest();
            }

            // settings (one step per press)
            if (gamepad1.dpad_up && !lastUp)       target += 50;
            if (gamepad1.dpad_down && !lastDown)   target -= 50;
            if (gamepad1.dpad_right && !lastRight) feedMs += 50;
            if (gamepad1.dpad_left && !lastLeft)   feedMs = Math.max(50, feedMs - 50);
            lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
            lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;

            // loading balls between tests
            if (gamepad1.left_trigger > 0.1)  setTransfer(TRANSFER_POWER);
            else if (gamepad1.left_bumper)    setTransfer(-TRANSFER_POWER);
            else                              setTransfer(0);

            showStatus();
        }
        setTransfer(0);
        setShooterPower(0);
    }

    // Tests A, B, C: spin up, open, feed for ms at this power, close. (PedroV3's shootFor.)
    private void feedOpen(double power, double ms) {
        waitForSpeed(SPINUP_TIMEOUT_MS);
        openSpeeds += String.format("%.0f ", shooterSpeed());
        stopperServo.setPosition(STOPPER_OPEN_POS);
        setTransfer(power);
        pause(ms);
        setTransfer(0);
        stopperServo.setPosition(STOPPER_CLOSED_POS);
    }

    private void startTest(String name) {
        lastTest = name;
        openSpeeds = "";
        minSpeed = Double.MAX_VALUE;
        setShooterVelocity(target);
    }

    private void endTest() {
        setShooterPower(0);   // coast, like auton
        if (minSpeed == Double.MAX_VALUE) minSpeed = 0;
    }

    // Wait until the speed has been within VELOCITY_TOLERANCE for STEADY_MS, or timeoutMs passes.
    private void waitForSpeed(double timeoutMs) {
        ElapsedTime t = new ElapsedTime();
        steadyTimer.reset();
        while (opModeIsActive() && t.milliseconds() < timeoutMs) {
            if (Math.abs(shooterSpeed() - target) > VELOCITY_TOLERANCE) steadyTimer.reset();
            if (steadyTimer.milliseconds() >= STEADY_MS) return;
            showStatus();
        }
    }

    // Like sleep(), but keeps tracking the lowest speed and updating the screen.
    private void pause(double ms) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) {
            minSpeed = Math.min(minSpeed, shooterSpeed());
            showStatus();
        }
    }

    private void showStatus() {
        telemetry.addLine("X=A 1.5s | O=B 3s | []=C slow | /\\=D pulses");
        telemetry.addData("Target speed", "%.0f   (D-pad UP/DOWN)", target);
        telemetry.addData("Pulse FEED_MS (D)", "%.0f   (D-pad LEFT/RIGHT)", feedMs);
        telemetry.addData("Speed now", "%.0f", shooterSpeed());
        telemetry.addLine();
        telemetry.addData("Last test", lastTest);
        telemetry.addData("Speed when opened", openSpeeds.isEmpty() ? "-" : openSpeeds);
        telemetry.addData("Lowest speed while feeding", "%.0f", minSpeed == Double.MAX_VALUE ? 0 : minSpeed);
        telemetry.update();
    }

    private void setTransfer(double power) {
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    private void setShooterVelocity(double ticksPerSecond) {
        shooterLeft.setVelocity(ticksPerSecond);
        shooterRight.setVelocity(ticksPerSecond);
    }

    private double shooterSpeed() {
        return (shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2.0;
    }
}
