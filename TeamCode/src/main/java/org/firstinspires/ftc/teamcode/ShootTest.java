package org.firstinspires.ftc.teamcode;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.CommandBuilder;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

/*
 * SHOOT TEST: no driving, it only shoots. Load pollen, set the options during INIT, press START.
 * Uses the same Ivy commands as PedroV3, so whatever works here can be copied straight over.
 *
 * During INIT (gamepad1):
 *   X (cross)   = mode BLIND: PedroV3's current shootFor(): spin up once, open, feed for SHOOT_MS.
 *   O (circle)  = mode GATED: like HamzaTeli2's R2: feed only while the speed is steady, pause after each dip.
 *   D-pad up/down     = target speed +/- 50 ticks/s
 *   bumpers R/L       = feed time +/- 250 ms
 * Every 100 ms of the test is logged:  adb logcat | grep ShootTest
 */
@Autonomous(name = "ShootTest", group = "Test")
public class ShootTest extends LinearOpMode {

    // ===== Same settings as PedroV3 (the ones adjustable in INIT start here) =====
    private static final double TRANSFER_POWER = -1.0;
    private double targetVelocity = 1800.0;   // D-pad up/down in INIT
    private double shootMs = 1500;            // bumpers in INIT
    private static final double VELOCITY_TOLERANCE = 100;
    private static final double STEADY_MS = 150;
    private static final double SPINUP_TIMEOUT_MS = 2000;
    private static final double STOPPER_MIN_OPEN_MS = 400;   // GATED mode only (same as HamzaTeli2)
    private static final double FLOWER_UP_POS = 1.0;
    private static final double STOPPER_CLOSED_POS = 0.5;
    private static final double STOPPER_OPEN_POS   = 0.0;

    private DcMotor transferLeft, transferRight;
    private DcMotorEx shooterLeft, shooterRight;
    private Servo flowerServo1, flowerServo2, stopperServo;
    private final ElapsedTime steadyTimer = new ElapsedTime();
    private final ElapsedTime stopperTimer = new ElapsedTime();

    // ===== What the test measured (shown on telemetry and in the log) =====
    private String phase = "WAITING";
    private final ElapsedTime phaseTimer = new ElapsedTime();
    private double spinUpMs = -1;
    private boolean spinUpTimedOut = false;
    private double minSpeedWhileFeeding = Double.MAX_VALUE;
    private int speedDips = 0;          // each ball that goes through drags the speed down: about 1 dip per ball
    private boolean inDip = false;
    private boolean stopperOpen = false;
    private double transferPower = 0;

    // ===== Commands (copy these to PedroV3) =====
    // Shared first step: set the speed, wait until it's steady (or give up after SPINUP_TIMEOUT_MS).
    Command spinUp() {
        return sequential(
                instant(() -> {
                    setShooterVelocity(targetVelocity);
                    steadyTimer.reset();
                    phase = "SPINUP";
                    phaseTimer.reset();
                }),
                race(waitUntil(() -> shooterReady()), waitMs(SPINUP_TIMEOUT_MS)),
                instant(() -> {
                    spinUpMs = phaseTimer.milliseconds();
                    spinUpTimedOut = spinUpMs >= SPINUP_TIMEOUT_MS - 20;
                    phase = "FEEDING";
                    phaseTimer.reset();
                })
        );
    }

    // BLIND: what PedroV3 does now. Opens once and feeds for ms, even if the speed drops between balls.
    Command shootBlind(double ms) {
        return sequential(
                spinUp(),
                instant(() -> { setStopper(true); setTransfer(TRANSFER_POWER); }),
                waitMs(ms),
                instant(() -> { setStopper(false); setTransfer(0); setShooterPower(0); phase = "DONE"; })
        );
    }

    // GATED: HamzaTeli2's R2 logic as an auton step. For ms milliseconds, open + feed only while ready,
    // and close + stop feeding while the speed recovers after a ball.
    Command shootGated(double ms) {
        return sequential(
                spinUp(),
                new CommandBuilder()
                        .setExecute(() -> {
                            boolean ready = shooterReady();
                            if (ready && !stopperOpen) {
                                setStopper(true);
                                stopperTimer.reset();
                            } else if (!ready && stopperOpen && stopperTimer.milliseconds() >= STOPPER_MIN_OPEN_MS) {
                                setStopper(false);
                            }
                            setTransfer(stopperOpen ? TRANSFER_POWER : 0);
                        })
                        .setDone(() -> phaseTimer.milliseconds() >= ms)
                        .setEnd(end -> { setStopper(false); setTransfer(0); }),
                instant(() -> { setShooterPower(0); phase = "DONE"; })
        );
    }

    // ===== Helpers (same as PedroV3) =====
    private void setTransfer(double power) {
        transferPower = power;
        transferLeft.setPower(power);
        transferRight.setPower(power);
    }

    private void setStopper(boolean open) {
        stopperOpen = open;
        stopperServo.setPosition(open ? STOPPER_OPEN_POS : STOPPER_CLOSED_POS);
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

    private boolean shooterReady() {
        if (Math.abs(shooterSpeed() - targetVelocity) > VELOCITY_TOLERANCE) {
            steadyTimer.reset();
        }
        return steadyTimer.milliseconds() >= STEADY_MS;
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        transferLeft  = hardwareMap.get(DcMotor.class, "tl");
        transferRight = hardwareMap.get(DcMotor.class, "tr");
        shooterLeft   = hardwareMap.get(DcMotorEx.class, "sl");
        shooterRight  = hardwareMap.get(DcMotorEx.class, "sr");
        flowerServo1  = hardwareMap.get(Servo.class, "f1");
        flowerServo2  = hardwareMap.get(Servo.class, "f2");
        stopperServo  = hardwareMap.get(Servo.class, "ss");

        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        transferLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        transferRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flowerServo1.setDirection(Servo.Direction.FORWARD);
        flowerServo2.setDirection(Servo.Direction.REVERSE);
        stopperServo.setDirection(Servo.Direction.FORWARD);

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setStopper(false);
        flowerServo1.setPosition(FLOWER_UP_POS);
        flowerServo2.setPosition(FLOWER_UP_POS);

        // ===== INIT: pick the mode and settings =====
        boolean gated = false;
        boolean prevUp = false, prevDown = false, prevRb = false, prevLb = false;
        while (opModeInInit()) {
            if (gamepad1.cross)  gated = false;
            if (gamepad1.circle) gated = true;
            if (gamepad1.dpad_up && !prevUp)          targetVelocity += 50;
            if (gamepad1.dpad_down && !prevDown)      targetVelocity -= 50;
            if (gamepad1.right_bumper && !prevRb)     shootMs += 250;
            if (gamepad1.left_bumper && !prevLb)      shootMs = Math.max(250, shootMs - 250);
            prevUp = gamepad1.dpad_up; prevDown = gamepad1.dpad_down;
            prevRb = gamepad1.right_bumper; prevLb = gamepad1.left_bumper;

            telemetry.addData("Mode", gated ? "GATED (X = blind)" : "BLIND, same as PedroV3 (O = gated)");
            telemetry.addData("Target speed", "%.0f ticks/s  (D-pad up/down)", targetVelocity);
            telemetry.addData("Feed time", "%.0f ms  (bumpers)", shootMs);
            telemetry.addLine("Load pollen, then press START.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        RobotLog.ii("ShootTest", "START mode=%s target=%.0f feedMs=%.0f", gated ? "GATED" : "BLIND", targetVelocity, shootMs);
        schedule(gated ? shootGated(shootMs) : shootBlind(shootMs));
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime logTimer = new ElapsedTime();

        while (opModeIsActive()) {
            Scheduler.execute();
            double speed = shooterSpeed();

            if (phase.equals("FEEDING")) {
                minSpeedWhileFeeding = Math.min(minSpeedWhileFeeding, speed);
                if (!inDip && speed < targetVelocity - 150) { inDip = true; speedDips++; }
                if (inDip && speed > targetVelocity - 50) inDip = false;
            }
            if (!phase.equals("DONE") && logTimer.milliseconds() >= 100) {
                RobotLog.ii("ShootTest", "t=%.2f %s speed=%.0f stopper=%s transfer=%.1f",
                        runtime.seconds(), phase, speed, stopperOpen ? "OPEN" : "closed", transferPower);
                logTimer.reset();
            }

            telemetry.addData("Mode", gated ? "GATED" : "BLIND");
            telemetry.addData("Phase", phase);
            telemetry.addData("Speed / target", "%.0f / %.0f", speed, targetVelocity);
            telemetry.addData("Stopper", stopperOpen ? "OPEN" : "closed");
            telemetry.addData("Transfer", "%.1f", transferPower);
            telemetry.addData("Spin-up time", spinUpMs < 0 ? "-" : String.format("%.0f ms%s", spinUpMs, spinUpTimedOut ? "  (TIMED OUT, never steady)" : ""));
            telemetry.addData("Lowest speed while feeding", minSpeedWhileFeeding == Double.MAX_VALUE ? "-" : String.format("%.0f", minSpeedWhileFeeding));
            telemetry.addData("Speed dips (~ balls shot)", speedDips);
            telemetry.update();
        }
        RobotLog.ii("ShootTest", "END spinUpMs=%.0f timedOut=%b minSpeed=%.0f dips=%d", spinUpMs, spinUpTimedOut, minSpeedWhileFeeding, speedDips);
        setTransfer(0);
        setShooterPower(0);
    }
}
