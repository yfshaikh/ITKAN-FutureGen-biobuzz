# Robot ports (BIOBUZZ robot)

From the Driver Hub robot configuration, Nov 16 2026. If you change the configuration, update this file.

## Control Hub

| Port | Name | What it is |
|---|---|---|
| Motor 0 | `br` | back right wheel |
| Motor 1 | `sr` | shooter right |
| Motor 2 | `tr` | transfer (intake) right |
| Motor 3 | `fr` | front right wheel |
| Servo 0 | `f2` | flower bar servo 2 |
| I2C bus 3 | `pinpoint` | goBILDA Pinpoint odometry computer (used by Pedro) |

## Expansion Hub

| Port | Name | What it is |
|---|---|---|
| Motor 0 | `bl` | back left wheel |
| Motor 1 | `sl` | shooter left |
| Motor 2 | `tl` | transfer (intake) left |
| Motor 3 | `fl` | front left wheel |
| Servo 0 | `f1` | flower bar servo 1 |
| Servo 1 | `ss` | stopper: holds pollen back until the flywheel is ready (HamzaTeli2: closed/blocking 0.5, open 0.0. HamzaTeli and PedroV2 still have them the wrong way round: 0.0 / 0.5) |

`fl` (letter L) is the front left wheel. `f1` (number one) is flower servo 1.

## Directions set in code

| Part | FORWARD | REVERSE | Set in |
|---|---|---|---|
| Drive wheels | `fl`, `bl` | `fr`, `br` | `MainTeleOp.java` and `pedro/Constants.java` (must match) |
| Transfer | `tr` | `tl` | `MainTeleOp.java`, `BioBuzzVisualizerAuton.java` |
| Shooter | `sl` | `sr` | `MainTeleOp.java` |
| Flower servos | `f1` | `f2` | `MainTeleOp.java`, `BioBuzzVisualizerAuton.java` |
| Pinpoint pods | | x pod and y pod both REVERSED | `pedro/Constants.java` |

## Encoders

Find these with the "Encoder Check" OpMode: turn one wheel by hand and see which name's number changes.

| Motor port | Encoder found |
|---|---|
| `fl` | ? |
| `fr` | ? |
| `bl` | ? |
| `br` | ? |
| `sl` | ? |
| `sr` | ? |
| unplugged | ? |

Pedro doesn't use the wheel encoders (it uses the Pinpoint), so only the shooter encoders matter, and only once shooter speed control is added.
