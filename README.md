# 2026CompetitiveConcept

This repository contains the code used for the WestCoast Products 2026 [Competitive Concept](https://wcproducts.com/pages/wcp-competitive-concepts).

The project is based on one of CTRE's [Phoenix 6 example projects](https://github.com/CrossTheRoadElec/Phoenix6-Examples/tree/main/java/SwerveWithChoreo). It uses WPILib [command-based programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html) to manage robot subsystems and actions, a [Limelight](https://limelightvision.io/) for vision, and [Choreo](https://choreo.autos/) for autonomous path following.

## Controls

All controls are mapped to a single Xbox controller (Port 0).

### Driving

| Input | Action |
|-------|--------|
| Left Stick | Drive (forward/backward & strafe) |
| Right Stick | Rotate |
| Back Button | Reset field-centric heading |

### Heading Lock

| Button | Heading |
|--------|---------|
| A | 180° (backward) |
| B | 90° (right) |
| X | 270° (left) |
| Y | 0° (forward) |

### Shooting

| Input | Action |
|-------|--------|
| Right Trigger (hold) | Auto-aim and shoot - aims robot at hub, spins up shooter, and feeds when ready |
| Right Bumper (hold) | Manual shoot - spins up shooter to dashboard RPM value and feeds |

### Intake

| Input | Action |
|-------|--------|
| Left Trigger (hold) | Deploy intake and run rollers |
| Left Bumper | Stow intake |

### Hanger

| Input | Action |
|-------|--------|
| D-Pad Up | Extend hanger to hanging position |
| D-Pad Down | Retract hanger to hung position |