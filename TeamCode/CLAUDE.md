# CLAUDE.md — FTC Robot Project

## Project Overview

FTC (FIRST Tech Challenge) robot codebase for **DECODE (2025-26)** season.
- **Team:** 32987 — SBU6 Robotics
- **FTC SDK Version:** 11.0.0
- **Build System:** Gradle 8.9 / Android Gradle Plugin 8.7.0

---

## Module Structure

| Module | Type | Purpose |
|---|---|---|
| `TeamCode` | Android App | Main robot code (OpModes, autonomous, teleop) |
| `FtcRobotController` | Android Library | FTC SDK framework — do not edit |
| `MeepMeep` | Java Library | Path visualization for RoadRunner (runs on PC, not robot) |

All robot code lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

---

## Key Dependencies

| Library | Version | Purpose |
|---|---|---|
| FTC SDK | 11.0.0 | Robot hardware abstraction |
| RoadRunner (ftc) | 0.1.25 | Autonomous path following |
| RoadRunner (core) | 1.0.1 | Motion profiling |
| RoadRunner (actions) | 1.0.1 | Action-based autonomous |
| Sloth | 0.2.4 | Hot reload — push code without full reinstall |
| SlothBoard Dashboard | 0.2.4+0.5.1 | Telemetry dashboard |
| MeepMeep | 0.1.7 | Path visualization (PC only) |

---

## Building & Deploying

### Full Build (first install or after structural changes)
Use Android Studio's standard **Run** button to build and install the full APK onto the Control Hub.

### Hot Reload with Sloth (faster, day-to-day use)
Sloth allows pushing code changes without reinstalling the full APK.

**Connect Control Hub over WiFi ADB:**
1. Power on the Control Hub
2. Connect your Mac to the Control Hub's WiFi (`FTC-XXXX` network)
3. Run: `adb connect 192.168.43.1:5555`
4. Verify: `adb devices`

**Deploy changed code:**
```bash
./gradlew :TeamCode:removeSlothRemote :TeamCode:deploySloth
```
Or configure Android Studio run configurations:
- Before launch: `removeSlothRemote`
- Launch: `deploySloth`

---

## Gradle Repositories

```
https://repo.dairy.foundation/releases   — Sloth, Load plugin
https://repo.dairy.foundation/snapshots  — Sloth snapshots
https://maven.brott.dev/                 — RoadRunner, MeepMeep
```

---

## Wiki Links

<!-- TODO: Paste relevant documentation links here -->
- Roadrunner SDK repo I made a fork of: https://github.com/acmerobotics/road-runner-ftc
- RoadRunner Docs: https://rr.brott.dev/docs/
- Learn Roadrunner docs: https://learnroadrunner.com/introduction.html
- Sloth / Dairy Foundation Repo: https://github.com/Dairy-Foundation/Sloth 
- MeepMeep Path Planner repo (There is also lots of information on the learn RR and RR docs): https://github.com/acmerobotics/MeepMeep
-My repo from last year (Good programming style which i like. However this may not br a correct/recommended style so take this with a pinch of salt): https://github.com/adamr500/Ni4_Roadrunner_V1
- Official FTC DECODE season game manual: https://ftc-resources.firstinspires.org/ftc/game/manual
---

## Notes

-My robot works as such:
-4 wheel Mecannum drivetrain
-Intake brings balls (artefacts) from the outside into the robot
-There is a stopper servo to prevent the balls (artefacts) from entering the shooter when I am not in the shooter zone
-There is a turret which spins the shooter allowing for auto aiming regardless of the robot heading. The turret has a limited range since there are cables going to the turret
-The shooter has 2 motors and spins a flywheel to shoot balls
-There is an adjustable hood controlled by 2 servos which changes the angle of the hood to adjust for different shot ranges
-I will use GoBilda 2 pod odometry and pinpoint IMU for autonomous and to find my position on the field during teleop for auto turret aiming