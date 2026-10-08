# Repository Guidelines

## Project Structure & Module Organization

This is a Java 17 FRC robot project built with WPILib GradleRIO. Robot code lives in `src/main/java/frc/robot/`: `Robot.java` owns lifecycle hooks, `RobotContainer.java` configures commands and controls, and mechanism packages such as `Drivetrain/`, `Intake/`, and `Vision/` contain subsystem classes plus their `Constants.java` files. Deployable robot assets live in `src/main/deploy/`, including PathPlanner configuration under `pathplanner/`. Vendor dependency definitions are in `vendordeps/`; do not edit generated Gradle wrapper files unless intentionally upgrading Gradle.

## Build, Test, and Development Commands

Run commands from the repository root:

- `./gradlew build` — compiles the robot code and runs checks/tests.
- `./gradlew test` — runs JUnit 5 tests.
- `./gradlew simulateJava` — launches the desktop robot simulator with GUI and Driver Station.
- `./gradlew deploy` — builds and deploys to the configured RoboRIO; confirm the team number and target first.
- `./gradlew clean` — removes local build output when a clean rebuild is needed.

Use `gradlew.bat` instead of `./gradlew` on Windows.

## Coding Style & Naming Conventions

Use the existing Java style: two-space indentation, braces on the declaration line, and one import per line. Keep packages under `frc.robot`; name classes in PascalCase and methods/fields in camelCase. Group mechanism-specific values in that mechanism's `Constants.java` rather than embedding CAN IDs, dimensions, or tuning values in command logic. Prefer descriptive names such as `getAutonomousCommand` and `robotRelativeSpeeds`.

## Testing Guidelines

Tests use JUnit Jupiter (JUnit 5). Place new tests in `src/test/java/` mirroring the production package path, and name them `*Test.java` (for example, `DrivetrainTest.java`). Test calculations, command behavior, and safety-relevant limits without hardware when practical. Run `./gradlew test` before submitting changes; there is no repository-wide coverage threshold configured.

## Commit & Pull Request Guidelines

Recent commits use short, imperative, sentence-style summaries (for example, `Finished basic drivetrain feature`). Keep each commit focused and describe the affected subsystem. Pull requests should explain the behavior change, list validation performed (such as simulation or tests), link the relevant issue when available, and include screenshots or Driver Station/log evidence for operator-visible changes. Call out changes to CAN IDs, control modes, deploy assets, or vendor dependencies explicitly.
