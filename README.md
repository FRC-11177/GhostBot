# GhostBot

## Robot Layout

4x NEO Brushless w/ SparkMAX Differential Chassis
Pivot intake w/ NEO Vortex for pivot and NEO 550 for intaking bar
Linear passing mechansim w/ NEO v1.1
NEO Vortex for shooting motor

## Mechanism Details

### Drivetrain

#### Motor Informations

| Motor Name | CAN ID | Controlmode|
|--|--|-----------------------------
| FL | 10 | MaxMotionVelocityControl|
| BL | 11 | Follower |
| FR | 12 | MaxMotionVelocityControl |
| BR | 13 | Follower |

#### Other Details

- Gear Ratio: 10.71 : 1
- Wheel Radius: 3 in.
- Track Width: 25 in.
- Gyroscope: Use NavX 2 as default

#### Open API

- `drive(ChassisSpeeds robotRelativeSpeeds)` for general mechanism requests
- `drive(Supplier<ChassisSpeeds>)` for driver request
- `getPose()` for other mechcanism to get the location of the robot.