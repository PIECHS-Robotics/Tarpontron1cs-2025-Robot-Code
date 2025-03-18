
---

## 🛠️ Wiring Diagram & Port Assignments

### CAN Bus Assignments
| Device | CAN ID |
|---|---|
| Front Left Drive Motor (NEO) | 6 |
| Front Left Steering Motor (NEO) | 5 |
| Front Right Drive Motor (NEO) | 8 |
| Front Right Steering Motor (NEO) | 7 |
| Rear Left Drive Motor (NEO) | 12 |
| Rear Left Steering Motor (NEO) | 11 |
| Rear Right Drive Motor (NEO) | 10 |
| Rear Right Steering Motor (NEO) | 9 |
| Coral Elevator (Neo Vortex) | 4 |
| Coral Arm (NEO) | 3 |
| Coral Intake (NEO) | 2 |
| Algae Intake (Neo Vortex) | 13 |
| Algae Pivot Arm (NEO) | 14 |

### DIO Assignments
| Sensor | DIO Port |
|---|---|
| Coral Elevator Bottom Limit Switch (REV Magnetic) | 9 |

---

## 🎮 Driver Controls - PS4 Controller
| Button | Function |
|---|---|
| Left Stick | Drive Translation X/Y |
| Right Stick | Drive Rotation |
| L1 | Hold: Swerve Lock (X Formation) |
| L2 | Hold: Run Coral Intake |
| R2 | Hold: Reverse Coral Intake |
| R1 | Hold: Run Algae Intake |
| Triangle | Move Coral to Level 4 |
| Square | Move Coral to Level 3 |
| Cross | Move Coral to Level 2 |
| Circle | Move Coral to Feeder Station (HP), Stow Algae Arm |
| Options | Zero Gyro Heading |

---

## 🔄 Homing Process (Coral Elevator)
- On startup, the **Coral Elevator automatically lowers slowly until it triggers the magnetic limit switch**.
- Once the switch triggers, the elevator's **encoder is zeroed**, and the elevator is ready for positional control.

---

## 📊 Recommended SmartDashboard Layout
| Key | Purpose |
|---|---|
| Drive/Heading | Current robot heading |
| Drive/Swerve States | Per-module angles & speeds |
| Coral/Arm/Position | Arm encoder position |
| Coral/Elevator/Position | Elevator encoder position |
| Coral/Intake/Output | Intake motor output |
| Algae/Arm/Position | Algae arm encoder position |
| Algae/Intake/Output | Algae intake motor output |

---

## 🔧 First-Time Setup Guide
### Cloning this Repo
```bash
git clone https://github.com/PIECHS-Robotics/Tarpontron1cs-2025-Robot-Code.git
