# Dynamixel OpenRB Controller

Firmware for **OpenRB-150** controlling **AX-12A Dynamixel motors** via
a custom **serial protocol**.\
The board receives commands from a host application (Rust/Nannou,
OpenFrameworks, etc.) and drives the motors in real time.

The main use case is **interactive robot control from a 3D
application**, where the software computes the desired joint angles and
sends them to the robot.

------------------------------------------------------------------------

# Overview

Architecture:

Rust / Nannou / OpenFrameworks\
│\
│ USB Serial\
▼\
OpenRB-150\
│\
│ TTL Dynamixel Bus\
▼\
AX-12A Motors

Responsibilities:

  Component              Responsibility
  ---------------------- -------------------------------------
  **Host application**   Computes desired joint positions
  **OpenRB firmware**    Receives commands and drives motors
  **Dynamixel motors**   Execute movement

------------------------------------------------------------------------

# Hardware

Required hardware:

-   OpenRB-150 board
-   AX-12A Dynamixel motors
-   12V power supply
-   Dynamixel TTL cables
-   USB connection to PC

Example configuration:

Motor IDs: 1, 2, 3\
Protocol: Dynamixel 1.0\
Baudrate: 1000000

------------------------------------------------------------------------

# Features

-   Multi-motor control
-   Real-time serial communication
-   Packet CRC validation
-   Latest-command-wins buffering
-   Safety watchdog
-   Motor centering command
-   Stop/hold command
-   Scalable to additional motors

------------------------------------------------------------------------

# Communication Protocol

The firmware uses a **custom binary packet protocol**.

### Packet Format

\[AA\]\[55\]\[LEN\]\[CMD\]\[PAYLOAD...\]\[CRC_L\]\[CRC_H\]

  Field       Description
  ----------- ---------------------------
  `AA 55`     Packet start bytes
  `LEN`       Length of `CMD + PAYLOAD`
  `CMD`       Command ID
  `PAYLOAD`   Command specific data
  `CRC`       CRC16-Modbus checksum

CRC is computed over:

\[LEN\]\[CMD\]\[PAYLOAD...\]

------------------------------------------------------------------------

# Commands

## Set single motor

CMD = 0x01\
Payload:

\[id\]\[pos_lo\]\[pos_hi\]\[spd_lo\]\[spd_hi\]

  Field   Description
  ------- --------------------------
  `id`    motor ID
  `pos`   target position (0-1023)
  `spd`   speed (1-1023)

------------------------------------------------------------------------

## Set multiple motors

CMD = 0x02

Payload:

\[count\]\
(id pos spd) repeated count times

Example for 3 motors:

\[count=3\]\
\[id1 pos1 spd1\]\
\[id2 pos2 spd2\]\
\[id3 pos3 spd3\]

This allows updating **all motors in one packet**.

------------------------------------------------------------------------

## Stop all motors

CMD = 0x03\
Payload: none

The controller reads the **current motor positions** and sets them as
targets so the motors hold their current pose.

------------------------------------------------------------------------

## Center all motors

CMD = 0x04\
Payload: none

All motors move to position:

512

------------------------------------------------------------------------

# Safety Features

## Watchdog

If no command is received for:

WATCHDOG_MS = 200 ms

The controller stops applying commands.

Optional behavior:

TORQUE_OFF_ON_TIMEOUT

Can disable motor torque when communication stops.

------------------------------------------------------------------------

# Control Loop

The firmware operates in two stages.

### 1. Receive commands

Incoming serial packets are parsed and stored as **latest motor
targets**.

goalPos\[i\]\
goalSpd\[i\]\
pending\[i\]

### 2. Apply commands

At a fixed rate:

APPLY_HZ = 50 Hz

The controller sends commands to motors.

To reduce start-time differences:

1.  Write speeds to all motors\
2.  Write goal positions to all motors

This makes movement appear **more synchronized**.

------------------------------------------------------------------------

# Motor Limits

AX-12A raw position range:

0 -- 1023

Speed range:

1 -- 1023

Note:

Speed = 0 → maximum speed

The firmware clamps values to safe ranges.

------------------------------------------------------------------------

# Example Packet

Example: move 3 motors.

AA 55\
LEN\
02\
03\
01 pos spd\
02 pos spd\
03 pos spd\
CRC

------------------------------------------------------------------------

# Arduino Structure

Main components in the firmware:

### Hardware initialization

setup()

-   open serial
-   initialize Dynamixel bus
-   ping motors
-   enable torque

### Serial parser

State machine:

WaitSTX1\
WaitSTX2\
WaitLEN\
ReadBody

Ensures valid packets are reconstructed from the serial byte stream.

### Command dispatch

dispatchPacket()

Routes packets to the correct command handler.

### Motor control loop

loop()

Two steps:

drainSerialKeepLatest()\
applyLatestIfDue()

------------------------------------------------------------------------

# Integration with Host Software

The controller is designed to work with applications like:

-   Rust + Nannou
-   OpenFrameworks
-   Python
-   ROS

Typical workflow:

1.  Host computes robot joint targets\
2.  Host sends packet\
3.  OpenRB applies motor commands\
4.  Robot moves

------------------------------------------------------------------------

# Example Use Case

A **3D robot arm model** is rendered in Nannou.

User interaction:

drag end effector\
↓\
inverse kinematics\
↓\
joint angles\
↓\
serial packet\
↓\
real robot moves

------------------------------------------------------------------------

# Future Improvements

Possible extensions:

-   Dynamixel SYNC_WRITE for perfect simultaneous motion
-   Motor state feedback
-   Temperature monitoring
-   Torque control
-   Trajectory interpolation
-   Inverse kinematics on embedded controller

------------------------------------------------------------------------

# License

MIT / Open source.
