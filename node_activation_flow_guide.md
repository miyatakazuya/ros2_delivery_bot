# Autonomous Package Delivery — Node Activation Logic (FSM Toggle System)

This document describes the **full mission flow** and the **activation/deactivation logic** for each ROS2 node in the autonomous delivery robot system. The behavior is governed by a **Finite State Machine (FSM)** inside `mission_controller.py`, which toggles other nodes ON/OFF through a shared ROS2 topic (e.g., `/node_control`).  
This approach improves compute efficiency, simplifies debugging, and ensures that only the necessary nodes operate during each mission phase.

---

# ROS2 Nodes Overview

| Node Name               | Purpose |
|-------------------------|---------|
| `mission_controller`    | Main FSM controller (always ON) |
| `box_detection`         | YOLO-based container detection (front camera) |
| `apriltag_node`         | AprilTag detection (front camera) |
| `apriltag_back_node`    | AprilTag detection (rear camera) |
| `parking_controller`    | Executes 180° rotation + reverse parking |
| `servo_controller`      | Tilts dump-bed to release package |

Nodes are toggled ON/OFF via messages of the form:
“<node_name>:1”   # activate node
“<node_name>:0”   # deactivate node
---

# Mission Phases

The mission proceeds through the following phases:

1. **STARTUP**
2. **SEARCH**
3. **ALIGN_FRONT**
4. **TURN_AROUND**
5. **BACKUP_PARK**
6. **DROP_PACKAGE**
7. **EXIT / DONE**

Each phase activates only the nodes required for that behavior.

---

# Node Activation by Phase

## **1. STARTUP / IDLE**

Robot is powered on, initializes systems, waits for mission start.

| Node | Active? | Notes |
|------|---------|--------|
| mission_controller | **1** | Always running |
| box_detection | 1 | Optional to delay |
| apriltag_node | 1 | Optional to delay |
| parking_controller | 0 | No motion yet |
| apriltag_back_node | 0 | Rear camera unused |
| servo_controller | 0/1 | Idle |

**Transition → SEARCH**  
Triggered by a mission start condition (button, remote command, timer, etc.)

---

## **2. SEARCH**

Robot looks for the container using YOLO + front AprilTag.

| Node | Active? |
|------|---------|
| mission_controller | **1** |
| box_detection | **1** |
| apriltag_node | **1** |
| parking_controller | 0 |
| apriltag_back_node | 0 |
| servo_controller | 0 |

**Purpose:**  
- YOLO identifies potential container regions.  
- Front AprilTag node confirms the AprilTag and estimates container pose.

**Transition → ALIGN_FRONT**  
When AprilTag is detected with stable pose estimation.

---

## **3. ALIGN_FRONT**

Robot aligns its front side head-on with the container using AprilTag pose data.

| Node | Active? |
|------|---------|
| mission_controller | **1** |
| box_detection | **0** |
| apriltag_node | **1** |
| parking_controller | 0 |
| apriltag_back_node | 0 |
| servo_controller | 0 |

**Purpose:**  
- Center car relative to container.  
- Adjust heading to be perpendicular.  
- Move into a “front staging” position.

**Transition → TURN_AROUND**  
When alignment and distance meet required thresholds.

Upon transition: **front AprilTag is no longer needed → turn OFF.**

---

## **4. TURN_AROUND (180° Rotation)**

Robot rotates so its *rear* faces the container.

| Node | Active? |
|------|---------|
| mission_controller | **1** |
| box_detection | 0 |
| apriltag_node | **0** |
| apriltag_back_node | **1** |
| parking_controller | **1** |
| servo_controller | 1 |

**Purpose:**  
- `parking_controller` performs controlled 180° rotation.  
- Rear AprilTag node begins detecting the container from behind.

**Transition → BACKUP_PARK**  
When rotation is complete and rear AprilTag becomes visible.

---

## **5. BACKUP_PARK**

Robot reverses toward the container using rear AprilTag for alignment.

| Node | Active? |
|------|---------|
| mission_controller | **1** |
| box_detection | 0 |
| apriltag_node | 0 |
| apriltag_back_node | **1** |
| parking_controller | **1** |
| servo_controller | 1 |

**Purpose:**  
- Fine-tune heading while reversing.  
- Stop when dump-bed is positioned over the container.

**Transition → DROP_PACKAGE**  
When tag-relative distance < target threshold.

---

## **6. DROP_PACKAGE**

Robot activates servo to release the payload.

| Node | Active? |
|------|---------|
| mission_controller | **1** |
| box_detection | 0 |
| apriltag_node | 0 |
| apriltag_back_node | 1 or 0 |
| parking_controller | 0 |
| servo_controller | **1** |

**Purpose:**  
- Servo tilts the bed to drop the package.  
- AprilTag (rear) may remain ON if position monitoring is needed.

**Transition → EXIT / DONE**  
When servo completes its motion.

---

## **7. EXIT / DONE**

Mission successfully completed.

| Node | Active? |
|------|---------|
| mission_controller | **1** |
| box_detection | 0 |
| apriltag_node | 0 |
| apriltag_back_node | 0 |
| parking_controller | 0 |
| servo_controller | 0 |

**Purpose:**  
- Stop movement.  
- Shut down compute-intensive nodes.  
- System waits for next mission or shutdown.

---

# 🔄 Quick Summary of Toggle Timeline
STARTUP:
box_detection = 1
apriltag_node = 1

SEARCH:
(looking for container)
→ when detected →

ALIGN_FRONT:
box_detection = 0
apriltag_node = 1
→ when aligned →

TURN_AROUND:
apriltag_node = 0
apriltag_back_node = 1
parking_controller = 1

BACKUP_PARK:
apriltag_back_node = 1
parking_controller = 1
→ when in drop position →

DROP_PACKAGE:
servo_controller = 1
parking_controller = 0

DONE:
all nodes OFF except mission_controller