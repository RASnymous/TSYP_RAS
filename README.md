# PCB — Radio Beacon

This folder contains the **PCB design and hardware documentation** for the radio beacons used in the project's resilient communication system.

## Context

In disaster response, underground exploration, and hazardous inspections, robots may operate without GPS or communication networks.

The system uses small **LoRa radio beacons** to store and relay critical mission information:

```text
Robot 1 → Beacon → Robot 2
```

A first robot can leave information at a location, allowing a second robot to retrieve it and continue the mission without starting from zero.

## PCB Role

The PCB provides the hardware platform for the beacon, including:

* Microcontroller
* LoRa communication module
* Power supply / battery interface
* Supporting electronic components

The beacon is designed to be **compact, low-power, and suitable for deployment in constrained environments**.

## Images

### PCB Size Reference

![20 mm coin reference](/pcb/media/coin20mm.png)

### LoRa Module

![LoRa module](/pcb/media/lora.png)

## Communication

The beacon uses **LoRa** for long-range, low-power communication. Beacon messages can contain information such as:

* Beacon ID
* Position
* Event type
* Severity / confidence
* Timestamp
* TTL

The PCB is part of the larger **autonomous robot resilient-communication system**.
