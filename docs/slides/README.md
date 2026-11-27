# Architecture diagrams

Every diagram from the APNVI final presentation, from the device as a whole down to each stage's internals. The full deck is also available as a PDF: [APNVI-presentation.pdf](../APNVI-presentation.pdf).

## Contents

- [The device and what it does](#the-device-and-what-it-does)
  - None of today's aids can take you to room 117
  - One device, held like a cane
  - From one sentence to “You have arrived”
  - It stops before it touches anything
  - Ask, and it looks for you
  - It draws the room, and names what is in it
  - The handle leads the way
  - The grip only speaks when it matters
  - Free mode: you push, it steers
  - We tested it on the floor, not just on paper
  - What broke, and how we fixed it
- [The system](#the-system)
  - The whole system
  - The whole system: every topic
  - How the parts talk: ROS 2 in one picture
  - One trip, start to finish: “Take me to the water bottle”
- [Stage 1: obstacles](#stage-1-obstacles)
  - Stage 1: what counts as “in the way”
  - Stage 1 inside: the programs and how they talk
  - Stage 1 to a stop: the hazard gate
- [Stage 2: words and things](#stage-2-words-and-things)
  - Stage 2 inside: the programs and how they talk
  - Stage 2: a question or a command?
  - Stage 2: from a box on a picture to a pin on the map
  - Stage 2: the map labels itself
- [Stage 3: the map](#stage-3-the-map)
  - Stage 3 inside: the programs and how they talk
  - Stage 3: map drift and the fix
  - Stage 3: finding itself on a saved map
  - Stage 3: a map’s life, and a new building
- [Stage 4: guiding](#stage-4-guiding)
  - Stage 4 inside: the programs and how they talk
  - Stage 4: Nav2 under the hood
  - Stage 4: a trip, its states and its rules
  - Stage 4: getting unstuck by itself
  - Stage 4: free mode, steering with no trip
  - Stage 4: the grip, its signals and buttons
- [Computers](#computers)
  - What runs where, and what works without the network
  - How the device reaches the campus PC
- [Hardware](#hardware)
  - Wiring and power
  - What each sensor sees
- [Safety](#safety)
  - Every way the wheels stop
- [Evidence](#evidence)
  - All the test results
  - How fast each part is
- [Why this way](#why-this-way)
  - APNVI next to what already exists
  - Why we built it this way

## The device and what it does

### None of today's aids can take you to room 117

![None of today's aids can take you to room 117](02-none-of-today-s-aids-can-take-you-to-room-117.png)

### One device, held like a cane

![One device, held like a cane](03-one-device-held-like-a-cane.png)

### From one sentence to “You have arrived”

![From one sentence to “You have arrived”](04-from-one-sentence-to-you-have-arrived.png)

### It stops before it touches anything

![It stops before it touches anything](05-it-stops-before-it-touches-anything.png)

### Ask, and it looks for you

![Ask, and it looks for you](06-ask-and-it-looks-for-you.png)

### It draws the room, and names what is in it

![It draws the room, and names what is in it](07-it-draws-the-room-and-names-what-is-in-it.png)

### The handle leads the way

![The handle leads the way](08-the-handle-leads-the-way.png)

### The grip only speaks when it matters

![The grip only speaks when it matters](09-the-grip-only-speaks-when-it-matters.png)

### Free mode: you push, it steers

![Free mode: you push, it steers](10-free-mode-you-push-it-steers.png)

### We tested it on the floor, not just on paper

![We tested it on the floor, not just on paper](11-we-tested-it-on-the-floor-not-just-on-paper.png)

### What broke, and how we fixed it

![What broke, and how we fixed it](12-what-broke-and-how-we-fixed-it.png)

## The system

### The whole system

![The whole system](16-the-whole-system.png)

### The whole system: every topic

![The whole system: every topic](17-the-whole-system-every-topic.png)

### How the parts talk: ROS 2 in one picture

![How the parts talk: ROS 2 in one picture](18-how-the-parts-talk-ros-2-in-one-picture.png)

### One trip, start to finish: “Take me to the water bottle”

![One trip, start to finish: “Take me to the water bottle”](19-one-trip-start-to-finish-take-me-to-the-water-bott.png)

## Stage 1: obstacles

### Stage 1: what counts as “in the way”

![Stage 1: what counts as “in the way”](20-stage-1-what-counts-as-in-the-way.png)

### Stage 1 inside: the programs and how they talk

![Stage 1 inside: the programs and how they talk](21-stage-1-inside-the-programs-and-how-they-talk.png)

### Stage 1 to a stop: the hazard gate

![Stage 1 to a stop: the hazard gate](22-stage-1-to-a-stop-the-hazard-gate.png)

## Stage 2: words and things

### Stage 2 inside: the programs and how they talk

![Stage 2 inside: the programs and how they talk](23-stage-2-inside-the-programs-and-how-they-talk.png)

### Stage 2: a question or a command?

![Stage 2: a question or a command?](24-stage-2-a-question-or-a-command.png)

### Stage 2: from a box on a picture to a pin on the map

![Stage 2: from a box on a picture to a pin on the map](25-stage-2-from-a-box-on-a-picture-to-a-pin-on-the-ma.png)

### Stage 2: the map labels itself

![Stage 2: the map labels itself](26-stage-2-the-map-labels-itself.png)

## Stage 3: the map

### Stage 3 inside: the programs and how they talk

![Stage 3 inside: the programs and how they talk](27-stage-3-inside-the-programs-and-how-they-talk.png)

### Stage 3: map drift and the fix

![Stage 3: map drift and the fix](28-stage-3-map-drift-and-the-fix.png)

### Stage 3: finding itself on a saved map

![Stage 3: finding itself on a saved map](29-stage-3-finding-itself-on-a-saved-map.png)

### Stage 3: a map’s life, and a new building

![Stage 3: a map’s life, and a new building](30-stage-3-a-map-s-life-and-a-new-building.png)

## Stage 4: guiding

### Stage 4 inside: the programs and how they talk

![Stage 4 inside: the programs and how they talk](31-stage-4-inside-the-programs-and-how-they-talk.png)

### Stage 4: Nav2 under the hood

![Stage 4: Nav2 under the hood](32-stage-4-nav2-under-the-hood.png)

### Stage 4: a trip, its states and its rules

![Stage 4: a trip, its states and its rules](33-stage-4-a-trip-its-states-and-its-rules.png)

### Stage 4: getting unstuck by itself

![Stage 4: getting unstuck by itself](34-stage-4-getting-unstuck-by-itself.png)

### Stage 4: free mode, steering with no trip

![Stage 4: free mode, steering with no trip](35-stage-4-free-mode-steering-with-no-trip.png)

### Stage 4: the grip, its signals and buttons

![Stage 4: the grip, its signals and buttons](36-stage-4-the-grip-its-signals-and-buttons.png)

## Computers

### What runs where, and what works without the network

![What runs where, and what works without the network](37-what-runs-where-and-what-works-without-the-network.png)

### How the device reaches the campus PC

![How the device reaches the campus PC](38-how-the-device-reaches-the-campus-pc.png)

## Hardware

### Wiring and power

![Wiring and power](39-wiring-and-power.png)

### What each sensor sees

![What each sensor sees](40-what-each-sensor-sees.png)

## Safety

### Every way the wheels stop

![Every way the wheels stop](41-every-way-the-wheels-stop.png)

## Evidence

### All the test results

![All the test results](42-all-the-test-results.png)

### How fast each part is

![How fast each part is](43-how-fast-each-part-is.png)

## Why this way

### APNVI next to what already exists

![APNVI next to what already exists](44-apnvi-next-to-what-already-exists.png)

### Why we built it this way

![Why we built it this way](45-why-we-built-it-this-way.png)
