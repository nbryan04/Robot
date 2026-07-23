#include "mission.h"

Mission::Mission(Drivetrain& dt, Ultrasonic& us, Camera& cam, Claw& cl,
                 MetalDetector& md, TiltSensor& ts, LineFollower& lf)
    : drive(dt), ultra(us), camera(cam), claw(cl),
      metal(md), tilt(ts), line(lf) {}

void Mission::begin() {
    enter(ROUTER);
}

void Mission::jumpTo(State s) {
    enter(s);
}

// -------- helpers ----------------------------------------------------------

void Mission::enter(State s) {
    state = s;
    subStep = 0;
    stateTimer = millis();
    if (s == CENTRE_ROCK) centreAttempts = 0;  // fresh distance-correction loop
}

bool Mission::driveIdle() {
    return drive.state == Drivetrain::Idle;
}

// Heading (deg, relative to the sweep-start forward) during an active +arc turn
// that runs from -arc/2 up to +arc/2.
float Mission::sweepHeadingDeg(float arcDeg) {
    return -arcDeg / 2.0f + arcDeg * drive.moveProgress();
}

// Global interrupts, checked every loop (placeholders for now).
bool Mission::checkStall() {
    // TODO: detect a stall (commanded to move but encoders not changing) and
    // recover, then resume. Raise the threshold while climbing the ramp.
    return false;
}

bool Mission::checkTimeLow() {
    // TODO: when the run clock is low, abandon remaining clusters and head for
    // the panel (via the ramp if still on the LOWER deck).
    return false;
}

// -------- main update ------------------------------------------------------

void Mission::update() {
    // Global interrupts are checked every loop, not drawn as edges.
    if (checkStall())   { /* TODO: recover then resume */ }
    if (checkTimeLow()) { /* TODO: abandon clusters, head for panel */ }

    switch (state) {

    // ---- n18: Phase Router ------------------------------------------------
    case ROUTER:
        if (phase == COLLECT) {
            enter(HOP_TO_CLUSTER);
        } else {
            // PANEL / DONE are out of scope for this pass.
            enter(HOLD);
        }
        break;

    // ---- n1: Dead-reckon to next cluster ----------------------------------
    case HOP_TO_CLUSTER: {
        int idx = rocks_visited;
        if (idx > 5) idx = 5;  // defensive clamp

        if (subStep == 0) {
            // Counter says the ramp is next: divert before hopping.
            if (level == LOWER && rocks_visited == 4) {
                enter(RAMP_APPROACH);
                break;
            }
            // Start the first leg of the hop path. Skip the turn if it's a
            // zero-angle leg (issuing a 0-magnitude move makes the drivetrain
            // jitter in place instead of finishing).
            hopLeg = 0;
            if (HOP_LEGS[idx][hopLeg].angleDeg != 0.0f) {
                drive.turn(HOP_LEGS[idx][hopLeg].angleDeg, HOP_SPEED);
            }
            subStep = 1;
        } else if (subStep == 1) {
            // Turn finished (or none issued): drive this leg's straight segment.
            // Skip the drive on a zero-distance leg so a rotation-only leg works.
            if (driveIdle()) {
                if (HOP_LEGS[idx][hopLeg].distMM != 0.0f) {
                    drive.driveStraight(HOP_LEGS[idx][hopLeg].distMM, HOP_SPEED);
                }
                subStep = 2;
            }
        } else {  // subStep == 2
            if (driveIdle()) {
                hopLeg++;
                if (hopLeg < HOP_LEG_COUNT[idx]) {
                    // More legs to go: turn into the next one (skip if zero).
                    if (HOP_LEGS[idx][hopLeg].angleDeg != 0.0f) {
                        drive.turn(HOP_LEGS[idx][hopLeg].angleDeg, HOP_SPEED);
                    }
                    subStep = 1;
                } else {
                    // Whole path done: this arrival heading is the reference the
                    // next hop is measured from. Start tracking excursion rotation.
                    clusterHeading = 0.0f;
                    excursionForward = 0.0f;
                    sweepPass = 0;  // fresh rock: start the sweep-pass count over
                    approachAttempts = 0;  // and its re-approach retry count
                    // Tilt cross-check: ramp may show up here.
                    if (level == LOWER && tilt.isOnRamp()) {
                        enter(RAMP_APPROACH);
                    } else if (enableRockSearch) {
                        enter(FIND_ROCK);
                    } else if (enableMetalScan) {
                        // No search, but still run the claw sequence at the rock.
                        enter(NEED_METAL);
                    } else {
                        enter(ADVANCE_CLUSTER);  // nav-only: skip the search
                    }
                }
            }
        }
        break;
    }

    // ---- n2: Find Rock (wide ultrasonic sweep) ----------------------------
    case FIND_ROCK:
        if (subStep == 0) {
            // Rotate to the start of the arc.
            drive.turn(-SWEEP_ARC / 2.0f, SWEEP_SPEED);
            clusterHeading += -SWEEP_ARC / 2.0f;
            subStep = 1;
        } else if (subStep == 1) {
            if (driveIdle()) {
                // Sweep across the full arc while polling the ultrasonic.
                // beginScan() wipes stale readings so no pre-sweep data can
                // trigger a false edge.
                ultra.beginScan();
                foundStartEdge = false;
                foundEndEdge = false;
                minSweepDistance = Ultrasonic::MAX_VALID_DISTANCE;  // nothing closer yet
                minSweepAngle = 0.0f;
                sumOnRockAngle = 0.0f;
                onRockCount = 0;
                drive.turn(SWEEP_ARC, SWEEP_SPEED);
                clusterHeading += SWEEP_ARC;
                subStep = 2;
            }
        } else {  // subStep == 2: sweeping
            // Readings glitch as the beam glances off a rock's edges, so we
            // don't trust the distance to "confirm" the rock. Instead we take
            // the FIRST start edge and the LAST end edge (collapsing any edge
            // flicker into one span) and validate by angular width below.
            Ultrasonic::EdgeEvent e = ultra.checkEdgeEvents();
            float h = sweepHeadingDeg(SWEEP_ARC);
            // Latch EVERY edge (accepted or not) for the OLED debug view: which
            // edge it was and how many degrees into the sweep arc it fired.
            if (e == Ultrasonic::START_EDGE || e == Ultrasonic::END_EDGE) {
                lastEdgeEvent    = e;
                lastEdgeDeltaDeg = h + SWEEP_ARC / 2.0f;  // deg from sweep start
                edgeEventSeq++;
            }
            if (e == Ultrasonic::START_EDGE) {
                if (!foundStartEdge) {          // keep the first entry
                    foundStartEdge = true;
                    startEdgeAngle = h;
                }
            } else if (e == Ultrasonic::END_EDGE && foundStartEdge) {
                foundEndEdge = true;
                endEdgeAngle = h;               // keep updating to the last exit
            }

            // Track the closest valid reading + its angle (the rock's nearest
            // point), used as the aim point in AIM_MIN_DISTANCE mode.
            float d = ultra.filteredDistanceCm;
            if (d > 0 && d < minSweepDistance) {
                minSweepDistance = d;
                minSweepAngle = h;
            }

            // Accumulate the angular centroid: average the heading over every
            // sample where the edge detector says we are on the object. Uses the
            // whole silhouette, so a single surface bump can't drag the aim off
            // centre the way the nearest-point does on a rough rock.
            if (ultra.scanState == Ultrasonic::TRACKING_OBJECT) {
                sumOnRockAngle += h;
                onRockCount++;
            }

            if (driveIdle()) {
                // A real rock subtends a big enough angle between its edges;
                // a noise glitch collapses to a tiny span. Require both edges
                // AND a wide enough gap.
                float rockWidth = endEdgeAngle - startEdgeAngle;
                if (rockWidth < 0) rockWidth = -rockWidth;

                if (foundStartEdge && foundEndEdge && rockWidth >= MIN_ROCK_ANGLE) {
                    float edgeMidpoint = (startEdgeAngle + endEdgeAngle) / 2.0f;
                    switch (aimMode) {
                        case AIM_CENTROID:
                            // Fall back to the edge midpoint if we somehow have
                            // no on-rock samples.
                            rockBearing = (onRockCount > 0)
                                ? (sumOnRockAngle / (float)onRockCount)
                                : edgeMidpoint;
                            break;
                        case AIM_MIN_DISTANCE:
                            rockBearing = minSweepAngle;
                            break;
                        case AIM_EDGE_MIDPOINT:
                        default:
                            rockBearing = edgeMidpoint;
                            break;
                    }
                    enter(TRAVEL_TO_ROCK);
                } else {
                    // No clean low-high-low blip wide enough to be a rock.
                    enter(ADVANCE_CLUSTER);
                }
            }
        }
        break;

    // ---- n3: Travel to Rock -----------------------------------------------
    case TRAVEL_TO_ROCK:
        if (subStep == 0) {
            // Face the rock: from +SWEEP_ARC/2 to rockBearing.
            drive.turn(rockBearing - SWEEP_ARC / 2.0f, SWEEP_SPEED);
            clusterHeading += rockBearing - SWEEP_ARC / 2.0f;
            subStep = 1;
        } else if (subStep == 1) {
            if (driveIdle()) {
                drive.driveStraight(TRAVEL_MAX_MM, TRAVEL_SPEED);
                subStep = 2;
            }
        } else {  // subStep == 2: closing in
            float d = ultra.filteredDistanceCm;
            bool valid = (d > 0 && d < Ultrasonic::MAX_VALID_DISTANCE);
            if (valid && d <= GRAB_DISTANCE_CM) {
                excursionForward += drive.lastMoveDistanceMM();  // actual travel
                drive.stop();
                enter(CENTRE_ROCK);
            } else if (driveIdle()) {
                // Reached give-up distance without arriving: rock lost. Bank the
                // forward distance we just drove so, if this attempt is later
                // abandoned, ADVANCE_CLUSTER reverses it and the robot returns to
                // the arrival pose exactly like a found+centred rock (instead of
                // being left displaced forward).
                excursionForward += drive.lastMoveDistanceMM();
                approachAttempts++;
                if (approachAttempts >= APPROACH_MAX_TRIES) {
                    // Can't close on this rock (out of range, bad aim, or the
                    // sensor never reads <= GRAB_DISTANCE_CM). Abandon cleanly via
                    // ADVANCE_CLUSTER -> realign -> (stopAfterRock) HOLD, instead
                    // of re-sweeping forever and spiralling in circles.
                    enter(ADVANCE_CLUSTER);
                } else {
                    enter(FIND_ROCK);  // re-sweep and try again
                }
            }
        }
        break;

    // ---- n4: Centre on Rock (drive to the target distance) ----------------
    // TRAVEL overshoots because the averaged reading lags the motion, so here
    // we close the loop: let the reading settle, then nudge forward/back to sit
    // at GRAB_DISTANCE_CM. Per-move relative odometry: facing the rock IS the
    // fresh origin, nothing global to reset.
    case CENTRE_ROCK:
        if (subStep == 0) {
            // Let the ultrasonic filter settle now the robot is stopped.
            stateTimer = millis();
            subStep = 1;
        } else if (subStep == 1) {
            if (millis() - stateTimer < CENTRE_SETTLE_MS) break;

            float d = ultra.filteredDistanceCm;
            bool valid = (d > 0 && d < Ultrasonic::MAX_VALID_DISTANCE);
            if (!valid) {
                enter(ADVANCE_CLUSTER);  // lost the rock while aligning
                break;
            }

            float errorCm = d - GRAB_DISTANCE_CM;  // + = too far, - = too close
            if (fabs(errorCm) <= CENTRE_MARGIN_CM || centreAttempts >= CENTRE_MAX_TRIES) {
                // Centred. Run another sweep+centre pass to refine from closer in,
                // up to SWEEP_PASSES total, then move on to the scan.
                sweepPass++;
                if (sweepPass < SWEEP_PASSES) {
                    enter(FIND_ROCK);
                } else {
                    enter(TELETUBBY_SWEEP);
                }
            } else {
                // Drive the distance error: forward if too far, back if too close.
                centreAttempts++;
                drive.driveStraight(errorCm * 10.0f, CENTRE_SPEED);
                subStep = 2;
            }
        } else {  // subStep == 2: wait for the correction move, then re-measure
            if (driveIdle()) {
                excursionForward += drive.lastMoveDistanceMM();  // this correction
                subStep = 0;
            }
        }
        break;

    // ---- n5: Teletubby scan (stationary; the camera does the scanning) ----
    case TELETUBBY_SWEEP:
        if (subStep == 0) {
            if (!enableTeletubbySweep) {  // bench-test bypass
                enter(NEED_METAL);
                break;
            }
            // Rock is aligned: settle briefly before triggering the camera.
            stateTimer = millis();
            subStep = 1;
        } else if (subStep == 1) {
            if (millis() - stateTimer >= CAMERA_PRESCAN_DELAY_MS) {
                // Trigger a camera scan, then hold still for 2 s to let it look.
                // The camera link is one-way for now (checkForTeletubby fires the
                // request and always reports false), so nothing is counted yet --
                // the structure is here for when it reports a real result.
                // (POINT_TELETUBBY / RECENTRE_ROCK below are unused until the
                // camera can give a bearing to point at.)
                if (teletubbies < 2 && camera.checkForTeletubby()) {
                    teletubbies += 1;
                }
                stateTimer = millis();
                subStep = 2;
            }
        } else {  // subStep == 2: hold for the scan window
            if (millis() - stateTimer >= TELETUBBY_SCAN_MS) {
                enter(NEED_METAL);
            }
        }
        break;

    // ---- n9: Point to teletubby -------------------------------------------
    case POINT_TELETUBBY:
        // We are already pointing at the blob (stopped mid-sweep).
        if (subStep == 0) {
            teletubbies += 1;
            stateTimer = millis();
            subStep = 1;
        } else {
            if (millis() - stateTimer >= POINT_DWELL_MS) {
                enter(RECENTRE_ROCK);
            }
        }
        break;

    // ---- n24: Re-centre on Rock -------------------------------------------
    case RECENTRE_ROCK:
        if (subStep == 0) {
            // Pivot back from the blob to rock-forward.
            drive.turn(-teletubbyBearing, SWEEP_SPEED);
            subStep = 1;
        } else {
            if (driveIdle()) {
                float d = ultra.filteredDistanceCm;
                bool reacquired = (d > 0 && d < GRAB_DISTANCE_CM + CENTRE_MARGIN_CM);
                if (reacquired) {
                    enter(NEED_METAL);
                } else {
                    enter(ADVANCE_CLUSTER);  // re-centre fails
                }
            }
        }
        break;

    // ---- n22: Still need the metal rock? ----------------------------------
    case NEED_METAL:
        if (rock == 1 || !enableMetalScan) {
            enter(ADVANCE_CLUSTER);  // already have it (or scan bypassed)
        } else {
            enter(LOWER_CLAW);
        }
        break;

    // ---- n6: Lower claw ---------------------------------------------------
    case LOWER_CLAW:
        if (subStep == 0) {
            claw.lowerToHover();          // close -> hover -> open, stop at hover
            subStep = 1;
        } else if (subStep == 1) {
            // Let the frequency filter settle at hover before baselining.
            if (!claw.actionBusy()) {
                stateTimer = millis();
                subStep = 2;
            }
        } else if (subStep == 2) {
            if (millis() - stateTimer < HOVER_SETTLE_MS) break;
            // Baseline the detector HERE, at hover: the coil is extended out
            // front, clear of the rock and the metal at the back of the robot.
            // tare() zeroes the baseline to the current (settled) reading. Only
            // touch the hardware when using the real detector.
            if (testMetalOnRock == 0) {
                metal.tare();
            }
            claw.lowerToRock();           // hover -> down onto the rock
            subStep = 3;
        } else {
            if (!claw.actionBusy()) enter(SCAN_METAL);
        }
        break;

    // ---- n7: Scan for metal -----------------------------------------------
    case SCAN_METAL:
        if (subStep == 0) {
            // Claw just reached the rock: let the lowering spike flush out of
            // the frequency filter before we trust the shift.
            stateTimer = millis();
            subStep = 1;
        } else if (subStep == 1) {
            if (millis() - stateTimer < SCAN_SETTLE_MS) break;
            metalSampleCount = 0;
            metalAllAbove = true;
            stateTimer = millis();
            subStep = 2;
        } else {  // subStep == 2: take several spaced confirmation samples
            if (millis() - stateTimer < METAL_SAMPLE_SPACING_MS) break;
            stateTimer = millis();

            bool aboveThreshold;
            if (testMetalOnRock != 0) {
                // TEST override: metal only on rock N (>0). Any other non-zero
                // value (e.g. -1) forces every rock to read as a decoy.
                aboveThreshold = (testMetalOnRock > 0) && (rocks_visited + 1 == testMetalOnRock);
            } else {
                aboveThreshold = metal.isMetalDetected();
            }

            if (!aboveThreshold) metalAllAbove = false;  // one miss = not metal
            metalSampleCount++;

            if (metalSampleCount >= METAL_SAMPLE_COUNT) {
                // Grab only if EVERY sample cleared the threshold. A transient
                // spike decays before all samples land, so it can't misfire.
                if (metalAllAbove) {
                    enter(ENGAGE_CLAW);   // sustained metal
                } else {
                    enter(RAISE_CLAW);    // decoy / spike only
                }
            }
        }
        break;

    // ---- n8: Engage Claw --------------------------------------------------
    case ENGAGE_CLAW:
        if (subStep == 0) {
            claw.closeHand();
            subStep = 1;
        } else if (!claw.actionBusy()) {
            enter(STORE_ROCK);
        }
        break;

    // ---- n10: Store Rock in Basket ----------------------------------------
    case STORE_ROCK:
        if (subStep == 0) {
            claw.storeToBasket();
            subStep = 1;
        } else if (!claw.actionBusy()) {
            rock = 1;
            enter(ADVANCE_CLUSTER);
        }
        break;

    // ---- n16: Raise Claw --------------------------------------------------
    case RAISE_CLAW:
        if (subStep == 0) {
            claw.raiseToRest();
            subStep = 1;
        } else if (!claw.actionBusy()) {
            enter(ADVANCE_CLUSTER);
        }
        break;

    // ---- n12: Advance to next cluster -------------------------------------
    case ADVANCE_CLUSTER:
        // subStep 0-1: TEST return. Reverse the net forward distance driven this
        //   excursion. All forward motion was along rockBearing and the robot
        //   still faces that way (the claw is up here), so a straight reverse
        //   lands it back at the arrival position.
        // subStep 2-3: undo the net rotation the sweep/approach added, so the
        //   next hop is measured from the heading we ARRIVED with.
        if (subStep == 0) {
            if (returnAfterCentre && fabs(excursionForward) > 1.0f) {
                drive.driveStraight(-excursionForward, TRAVEL_SPEED);
                subStep = 1;
            } else {
                subStep = 2;   // nothing to return
            }
        } else if (subStep == 1) {
            if (driveIdle()) subStep = 2;
        } else if (subStep == 2) {
            if (fabs(clusterHeading) > 0.5f) {
                drive.turn(-clusterHeading, SWEEP_SPEED);
                clusterHeading = 0.0f;
                subStep = 3;
            } else {
                rocks_visited += 1;
                enter(ALL_DONE);
            }
        } else {  // subStep == 3
            if (driveIdle()) {
                rocks_visited += 1;
                enter(ALL_DONE);
            }
        }
        break;

    // ---- n23: All rocks done? ---------------------------------------------
    case ALL_DONE:
        if (rocks_visited >= 6) {
            phase = PANEL;  // n11: collection complete, head for the panel
        }
        enter(ROUTER);
        break;

    // ---- n27: Approach ramp -----------------------------------------------
    case RAMP_APPROACH:
        if (subStep == 0) {
            drive.driveStraight(RAMP_APPROACH_MM, RAMP_SPEED);
            stateTimer = millis();
            subStep = 1;
        } else {
            line.update();  // keep the line on the ramp run (stub for now)
            if (tilt.isOnRamp()) {
                drive.stop();
                enter(RAMP_CLIMB);
            } else if (millis() - stateTimer > RAMP_SEARCH_MAX_MS || driveIdle()) {
                drive.stop();
                enter(RAMP_RECOVERY);
            }
        }
        break;

    // ---- n25: Climb ramp --------------------------------------------------
    case RAMP_CLIMB:
        if (subStep == 0) {
            drive.driveStraight(RAMP_CLIMB_MM, RAMP_SPEED);
            stateTimer = millis();
            subStep = 1;
        } else {
            line.update();  // follow line straight up (stub for now)
            bool settled = (millis() - stateTimer > RAMP_MIN_CLIMB_MS);
            if (settled && !tilt.isOnRamp()) {
                drive.stop();
                enter(CREST);  // tilt back to flat: crest reached
            } else if (millis() - stateTimer > CLIMB_MAX_MS || driveIdle()) {
                drive.stop();
                enter(RAMP_RECOVERY);
            }
        }
        break;

    // ---- n30: Ramp recovery -----------------------------------------------
    case RAMP_RECOVERY:
        // TODO: re-acquire the line and retry the approach.
        // Last resort per the FSM: assume the crest and continue on odometry.
        enter(CREST);
        break;

    // ---- n26: Crest: re-localize ------------------------------------------
    case CREST:
        level = UPPER;
        if (rocks_visited < 4) rocks_visited = 4;  // snap to 4
        // Per-move relative odometry: distance origin resets on the next move.
        // NOTE: crest fixes DISTANCE origin, not HEADING.
        enter(ROUTER);
        break;

    // ---- terminal: PANEL / DONE out of scope this pass --------------------
    case HOLD:
        drive.stop();
        break;
    }
}
