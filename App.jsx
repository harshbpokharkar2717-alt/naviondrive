import React, { useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader.js';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader.js';
import { clone as cloneSkeleton } from 'three/examples/jsm/utils/SkeletonUtils.js';

const TRACK_LENGTH = 1600;
const ROAD_WIDTH = 18;
const SHOULDER_WIDTH = 5;
const LANE_WIDTH = ROAD_WIDTH / 4;

const EGO_SPEED_KMH = 150;
const EGO_SPEED_MS = EGO_SPEED_KMH / 3.6;
const EGO_WAIT_SPEED_KMH = 50;
const EGO_WAIT_SPEED_MS = EGO_WAIT_SPEED_KMH / 3.6;
const EGO_YIELD_TRIGGER_GAP = 38;
const EGO_FRONT_PROTECTED_GAP = 95;
const TRAFFIC_COUNT = 22;
const INITIAL_TRAFFIC_COUNT = 15;
const MAX_TATA_TRUCKS = 1;
const EXOTIC_TARGETS = { gt3: 2, ferrariF40: 1, ferrariF40B: 1 };
const EGO_LENGTH = 4.95;
const EGO_WIDTH = 2.05;
const EGO_COLLISION_LATERAL = 2.0;
const LANE_CHANGE_LATERAL_BUFFER = 0.85;
const EGO_SAFE_FRONT_GAP = 30;
const EGO_SAFE_REAR_GAP = 18;
const EGO_EMERGENCY_FRONT_GAP = 8;
const EGO_THINK_TIME = 0.35;
const EGO_CHANGE_FRONT_GAP = 24;
const EGO_CHANGE_REAR_GAP = 16;
const EGO_BRAKE_SPEED_KMH = 38;
const PHYSICAL_COLLISION_BUFFER = 0.95;
const PHYSICAL_HOLD_SECONDS = 0.18;
const HARD_FOLLOW_BUFFER = 2.2;
const EGO_PHYSICAL_BUFFER = 2.4;
const EGO_COLLISION_SEARCH_STEPS = 12;
const EGO_CORRIDOR_LENGTH = 95;
const EGO_CORRIDOR_WIDTH = 1.35;
const EGO_CORRIDOR_START = 12;
const EGO_CORRIDOR_BEHIND = 28;
const EGO_VISIBLE_TRAFFIC_BEHIND = 110;
const EGO_VISIBLE_TRAFFIC_AHEAD = 420;

// NAVION DEMO MODE: deterministic, scripted traffic scenarios for repeatable presentations.
const SCRIPTED_SIMULATION = true;
const SCRIPT_PHASE_DURATION = 8.0;
const SCRIPT_PHASES = [
  { name: 'SINGLE LEFT OVERTAKE', targetLane: 0, leadLane: 1, leadSpeed: 72, leadGap: 90, maneuverAt: 0.9, returnAt: 4.8 },
  { name: 'SINGLE RIGHT OVERTAKE', targetLane: 2, leadLane: 1, leadSpeed: 76, maneuverAt: 1.5, returnAt: 5.2 },
  { name: 'TWO-LANE OVERTAKE', targetLane: 3, leadLane: 1, leadSpeed: 70, maneuverAt: 1.6, returnAt: 5.6 },
  { name: 'LEAD VEHICLE YIELDS', targetLane: 1, leadLane: 1, leadSpeed: 82, maneuverAt: 1.7, returnAt: null }
];

// Three-stage forward radar model: detect -> predict -> trigger maneuver.
const RADAR_OUTER_RANGE = 60;
const RADAR_MIDDLE_RANGE = 34;
const RADAR_INNER_RANGE = 14;
const RADAR_OUTER_HALF_ANGLE = 10;
const RADAR_MIDDLE_HALF_ANGLE = 17;
const RADAR_INNER_HALF_ANGLE = 28;

// NAVION ROAD-HAZARD DEMO: fixed potholes for repeatable avoidance.
// These are visual road hazards only; the existing traffic/overtake logic remains unchanged.
const POTHOLES = [
  { id: 'p1', s: 155, lane: 0, length: 3.40, width: 2.05, rotation: 0.10, depth: 0.18, chips: 7 },
  { id: 'p2', s: 410, lane: 2, length: 2.85, width: 1.75, rotation: -0.18, depth: 0.14, chips: 6 },
  { id: 'p3', s: 665, lane: 1, length: 3.75, width: 2.20, rotation: 0.22, depth: 0.22, chips: 9 },
  { id: 'p4', s: 925, lane: 0, length: 3.05, width: 1.90, rotation: -0.12, depth: 0.16, chips: 6 },
  { id: 'p5', s: 1190, lane: 2, length: 4.10, width: 2.35, rotation: 0.16, depth: 0.24, chips: 10 },
  { id: 'p6', s: 1435, lane: 3, length: 3.25, width: 2.00, rotation: -0.20, depth: 0.19, chips: 8 }
];
const POTHOLE_DETECT_RANGE = 72;
const POTHOLE_CLEAR_GAP = 13;
const POTHOLE_LANE_BUFFER = 0.58;
const POTHOLE_AVOID_DURATION = 0.95;
const POTHOLE_LATERAL_CLEARANCE = 1.05;
// In the fixed demo, road hazards must produce a visible lane change rather
// than getting stuck behind an overly conservative traffic prediction.
const POTHOLE_FORCE_DETECT_RANGE = 92;
const POTHOLE_FORCE_LANE_CHANGE_GAP = 58;

// NAVION ROAD-CROSSING DEMO: deterministic pedestrian/animal crossing events.
// A roadside actor moves onto the carriageway, stops in the Dodge's lane long enough
// to be clearly detected, lets the Dodge dodge around it, then exits to the shoulder.
const CROSSING_EVENT_INTERVAL = 18.0;
// The actor now enters the road, settles into ONE lane, stays there long enough
// for the Dodge to clearly detect and avoid it, and only then leaves the road.
const CROSSING_START_GAP = 100;
const CROSSING_ENTRY_DURATION = 1.15;
const CROSSING_HOLD_DURATION = 5.0;
const CROSSING_EXIT_DURATION = 1.8;
const CROSSING_START_LATERAL = ROAD_WIDTH / 2 + 3.0;
const CROSSING_DETECT_GAP = 82;
const CROSSING_CLEAR_GAP = 10;

const MAIN_CAR = {
  path: ['/models/cars/2025_mclaren_w1.glb', '/models/cars/mclaren_w1.glb', '/models/cars/2025-McLaren-W1.glb', '/models/cars/e5a7f977-0994-4b6a-8881-d228aa7658f3.glb'],
  length: 4.95,
  rotateY: Math.PI,
  label: 'McLaren W1'
};

const TRAFFIC_DEFS = {
  hilux: {
    label: 'Toyota Hilux', loader: 'gltf',
    paths: ['/models/cars/2022_toyota_hilux.glb', '/models/cars/toyota_hilux.glb', '/models/cars/hilux.glb', '/models/cars/73d46a8d-f8f8-48a1-be7b-a7c4b33ea411.glb'],
    targetLength: 5.3, targetWidth: 2.05, speedMin: 48, speedMax: 68,
    rotateY: 0, color: '#91a6ad', spawnWeight: 1.8
  },
  bmw: {
    label: 'BMW 750Li', loader: 'gltf',
    paths: ['/models/cars/2020_bmw_750li_xdrive.glb', '/models/cars/bmw_750li.glb', '/models/cars/2020-BMW-750Li-xDrive.glb', '/models/cars/9cc25d86-98e0-4c28-813e-b9f281580905.glb'],
    targetLength: 5.15, targetWidth: 2.0, speedMin: 52, speedMax: 72,
    rotateY: Math.PI, color: '#7f8e99', spawnWeight: 1.6
  },
  ford: {
    label: 'Ford F-150 Raptor', loader: 'gltf',
    paths: ['/models/cars/ford-f-150_raptor.glb', '/models/cars/ford_f150_raptor.glb', '/models/cars/ford_f_150_raptor.glb', '/models/cars/f150_raptor.glb', '/models/cars/dc1fb327-3baa-40ff-9734-4d6237778bd3.glb', '/models/cars/489ed652-876b-4bf8-8843-318a89dbe2b8.glb'],
    targetLength: 5.5, targetWidth: 2.15, speedMin: 50, speedMax: 70,
    rotateY: Math.PI, color: '#c6ccd0', spawnWeight: 1.2
  },
  tataTruck: {
    label: 'Tata Signa Cargo Truck', loader: 'gltf',
    paths: ['/models/cars/tata_signa_cargo_truck__low_poly_game_ready_pbr.glb', '/models/cars/tata_signa_cargo_truck.glb', '/models/cars/tata_signa_cargo_truck_low_poly.glb', '/models/cars/truck_low_poly.glb', '/models/cars/2936d13a-5bd4-4728-a2da-73dac5f631c1.glb'],
    targetLength: 11.6, targetWidth: 2.55, speedMin: 34, speedMax: 46,
    rotateY: Math.PI, color: '#d78d48', heavy: true, spawnWeight: 0.55
  },
  dodgeChallenger: {
    label: 'Dodge Challenger SRT Demon', loader: 'gltf',
    paths: ['/models/cars/main car/2018_dodge_challenger_srt_demon.glb'],
    targetLength: 4.95, targetWidth: 2.05, speedMin: 58, speedMax: 82,
    rotateY: Math.PI, color: '#c91522', spawnWeight: 2.2
  },
  mclaren: {
    label: 'McLaren W1', loader: 'gltf',
    paths: ['/models/cars/2025_mclaren_w1.glb', '/models/cars/mclaren_w1.glb', '/models/cars/2025-McLaren-W1.glb', '/models/cars/e5a7f977-0994-4b6a-8881-d228aa7658f3.glb'],
    targetLength: 4.6, targetWidth: 1.95, speedMin: 70, speedMax: 98,
    rotateY: Math.PI, color: '#dfe5e8', sports: true, spawnWeight: 6.0
  },
  gt3: {
    label: 'Porsche 992 GT3 RS', loader: 'gltf',
    paths: ['/models/cars/porsche_992_gt3_rs.glb', '/models/cars/porsche_911_gt2_1993.glb', '/models/cars/783170da-2eab-4e95-b586-fada88d47004.glb'],
    targetLength: 4.6, targetWidth: 1.98, speedMin: 66, speedMax: 94,
    rotateY: Math.PI, color: '#bfc3c6', sports: true, spawnWeight: 3.0
  },
  ferrariF40: {
    label: 'Ferrari F40', loader: 'gltf',
    paths: ['/models/cars/ferrari/source/lb-works_ferrari_f40_-__free.glb'],
    targetLength: 4.45, targetWidth: 1.95, speedMin: 72, speedMax: 100,
    rotateY: Math.PI, color: '#b51f2a', sports: true, spawnWeight: 2.5
  },
  ferrariF40B: {
    label: 'Ferrari F40 (Alt)', loader: 'gltf',
    paths: ['/models/cars/ferrari 2/source/lb-works_ferrari_f40_-__free.glb'],
    targetLength: 4.45, targetWidth: 1.95, speedMin: 70, speedMax: 96,
    rotateY: Math.PI, color: '#8d1e2a', sports: true, spawnWeight: 2.2
  },
  bike: {
    label: 'Normal Bike', loader: 'gltf',
    paths: ['/models/bikes/normal bike/source/8788 7  80884.glb'],
    targetLength: 1.75, targetWidth: 0.95, speedMin: 54, speedMax: 78,
    rotateY: 0, color: '#f7c84c', recklessCapable: true, spawnWeight: 1.8
  }
};

const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

function roadCenterX(s) {
  return (
    6.5 * Math.sin(s / 300) +
    2.2 * Math.sin(s / 710 + 0.7)
  );
}

function roadSlope(s) {
  return (
    (6.5 / 300) * Math.cos(s / 300) +
    (2.2 / 710) * Math.cos(s / 710 + 0.7)
  );
}

function roadPoint(s, lateral = 0, y = 0) {
  const u = ((s % TRACK_LENGTH) + TRACK_LENGTH) % TRACK_LENGTH;
  const centerX = roadCenterX(u);
  const slope = roadSlope(u);

  const nx = 1 / Math.sqrt(1 + slope * slope);
  const nz = slope / Math.sqrt(1 + slope * slope);

  return new THREE.Vector3(
    centerX + lateral * nx,
    y,
    -u + lateral * nz
  );
}

function roadHeading(s) {
  return -Math.atan2(roadSlope(s), 1);
}

function forwardVector(heading) {
  return new THREE.Vector3(
    Math.sin(heading),
    0,
    -Math.cos(heading)
  );
}

function pickWeightedTrafficKey(available) {
  const total = available.reduce((sum, key) => {
    return sum + (TRAFFIC_DEFS[key].spawnWeight || 1);
  }, 0);

  let r = Math.random() * total;

  for (const key of available) {
    r -= TRAFFIC_DEFS[key].spawnWeight || 1;
    if (r <= 0) return key;
  }

  return available[available.length - 1];
}

function signedTrackGap(aS, bS) {
  let gap = (aS - bS + TRACK_LENGTH) % TRACK_LENGTH;
  if (gap > TRACK_LENGTH / 2) gap -= TRACK_LENGTH;
  return gap;
}

function laneClearForTraffic(sim, lane, s, ignoreId = null) {
  let frontGap = Infinity;
  let backGap = Infinity;

  for (const other of sim.vehicles) {
    if (other.id === ignoreId) continue;
    if (Math.abs(other.lane - lane) > 0.38) continue;

    const gap = signedTrackGap(other.s, s);
    if (gap >= 0) frontGap = Math.min(frontGap, gap);
    else backGap = Math.min(backGap, Math.abs(gap));
  }

  // Treat the ego car as a moving participant in the target lane.
  if (Math.abs(sim.egoLane - lane) <= 0.38) {
    const egoGap = signedTrackGap(sim.egoS, s);
    if (egoGap >= 0) frontGap = Math.min(frontGap, egoGap);
    else backGap = Math.min(backGap, Math.abs(egoGap));
  }

  return { frontGap, backGap };
}

function chooseTrafficOvertakeLane(sim, vehicle) {
  const candidates = [vehicle.lane - 1, vehicle.lane + 1]
    .filter(lane => lane >= 0 && lane <= 3)
    .filter(lane => !wouldCreateEgoSideBySide(sim, lane, vehicle.s, vehicle.id, 65));

  let best = null;
  let bestScore = -Infinity;

  for (const lane of candidates) {
    const { frontGap, backGap } = laneClearForTraffic(sim, lane, vehicle.s, vehicle.id);
    const minFront = vehicle.driverType === 'reckless' ? 13 : 18;
    const minBack = vehicle.driverType === 'reckless' ? 11 : 17;

    if (frontGap < minFront || backGap < minBack) continue;

    const egoPenalty = Math.abs(lane - sim.egoLane) < 0.35 ? 18 : 0;
    const score = Math.min(frontGap, backGap) - egoPenalty;

    if (score > bestScore) {
      bestScore = score;
      best = lane;
    }
  }

  return best;
}

function trafficWidth(key) {
  return TRAFFIC_DEFS[key]?.targetWidth || 2.0;
}

// When the ego vehicle is intentionally following slowly, the lead vehicle
// should yield if a safe neighbouring lane exists. This keeps the ego lane clear
// instead of making the Dodge sit behind the same vehicle indefinitely.
function chooseYieldLaneForEgo(sim, vehicle) {
  const candidates = [vehicle.lane - 1, vehicle.lane + 1]
    .filter(lane => lane >= 0 && lane <= 3);

  let best = null;
  let bestScore = -Infinity;

  for (const lane of candidates) {
    const gaps = laneClearForTraffic(sim, lane, vehicle.s, vehicle.id);
    const minFront = 18;
    const minRear = 14;
    if (gaps.frontGap < minFront || gaps.backGap < minRear) continue;

    // Never move into the ego lane or a lane that is part of the ego's current
    // manoeuvre. Prefer the lane with the largest usable corridor.
    const score = Math.min(gaps.frontGap, gaps.backGap) +
      (lane === 1 || lane === 2 ? 2 : 0) -
      (Math.abs(lane - sim.egoLane) < 0.35 ? 100 : 0);

    if (score > bestScore) {
      bestScore = score;
      best = lane;
    }
  }

  return best;
}

function laneCenterLateral(lane) {
  return (lane - 1.5) * LANE_WIDTH;
}

// Road-hazard detector: unlike traffic, potholes are static world objects.
// Detection is based on the Dodge's projected lane and longitudinal gap, so the
// AI can identify a pothole before it reaches the vehicle and choose a safe
// neighbouring lane without changing the traffic density or radar system.
function findUpcomingPothole(sim) {
  let nearest = null;
  let nearestGap = Infinity;

  // Check the lane the Dodge is actually occupying and the lane it is
  // committed to. This prevents a pothole from being missed while the car is
  // already performing a smooth traffic overtake.
  const currentLane = clamp(Math.round(sim.egoLane), 0, 3);
  const plannedLane = Number.isFinite(sim.targetLane)
    ? clamp(Math.round(sim.targetLane), 0, 3)
    : currentLane;
  const lanesToCheck = new Set([currentLane, plannedLane]);
  const laneStep = plannedLane >= currentLane ? 1 : -1;
  for (let lane = currentLane; lane !== plannedLane; lane += laneStep) {
    lanesToCheck.add(lane);
  }

  for (const pothole of POTHOLES) {
    const gap = signedTrackGap(pothole.s, sim.egoS);
    if (gap <= POTHOLE_CLEAR_GAP || gap > POTHOLE_FORCE_DETECT_RANGE) continue;

    const laneMatch = [...lanesToCheck].some(lane =>
      Math.abs(pothole.lane - lane) <= POTHOLE_LANE_BUFFER
    );
    if (!laneMatch) continue;

    if (gap < nearestGap) {
      nearest = pothole;
      nearestGap = gap;
    }
  }

  return nearest ? { pothole: nearest, gap: nearestGap } : null;
}

function choosePotholeAvoidanceLane(sim, pothole) {
  const current = clamp(Math.round(sim.egoLane), 0, 3);
  const candidates = [current - 1, current + 1, current - 2, current + 2]
    .filter(lane => lane >= 0 && lane <= 3)
    // NEVER select the pothole's own lane as the avoidance lane.
    // The scripted overtake can temporarily set targetLane onto the pothole,
    // so this explicit exclusion is what makes the road hazard truly dodgeable.
    .filter(lane => Math.abs(lane - pothole.lane) > POTHOLE_LANE_BUFFER)
    .sort((a, b) => Math.abs(a - current) - Math.abs(b - current));

  // First use the normal traffic-aware planner.
  for (const lane of candidates) {
    const gaps = laneClearForTraffic(sim, lane, sim.egoS);
    const minFront = 20;
    const minRear = 14;
    if (gaps.frontGap < minFront || gaps.backGap < minRear) continue;

    const arrivalTime = Math.max(0.35, potholeGapToTime(sim, pothole));
    const arrivalS = (sim.egoS + EGO_SPEED_MS * arrivalTime) % TRACK_LENGTH;
    let blockedAtHazard = false;

    for (const v of sim.vehicles) {
      const vLane = trafficLaneAtTime(v, arrivalTime);
      if (Math.abs(vLane - lane) > 0.48) continue;
      const vS = (v.s + (v.speed / 3.6) * arrivalTime) % TRACK_LENGTH;
      const longitudinal = Math.abs(signedTrackGap(vS, arrivalS));
      const required = Math.max(11, (TRAFFIC_DEFS[v.key]?.targetLength || 5) * 0.8 + 4);
      if (longitudinal < required) {
        blockedAtHazard = true;
        break;
      }
    }
    if (blockedAtHazard) continue;

    if (!egoLanePathSafe(sim, lane, null, Math.abs(lane - current) > 1 ? 1.35 : 0.88)) continue;
    return lane;
  }

  // Fixed presentation mode: if traffic prediction is temporarily too
  // conservative, still choose the physically closest adjacent lane. The
  // animation has a collision envelope, so this is preferable to driving
  // directly over a clearly visible road hazard.
  if (SCRIPTED_SIMULATION) {
    const adjacent = candidates.filter(lane =>
      Math.abs(lane - current) === 1 &&
      Math.abs(lane - pothole.lane) > POTHOLE_LANE_BUFFER
    );
    let fallback = null;
    let bestScore = -Infinity;

    for (const lane of adjacent) {
      const gaps = laneClearForTraffic(sim, lane, sim.egoS);
      const score = Math.min(gaps.frontGap, gaps.backGap) +
        Math.abs(lane - pothole.lane) * 8 +
        ((lane === 1 || lane === 2) ? 3 : 0);
      if (score > bestScore) {
        bestScore = score;
        fallback = lane;
      }
    }
    return fallback;
  }

  return null;
}

function potholeGapToTime(sim, pothole) {
  const gap = Math.max(0, signedTrackGap(pothole.s, sim.egoS));
  return gap / Math.max(EGO_SPEED_MS, 1);
}
// The green corridor is the Dodge's projected safe passing envelope. During a
// committed lane change it follows the full swept path, so traffic is discouraged
// from sitting beside the Dodge in the lane(s) the Dodge needs to cross.
function egoCorridorLaneAtGap(sim, gap) {
  const ahead = clamp(gap, 0, EGO_CORRIDOR_LENGTH);
  const fromLane = sim.egoLane;
  const toLane = Number.isFinite(sim.targetLane) ? sim.targetLane : sim.egoLane;

  if (Math.abs(toLane - fromLane) < 0.02) return fromLane;

  const changeDistance = Math.max(24, EGO_SPEED_MS * (Math.abs(toLane - fromLane) > 1 ? 1.35 : 0.92));
  const p = clamp(ahead / changeDistance, 0, 1);
  const smooth = p * p * (3 - 2 * p);
  return fromLane + (toLane - fromLane) * smooth;
}

function getEgoReservedPassingLanes(sim) {
  const currentLane = clamp(Math.round(sim.egoLane), 0, 3);
  const lanes = new Set();

  // During an actual manoeuvre, reserve the complete swept lane sequence.
  // This is what prevents a second vehicle from filling the middle lane while
  // the Dodge is attempting a two-lane pass.
  const targetLane = Number.isFinite(sim.targetLane) ? clamp(Math.round(sim.targetLane), 0, 3) : currentLane;
  const step = targetLane >= currentLane ? 1 : -1;
  if (targetLane !== currentLane && (sim.controllerMode === 'OVERTAKE' || sim.controllerMode === 'THINK' || sim.controllerMode === 'FOLLOW_WAIT')) {
    for (let lane = currentLane; lane !== targetLane + step; lane += step) lanes.add(lane);
  }

  // While cruising, always keep ONE adjacent passing lane available. Pick the
  // side with the least traffic in the protected window so the scene still
  // looks natural instead of permanently emptying a fixed lane.
  if (lanes.size === 0) {
    const candidates = [currentLane - 1, currentLane + 1].filter(l => l >= 0 && l <= 3);
    let bestLane = null;
    let bestScore = -Infinity;
    for (const lane of candidates) {
      let score = 0;
      for (const v of sim.vehicles) {
        const vLane = trafficLaneAtTime(v, 0);
        if (Math.abs(vLane - lane) > 0.42) continue;
        const gap = signedTrackGap(v.s, sim.egoS);
        if (gap >= -EGO_CORRIDOR_BEHIND && gap <= EGO_CORRIDOR_LENGTH) {
          // Strongly prefer a lane that is already mostly empty.
          score -= gap >= 0 ? 2.0 : 1.5;
        }
      }
      // Prefer the inner lanes slightly, but only after occupancy is considered.
      score += (lane === 1 || lane === 2) ? 1.5 : 0;
      if (score > bestScore) { bestScore = score; bestLane = lane; }
    }
    if (bestLane !== null) lanes.add(bestLane);
  }

  return lanes;
}

function trafficConflictsWithEgoCorridor(sim, vehicle, lane, s) {
  const gap = signedTrackGap(s, sim.egoS);
  if (gap < -EGO_CORRIDOR_BEHIND || gap > EGO_CORRIDOR_LENGTH) return false;

  const reservedLanes = getEgoReservedPassingLanes(sim);
  for (const reservedLane of reservedLanes) {
    if (Math.abs(lane - reservedLane) < 0.46) return true;
  }

  // During an active lane change also protect the interpolated swept path.
  if (sim.controllerMode === 'OVERTAKE' || sim.controllerMode === 'THINK' || sim.controllerMode === 'FOLLOW_WAIT') {
    const corridorLane = egoCorridorLaneAtGap(sim, Math.max(0, gap));
    if (Math.abs(lane - corridorLane) < 0.46) return true;
  }

  return false;
}

// Keep both lanes beside the Dodge from being occupied at the same time in
// its immediate passing window. This prevents the AI from being boxed in by
// two cars running side-by-side next to the ego vehicle.
function wouldCreateEgoSideBySide(sim, candidateLane, candidateS, ignoreId = null, window = 65) {
  const egoLane = Math.round(sim.egoLane);
  const adjacent = [egoLane - 1, egoLane + 1].filter(lane => lane >= 0 && lane <= 3);
  if (!adjacent.includes(candidateLane) || adjacent.length < 2) return false;

  const otherAdjacentLane = adjacent.find(lane => lane !== candidateLane);
  if (otherAdjacentLane === undefined) return false;

  for (const v of sim.vehicles) {
    if (v.id === ignoreId) continue;
    const vLane = trafficLaneAtTime(v, 0);
    if (Math.abs(vLane - otherAdjacentLane) > 0.42) continue;
    const gap = signedTrackGap(v.s, sim.egoS);
    if (gap >= -EGO_CORRIDOR_BEHIND && gap <= Math.max(window, EGO_CORRIDOR_LENGTH)) return true;
  }

  return false;
}

function requiredLateralCenterGap(vehicleWidth, extra = LANE_CHANGE_LATERAL_BUFFER) {
  return (EGO_WIDTH / 2) + (vehicleWidth / 2) + extra;
}

function physicalCollisionDistance(def, extra = 0.75) {
  // Conservative center-to-center collision shell in the road plane.
  const egoRadius = Math.max(3.0, Math.hypot(EGO_LENGTH * 0.5, EGO_WIDTH * 0.5) + 0.45);
  const otherRadius = Math.hypot(
    (def.targetLength || 4.8) * 0.5,
    (def.targetWidth || 2.0) * 0.5
  ) + 0.45;
  return egoRadius + otherRadius + extra;
}

function physicalRoadDistance(aS, aLane, bS, bLane) {
  const a = roadPoint(aS, laneCenterLateral(aLane), 0.15);
  const b = roadPoint(bS, laneCenterLateral(bLane), 0.15);
  return a.distanceTo(b);
}

function lanePathHasSideConflict(sim, targetLane, horizon = 1.8, ignoreId = null) {
  const samples = [0, 0.25, 0.5, 0.75, 1.0, 1.25, horizon];
  const changeTime = 0.95;

  for (const t of samples) {
    const p = Math.min(1, t / changeTime);
    const smooth = p * p * (3 - 2 * p);
    const egoLane = sim.egoLane + (targetLane - sim.egoLane) * smooth;

    for (const v of sim.vehicles) {
      if (v.id === ignoreId) continue;
      const vp = v.targetLane === undefined
        ? v.lane
        : v.lane + (v.targetLane - v.lane) * Math.min(1, t / Math.max(v.laneChangeDuration || 1.5, 0.6));

      const lateralCenterGap = Math.abs(vp - egoLane) * LANE_WIDTH;
      const required = requiredLateralCenterGap(trafficWidth(v.key));
      if (lateralCenterGap < required) {
        const egoS = (sim.egoS + EGO_SPEED_MS * t) % TRACK_LENGTH;
        const vS = (v.s + (v.speed / 3.6) * t) % TRACK_LENGTH;
        const longitudinal = Math.abs(signedTrackGap(vS, egoS));
        if (longitudinal < 18) return true;
      }
    }
  }

  return false;
}

function egoLaneCheck(sim, lane, horizon = 1.8, ignoreId = null) {
  let minFront = Infinity;
  let minBack = Infinity;
  let conflict = false;
  const samples = [0, 0.25, 0.5, 0.8, 1.1, horizon];
  const changeTime = 1.35;

  for (const t of samples) {
    const p = Math.min(1, t / changeTime);
    const smooth = p * p * (3 - 2 * p);
    const egoLane = sim.egoLane + (lane - sim.egoLane) * smooth;
    const egoS = (sim.egoS + EGO_SPEED_MS * t) % TRACK_LENGTH;

    for (const v of sim.vehicles) {
      if (v.id === ignoreId) continue;
      const vS = (v.s + (v.speed / 3.6) * t) % TRACK_LENGTH;
      const vLane = v.targetLane === undefined
        ? v.lane
        : v.lane + (v.targetLane - v.lane) *
        Math.min(1, t / Math.max(v.laneChangeDuration || 1.5, 0.6));

      const lateral = Math.abs(vLane - egoLane) * LANE_WIDTH;
      const requiredLateral = requiredLateralCenterGap(trafficWidth(v.key));
      if (lateral > Math.max(EGO_COLLISION_LATERAL, requiredLateral + 0.35)) continue;

      const gap = signedTrackGap(vS, egoS);
      if (gap >= 0) minFront = Math.min(minFront, gap);
      else minBack = Math.min(minBack, Math.abs(gap));

      const requiredFront = t < changeTime ? EGO_CHANGE_FRONT_GAP : 22;
      const requiredBack = t < changeTime ? EGO_CHANGE_REAR_GAP : 14;

      if (gap >= 0 && gap < requiredFront) conflict = true;
      if (gap < 0 && Math.abs(gap) < requiredBack) conflict = true;
      if (Math.abs(gap) < 20 && lateral < requiredLateral) conflict = true;
    }
  }

  const sideConflict = lanePathHasSideConflict(sim, lane, Math.max(horizon, 2.2), ignoreId);

  return {
    safe: !conflict && !sideConflict && minFront > EGO_CHANGE_FRONT_GAP && minBack > EGO_CHANGE_REAR_GAP,
    minFront,
    minBack,
    score: Math.min(minFront, minBack) - (sideConflict ? 1000 : 0)
  };
}


function innerDodgeLaneOpen(sim, lane) {
  if (lane < 0 || lane > 3) return false;

  const { frontGap, backGap } = laneClearForTraffic(sim, lane, sim.egoS);
  // The inner radar is the maneuver trigger. Use an actionable gap here;
  // the long-horizon planner is intentionally not allowed to veto every dodge.
  if (frontGap < 14 || backGap < 12) return false;

  const changeTime = 0.90;
  for (const t of [0.12, 0.24, 0.38, 0.55, 0.72, 0.90]) {
    const p = Math.min(1, t / changeTime);
    const smooth = p * p * (3 - 2 * p);
    const egoLane = sim.egoLane + (lane - sim.egoLane) * smooth;
    const egoS = (sim.egoS + EGO_SPEED_MS * t) % TRACK_LENGTH;

    for (const v of sim.vehicles) {
      const vS = (v.s + (v.speed / 3.6) * t) % TRACK_LENGTH;
      const vLane = v.targetLane === undefined
        ? v.lane
        : v.lane + (v.targetLane - v.lane) *
        Math.min(1, t / Math.max(v.laneChangeDuration || 1.5, 0.6));

      const signedGap = signedTrackGap(vS, egoS);
      const targetLead = Math.abs(vLane - lane) < 0.35;

      // The car being overtaken is intentionally in the old lane; do not let
      // its physical shell block the adjacent target lane.
      if (!targetLead && Math.abs(vLane - sim.egoLane) < 0.35 && signedGap > 0) continue;

      if (Math.abs(signedGap) > 16) continue;

      const roadDistance = physicalRoadDistance(egoS, egoLane, vS, vLane);
      if (roadDistance < physicalCollisionDistance(TRAFFIC_DEFS[v.key], 0.15)) {
        return false;
      }
    }
  }

  return true;
}

function chooseBestEgoDodgeLane(sim) {
  const current = Math.round(sim.egoLane);
  const candidates = [current - 1, current + 1].filter(lane => lane >= 0 && lane <= 3);

  let best = null;
  let bestScore = -Infinity;

  for (const lane of candidates) {
    if (!innerDodgeLaneOpen(sim, lane)) continue;
    const { frontGap, backGap } = laneClearForTraffic(sim, lane, sim.egoS);
    const score = Math.min(frontGap, backGap) + (lane === 1 || lane === 2 ? 3 : 0);
    if (score > bestScore) {
      bestScore = score;
      best = lane;
    }
  }

  return best;
}

function chooseBestEgoLaneStable(sim, leadGap = Infinity) {
  const current = Math.round(sim.egoLane);
  const candidates = [current - 1, current + 1, current - 2, current + 2]
    .filter(lane => lane >= 0 && lane <= 3);

  let best = null;
  let bestScore = -Infinity;

  for (const lane of candidates) {
    const metric = egoLaneCheck(sim, lane, 1.6);
    if (!metric.safe) continue;

    const immediateBonus =
      metric.minFront > 70 ? 8 :
        metric.minFront > 45 ? 4 : 0;

    const centerBonus = lane === 1 || lane === 2 ? 2 : 0;
    const overtakeSideBonus = lane < current ? 3 : 0;
    const closePenalty = leadGap < 40 && metric.minFront < 40 ? 16 : 0;
    const score = metric.score + immediateBonus + centerBonus + overtakeSideBonus - closePenalty;

    if (score > bestScore) {
      bestScore = score;
      best = lane;
    }
  }

  return best;
}

function egoLaneAtTime(sim, targetLane, t, changeDuration = 1.8) {
  const p = Math.min(1, t / changeDuration);
  const smooth = p * p * (3 - 2 * p);
  return sim.egoLane + (targetLane - sim.egoLane) * smooth;
}

function trafficLaneAtTime(vehicle, t) {
  if (vehicle.targetLane === undefined) return vehicle.lane;
  return vehicle.lane +
    (vehicle.targetLane - vehicle.lane) *
    Math.min(1, t / Math.max(vehicle.laneChangeDuration || 1.5, 0.6));
}

// Full swept lateral-path check used before the Dodge is allowed to move
// sideways. It ignores the vehicle being overtaken because that car is expected
// to remain in the old lane while the Dodge passes.
function egoLanePathSafe(sim, targetLane, ignoreId = null, horizon = 1.0) {
  const samples = [0.08, 0.16, 0.28, 0.42, 0.58, 0.78, horizon];
  const changeDuration = Math.abs(targetLane - sim.egoLane) > 1 ? 1.35 : 0.85;

  for (const t of samples) {
    const p = Math.min(1, t / changeDuration);
    const smooth = p * p * (3 - 2 * p);
    const egoLane = sim.egoLane + (targetLane - sim.egoLane) * smooth;
    const egoS = (sim.egoS + EGO_SPEED_MS * t) % TRACK_LENGTH;

    for (const v of sim.vehicles) {
      if (v.id === ignoreId) continue;
      const def = TRAFFIC_DEFS[v.key];
      const vLane = trafficLaneAtTime(v, t);
      const vS = (v.s + (v.speed / 3.6) * t) % TRACK_LENGTH;
      const gap = signedTrackGap(vS, egoS);
      const lateral = Math.abs(vLane - egoLane) * LANE_WIDTH;
      const requiredLateral = requiredLateralCenterGap(def.targetWidth || 2.0, 0.55);

      if (lateral < requiredLateral && gap > -10 && gap < 18) return false;

      if (Math.abs(gap) < 14) {
        const worldDistance = physicalRoadDistance(egoS, egoLane, vS, vLane);
        if (worldDistance < physicalCollisionDistance(def, 0.05)) return false;
      }
    }
  }

  return true;
}

// Hard kinematic guard: before the Dodge advances even one frame, find the
// maximum distance it can safely travel without entering any traffic vehicle's
// physical envelope. This is deliberately independent of lane-center logic.
function constrainEgoAdvance(sim, requestedSpeed, dt) {
  // SIMPLE EGO SAFETY RULE:
  // A vehicle in a neighbouring lane must NEVER make the Dodge match its speed.
  // Only a vehicle that is actually in the Dodge's current lane and directly
  // ahead can reduce this frame's forward movement.
  const requestedAdvance = Math.max(0, requestedSpeed) * dt;
  if (requestedAdvance <= 0) return 0;

  const egoLane = sim.egoLane;
  const egoS = sim.egoS;
  let safeAdvance = requestedAdvance;

  for (const vehicle of sim.vehicles) {
    const vLane = trafficLaneAtTime(vehicle, dt * 0.5);
    const lateralGap = Math.abs(vLane - egoLane) * LANE_WIDTH;

    // Side-by-side traffic is ignored. It should not slow the Dodge.
    if (lateralGap > 1.9) continue;

    const gap = signedTrackGap(vehicle.s, egoS);

    // Only react to a vehicle directly ahead in the same physical corridor.
    if (gap <= 0 || gap > 9) continue;

    // Keep a small physical nose-to-tail buffer, but never return zero unless
    // the vehicle is already dangerously close.
    const holdGap = gap <= 3.2 ? 0.0 : gap - 3.2;
    safeAdvance = Math.min(safeAdvance, Math.max(0, holdGap));
  }

  // If traffic is merely beside us, keep full 150 km/h. If the lead car is
  // extremely close, allow a tiny forward movement instead of freezing the
  // simulation completely; the overtake controller can then take over.
  if (safeAdvance < 0.0005) {
    const currentLaneLead = sim.vehicles.some(v => {
      const vLane = trafficLaneAtTime(v, 0);
      const lateralGap = Math.abs(vLane - egoLane) * LANE_WIDTH;
      const gap = signedTrackGap(v.s, egoS);
      return lateralGap < 1.6 && gap > 0 && gap < 3.2;
    });
    return currentLaneLead ? 0 : requestedAdvance;
  }

  return safeAdvance;
}
function prepareRenderable(source, targetLength, rotateY, isMain = false) {
  const model = cloneSkeleton(source);

  model.traverse(node => {
    if (!node.isMesh) return;

    node.visible = true;
    node.frustumCulled = true;
    node.castShadow = false;
    node.receiveShadow = false;

    if (node.material) {
      const mats = Array.isArray(node.material) ? node.material : [node.material];
      mats.forEach(mat => {
        mat.visible = true;
        mat.transparent = false;
        mat.opacity = 1;
        mat.depthWrite = true;
        // Some downloaded GLBs have inconsistent face winding. DoubleSide keeps
        // the model visible while we normalize it; it does not change geometry.
        mat.side = THREE.FrontSide;
      });
    }
  });

  model.rotation.set(0, rotateY, 0);
  model.updateMatrixWorld(true);

  const rawBox = new THREE.Box3().setFromObject(model);
  const rawSize = new THREE.Vector3();
  rawBox.getSize(rawSize);

  const horizontalSize = Math.max(rawSize.x, rawSize.z);
  if (!Number.isFinite(horizontalSize) || horizontalSize <= 0) {
    throw new Error('Invalid 3D model bounds');
  }

  // Normalize from the model's actual horizontal footprint so cars do not
  // disappear because a long antenna/wheel/collision node dominates maxDim.
  model.scale.setScalar(targetLength / horizontalSize);
  model.updateMatrixWorld(true);

  const box = new THREE.Box3().setFromObject(model);
  const center = new THREE.Vector3();
  box.getCenter(center);

  // Center X/Z and place the lowest mesh exactly on the road.
  model.position.x -= center.x;
  model.position.z -= center.z;
  model.position.y -= box.min.y;

  if (!isMain) model.position.y += 0.015;

  model.updateMatrixWorld(true);

  // Store an orientation-independent collision shell based on the actual
  // normalized mesh footprint. This is deliberately a little larger than the
  // visible mesh so the rendered vehicles can never visibly interpenetrate.
  const finalBox = new THREE.Box3().setFromObject(model);
  const finalSize = new THREE.Vector3();
  finalBox.getSize(finalSize);
  model.userData.collisionRadius =
    0.5 * Math.hypot(finalSize.x, finalSize.z) + 0.55;
  model.userData.footprintLength = Math.max(finalSize.x, finalSize.z);
  model.userData.footprintWidth = Math.min(finalSize.x, finalSize.z);

  model.userData.normalized = true;
  model.userData.isMain = isMain;
  return model;
}

function withTimeout(promise, ms, label) {
  let timer = null;
  const timeout = new Promise((_, reject) => {
    timer = setTimeout(() => reject(new Error(`Timeout loading ${label}`)), ms);
  });

  return Promise.race([promise, timeout]).finally(() => {
    if (timer) clearTimeout(timer);
  });
}

async function loadModel(paths, loaderType, targetLength, rotateY, isMain = false) {
  const candidates = Array.isArray(paths) ? paths : [paths];
  let lastError = null;

  for (const path of candidates) {
    try {
      const loader = loaderType === 'fbx' ? new FBXLoader() : new GLTFLoader();
      const loaded = await withTimeout(
        loader.loadAsync(path),
        12000,
        path
      );

      const source = loaded.scene || loaded;
      const model = prepareRenderable(
        source,
        targetLength,
        rotateY,
        isMain
      );

      console.info('[NAVION] Loaded REAL MODEL:', path);
      return model;
    } catch (error) {
      lastError = error;
      console.warn('[NAVION] Failed candidate:', path, error?.message || error);
    }
  }

  throw lastError || new Error('No candidate model path loaded');
}

function createCurvedStrip(scene, width, materialColor, y) {
  const segments = 220;
  const positions = [];
  const indices = [];

  for (let i = 0; i <= segments; i++) {
    const s = TRACK_LENGTH * i / segments;
    const center = roadPoint(s, 0, y);
    const slope = roadSlope(s);

    const nx = 1 / Math.sqrt(1 + slope * slope);
    const nz = slope / Math.sqrt(1 + slope * slope);

    const left = new THREE.Vector3(
      center.x - nx * width / 2,
      y,
      center.z - nz * width / 2
    );

    const right = new THREE.Vector3(
      center.x + nx * width / 2,
      y,
      center.z + nz * width / 2
    );

    positions.push(
      left.x, left.y, left.z,
      right.x, right.y, right.z
    );
  }

  for (let i = 0; i < segments; i++) {
    const a = i * 2;
    const b = a + 1;
    const c = a + 2;
    const d = a + 3;

    indices.push(a, b, c, b, d, c);
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    'position',
    new THREE.Float32BufferAttribute(positions, 3)
  );
  geometry.setIndex(indices);
  geometry.computeVertexNormals();

  const mesh = new THREE.Mesh(
    geometry,
    new THREE.MeshBasicMaterial({
      color: materialColor,
      side: THREE.DoubleSide
    })
  );

  scene.add(mesh);
  return mesh;
}

function createRoad(scene) {
  // Broad roadside shoulders underneath the asphalt.
  createCurvedStrip(
    scene,
    ROAD_WIDTH + SHOULDER_WIDTH * 2,
    0x6f765b,
    -0.025
  );

  // Main asphalt.
  createCurvedStrip(
    scene,
    ROAD_WIDTH,
    0x2d3236,
    0
  );

  // Solid white edge lines.
  [-ROAD_WIDTH / 2, ROAD_WIDTH / 2].forEach(lateral => {
    const points = [];

    for (let i = 0; i <= 420; i++) {
      const s = TRACK_LENGTH * i / 700;
      points.push(
        roadPoint(s, lateral, 0.045)
      );
    }

    scene.add(
      new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({
          color: 0xf8f5e9
        })
      )
    );
  });

  // Three dashed lane dividers in ONE LineSegments object.
  // Hundreds of individual dash meshes caused unnecessary draw calls and were a
  // major source of long-session stutter.
  const dashPoints = [];
  for (const lateral of [-LANE_WIDTH, 0, LANE_WIDTH]) {
    for (let s = 6; s < TRACK_LENGTH; s += 16) {
      const endS = Math.min(s + 5.2, TRACK_LENGTH);
      const a = roadPoint(s, lateral, 0.055);
      const b = roadPoint(endS, lateral, 0.055);
      dashPoints.push(a, b);
    }
  }

  scene.add(
    new THREE.LineSegments(
      new THREE.BufferGeometry().setFromPoints(dashPoints),
      new THREE.LineBasicMaterial({ color: 0xf8f5e9 })
    )
  );

  // Short reflective edge markers.
  for (let s = 8; s < TRACK_LENGTH; s += 45) {
    [-1, 1].forEach(side => {
      const lateral = side * (ROAD_WIDTH / 2 + 0.22);
      const p = roadPoint(s, lateral, 0.08);

      const marker = new THREE.Mesh(
        new THREE.BoxGeometry(0.06, 0.18, 0.15),
        new THREE.MeshBasicMaterial({
          color: 0xf6efe0
        })
      );

      marker.position.copy(p);
      scene.add(marker);
    });
  }
}

function createPotholes(scene) {
  const group = new THREE.Group();
  group.userData.detectionMarkers = {};

  for (const pothole of POTHOLES) {
    const lateral = laneCenterLateral(pothole.lane);
    const point = roadPoint(pothole.s, lateral, 0.012);

    // A pothole is deliberately made from several irregular layers rather than
    // one flat ellipse. The broken asphalt rim, dark cavity and scattered chips
    // make it read as a real road defect at the helicopter-camera distance.
    const outer = new THREE.Mesh(
      new THREE.CircleGeometry(0.5, 13),
      new THREE.MeshBasicMaterial({
        color: 0x252729,
        transparent: true,
        opacity: 0.98,
        side: THREE.DoubleSide
      })
    );
    outer.scale.set(pothole.width, pothole.length, 1);
    outer.rotation.x = -Math.PI / 2;
    outer.rotation.z = pothole.rotation;
    outer.position.copy(point);
    outer.position.y = 0.016;

    const cavity = new THREE.Mesh(
      new THREE.CircleGeometry(0.5, 11),
      new THREE.MeshBasicMaterial({
        color: 0x070809,
        transparent: true,
        opacity: 0.98,
        side: THREE.DoubleSide
      })
    );
    cavity.scale.set(pothole.width * 0.73, pothole.length * 0.67, 1);
    cavity.rotation.x = -Math.PI / 2;
    cavity.rotation.z = pothole.rotation + 0.06;
    cavity.position.copy(point);
    cavity.position.y = 0.020;

    const innerShadow = new THREE.Mesh(
      new THREE.CircleGeometry(0.5, 9),
      new THREE.MeshBasicMaterial({
        color: 0x000000,
        transparent: true,
        opacity: 0.86,
        side: THREE.DoubleSide
      })
    );
    innerShadow.scale.set(pothole.width * 0.43, pothole.length * 0.39, 1);
    innerShadow.rotation.x = -Math.PI / 2;
    innerShadow.rotation.z = pothole.rotation - 0.10;
    innerShadow.position.copy(point);
    innerShadow.position.y = 0.024;

    // Broken asphalt chunks around the rim.
    const chipMat = new THREE.MeshBasicMaterial({
      color: 0x45484a,
      transparent: true,
      opacity: 0.92
    });
    for (let i = 0; i < pothole.chips; i++) {
      const a = (i / pothole.chips) * Math.PI * 2 + pothole.rotation;
      const radial = 0.72 + (i % 3) * 0.14;
      const chip = new THREE.Mesh(
        new THREE.CircleGeometry(0.11 + (i % 2) * 0.06, 5),
        chipMat.clone()
      );
      chip.scale.set(1.0 + (i % 3) * 0.45, 0.75 + (i % 2) * 0.35, 1);
      chip.rotation.x = -Math.PI / 2;
      chip.rotation.z = a;
      chip.position.copy(point);
      chip.position.x += Math.cos(a) * pothole.width * radial;
      chip.position.z += Math.sin(a) * pothole.length * radial;
      chip.position.y = 0.027;
      group.add(chip);
    }

    const detectionRing = new THREE.Mesh(
      new THREE.RingGeometry(1.35, 1.55, 32),
      new THREE.MeshBasicMaterial({
        color: 0xffc247,
        transparent: true,
        opacity: 0.0,
        side: THREE.DoubleSide,
        depthWrite: false
      })
    );
    detectionRing.rotation.x = -Math.PI / 2;
    detectionRing.position.copy(point);
    detectionRing.position.y = 0.040;

    group.add(outer, cavity, innerShadow, detectionRing);
    group.userData.detectionMarkers[pothole.id] = detectionRing;
  }

  scene.add(group);
  return group;
}
function createRadar(scene) {
  const group = new THREE.Group();

  // Sensor geometry uses the SAME local forward axis as the car: -Z.
  // The group is mounted onto the ego car, so it can never drift away from it.
  group.position.set(0, 1.35, -2.05);

  const addSector = (range, widthDeg, color, opacity) => {
    const half = THREE.MathUtils.degToRad(widthDeg / 2);
    const steps = 18;
    const vertices = [0, 0, 0];
    const indices = [];

    for (let i = 0; i <= steps; i++) {
      const a = -half + (2 * half * i) / steps;
      vertices.push(
        Math.sin(a) * range,
        0,
        -Math.cos(a) * range
      );
      if (i > 0) indices.push(0, i, i + 1);
    }

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    geometry.setIndex(indices);
    geometry.computeVertexNormals();

    group.add(new THREE.Mesh(
      geometry,
      new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity,
        side: THREE.DoubleSide,
        depthWrite: false
      })
    ));

    [-half, half].forEach(a => {
      group.add(new THREE.Line(
        new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(0, 0.015, 0),
          new THREE.Vector3(
            Math.sin(a) * range,
            0.015,
            -Math.cos(a) * range
          )
        ]),
        new THREE.LineBasicMaterial({
          color,
          transparent: true,
          opacity: 0.55
        })
      ));
    });
  };

  // Three explicit radar layers:
  // OUTER = object/vehicle detection only.
  // MIDDLE = relative-motion / closing-speed prediction.
  // INNER = maneuver trigger zone.
  addSector(RADAR_OUTER_RANGE, RADAR_OUTER_HALF_ANGLE * 2, 0x45cfff, 0.045);
  addSector(RADAR_MIDDLE_RANGE, RADAR_MIDDLE_HALF_ANGLE * 2, 0x249bff, 0.070);
  addSector(RADAR_INNER_RANGE, RADAR_INNER_HALF_ANGLE * 2, 0x48f0ff, 0.12);

  // Range boundary arcs make the 3 decision zones visually obvious.
  [
    [RADAR_OUTER_RANGE, 0x45cfff, 0.50],
    [RADAR_MIDDLE_RANGE, 0x249bff, 0.65],
    [RADAR_INNER_RANGE, 0x48f0ff, 0.9]
  ].forEach(([range, color, opacity]) => {
    const arcPts = [];
    const half = THREE.MathUtils.degToRad(
      range === RADAR_OUTER_RANGE ? RADAR_OUTER_HALF_ANGLE :
        range === RADAR_MIDDLE_RANGE ? RADAR_MIDDLE_HALF_ANGLE : RADAR_INNER_HALF_ANGLE
    );
    const steps = 24;
    for (let i = 0; i <= steps; i++) {
      const a = -half + (2 * half * i) / steps;
      arcPts.push(new THREE.Vector3(
        Math.sin(a) * range,
        0.02,
        -Math.cos(a) * range
      ));
    }
    group.add(new THREE.Line(
      new THREE.BufferGeometry().setFromPoints(arcPts),
      new THREE.LineBasicMaterial({
        color,
        transparent: true,
        opacity,
        depthWrite: false
      })
    ));
  });

  // Thin origin ring makes the actual sensor position obvious.
  const sensorDisc = new THREE.Mesh(
    new THREE.RingGeometry(0.18, 0.27, 24),
    new THREE.MeshBasicMaterial({
      color: 0x5bdcff,
      transparent: true,
      opacity: 0.95,
      side: THREE.DoubleSide,
      depthWrite: false
    })
  );
  sensorDisc.rotation.x = -Math.PI / 2;
  sensorDisc.position.y = 0.025;
  group.add(sensorDisc);

  const targetGeo = new THREE.BufferGeometry();
  targetGeo.setAttribute(
    'position',
    new THREE.Float32BufferAttribute(new Float32Array(6 * 2 * 3), 3)
  );
  targetGeo.setDrawRange(0, 0);

  group.userData.targetLines = new THREE.LineSegments(
    targetGeo,
    new THREE.LineBasicMaterial({
      color: 0x43cfff,
      transparent: true,
      opacity: 0.72,
      depthWrite: false
    })
  );
  group.add(group.userData.targetLines);

  group.userData.detectionMarkers = Array.from({ length: 6 }, () => {
    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.12, 8, 6),
      new THREE.MeshBasicMaterial({ color: 0x44d9ff, transparent: true, opacity: 0.9 })
    );
    marker.visible = false;
    group.add(marker);
    return marker;
  });

  return group;
}

function createEgoVehicleLinks(scene) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    'position',
    new THREE.Float32BufferAttribute(new Float32Array(40 * 2 * 3), 3)
  );
  geometry.setDrawRange(0, 0);

  const lines = new THREE.LineSegments(
    geometry,
    new THREE.LineBasicMaterial({
      color: 0x35bfff,
      transparent: true,
      opacity: 0.72,
      depthWrite: false,
      depthTest: false
    })
  );
  lines.renderOrder = 8;
  scene.add(lines);
  lines.userData.maxLinks = 40;
  return lines;
}
function createDetectionBoxes(scene, mount) {
  // Screen-space 2D detection windows. These are deliberately DOM overlays so
  // they stay aligned with the camera view instead of becoming invisible
  // world-space rectangles when the chase camera changes angle.
  const overlay = document.createElement('div');
  overlay.className = 'navion-detection-overlay';
  Object.assign(overlay.style, {
    position: 'absolute',
    inset: '0',
    zIndex: '6',
    pointerEvents: 'none',
    overflow: 'hidden'
  });
  mount.appendChild(overlay);

  const boxes = [];
  const makeBox = () => {
    const el = document.createElement('div');
    Object.assign(el.style, {
      position: 'absolute',
      boxSizing: 'border-box',
      border: '1.5px solid rgba(89,217,255,0.52)',
      background: 'rgba(89,217,255,0.025)',
      boxShadow: '0 0 3px rgba(89,217,255,0.18)',
      display: 'none',
      pointerEvents: 'none'
    });
    overlay.appendChild(el);
    boxes.push(el);
    return el;
  };

  for (let i = 0; i < 50; i++) makeBox();
  return { overlay, boxes };
}
function updateDetectionBoxes(detection, sim, roadsideLife, camera, mount) {
  if (!detection?.boxes || !sim || !camera || !mount) return;

  const boxes = detection.boxes;
  let boxIndex = 0;
  const width = mount.clientWidth;
  const height = mount.clientHeight;
  if (!width || !height) return;

  const setBoxStyle = (el, rect, color, opacity) => {
    if (!el || !rect) return;
    el.style.left = `${rect.left}px`;
    el.style.top = `${rect.top}px`;
    el.style.width = `${Math.max(5, rect.width)}px`;
    el.style.height = `${Math.max(7, rect.height)}px`;
    el.style.borderColor = color.replace('OPACITY', opacity.toFixed(2));
    el.style.background = color.replace('OPACITY', (opacity * 0.08).toFixed(3));
    el.style.boxShadow = `0 0 4px ${color.replace('OPACITY', (opacity * 0.22).toFixed(2))}`;
    el.style.display = 'block';
  };

  const projectBox = (worldBox, padding = 2) => {
    if (worldBox.isEmpty()) return null;
    const min = worldBox.min;
    const max = worldBox.max;
    const corners = [
      new THREE.Vector3(min.x, min.y, min.z), new THREE.Vector3(max.x, min.y, min.z),
      new THREE.Vector3(min.x, max.y, min.z), new THREE.Vector3(max.x, max.y, min.z),
      new THREE.Vector3(min.x, min.y, max.z), new THREE.Vector3(max.x, min.y, max.z),
      new THREE.Vector3(min.x, max.y, max.z), new THREE.Vector3(max.x, max.y, max.z)
    ];

    let left = Infinity, right = -Infinity, top = Infinity, bottom = -Infinity;
    let visiblePoint = false;
    for (const point of corners) {
      point.project(camera);
      // Ignore objects entirely behind the camera.
      if (point.z >= -1 && point.z <= 1) visiblePoint = true;
      const x = (point.x * 0.5 + 0.5) * width;
      const y = (-point.y * 0.5 + 0.5) * height;
      left = Math.min(left, x);
      right = Math.max(right, x);
      top = Math.min(top, y);
      bottom = Math.max(bottom, y);
    }

    if (!visiblePoint || right < 0 || left > width || bottom < 0 || top > height) return null;
    return {
      left: Math.max(0, left - padding),
      top: Math.max(0, top - padding),
      width: Math.min(width, right + padding) - Math.max(0, left - padding),
      height: Math.min(height, bottom + padding) - Math.max(0, top - padding)
    };
  };

  const next = () => boxes[boxIndex++];

  // VEHICLES — faint cyan windows exactly around the visible 3D model.
  const vehicles = [...sim.vehicles]
    .filter(v => !v.scriptInactive && v.root?.visible)
    .map(v => ({ v, gap: signedTrackGap(v.s, sim.egoS) }))
    .filter(item => item.gap > -8 && item.gap < 145)
    .sort((a, b) => a.gap - b.gap);

  for (const { v, gap } of vehicles) {
    if (boxIndex >= boxes.length) break;
    const el = next();
    const worldBox = new THREE.Box3().setFromObject(v.root);
    const rect = projectBox(worldBox, 2);
    if (!rect) { el.style.display = 'none'; continue; }
    setBoxStyle(el, rect, 'rgba(89,217,255,OPACITY)', gap < 65 ? 0.72 : 0.46);
  }

  // PEDESTRIANS + ANIMALS — faint 2D perception windows around the actual
  // rendered actor. Animals get their own subtle green box so they are visibly
  // classified separately from people.
  if (roadsideLife?.userData?.life) {
    const egoWorld = new THREE.Vector3();
    sim.egoRoot?.getWorldPosition(egoWorld);
    for (const actor of roadsideLife.userData.life) {
      if (boxIndex >= boxes.length || !actor.visible) break;
      const d = actor.userData || {};
      if (d.type !== 'person' && d.type !== 'dog') continue;
      const point = new THREE.Vector3();
      actor.getWorldPosition(point);
      const rel = point.clone().sub(egoWorld);
      rel.y = 0;
      const heading = roadHeading(sim.egoS);
      const forward = new THREE.Vector3(Math.sin(heading), 0, -Math.cos(heading));
      const longitudinal = rel.dot(forward);
      if (longitudinal < -10 || longitudinal > 145) continue;
      const el = next();
      const worldBox = new THREE.Box3().setFromObject(actor);
      const rect = projectBox(worldBox, 2);
      if (!rect) { el.style.display = 'none'; continue; }
      const isAnimal = d.type === 'dog';
      const crossing = d.crossingActive;
      setBoxStyle(
        el,
        rect,
        isAnimal ? 'rgba(132,255,166,OPACITY)' : 'rgba(255,211,90,OPACITY)',
        crossing ? 0.82 : longitudinal < 75 ? (isAnimal ? 0.60 : 0.62) : 0.38
      );
    }
  }

  // POTHOLES — orange ground detection windows; active hazard becomes brighter.
  for (const pothole of POTHOLES) {
    if (boxIndex >= boxes.length) break;
    const gap = signedTrackGap(pothole.s, sim.egoS);
    if (gap <= -8 || gap > 145) continue;
    const point = roadPoint(pothole.s, laneCenterLateral(pothole.lane), 0.08);
    const heading = pothole.rotation;
    const forward = new THREE.Vector3(Math.sin(heading), 0, -Math.cos(heading));
    const right = new THREE.Vector3(Math.cos(heading), 0, Math.sin(heading));
    const halfW = pothole.width * 0.5 + 0.12;
    const halfL = pothole.length * 0.5 + 0.12;
    const corners = [
      point.clone().addScaledVector(forward, halfL).addScaledVector(right, halfW),
      point.clone().addScaledVector(forward, halfL).addScaledVector(right, -halfW),
      point.clone().addScaledVector(forward, -halfL).addScaledVector(right, -halfW),
      point.clone().addScaledVector(forward, -halfL).addScaledVector(right, halfW)
    ];
    const worldBox = new THREE.Box3().setFromPoints(corners);
    worldBox.min.y = 0.05; worldBox.max.y = 0.09;
    const rect = projectBox(worldBox, 2);
    const el = next();
    if (!rect) { el.style.display = 'none'; continue; }
    const active = sim.potholeAvoidanceActive && sim.potholeId === pothole.id;
    const detected = gap <= POTHOLE_DETECT_RANGE && gap > POTHOLE_CLEAR_GAP &&
      Math.abs(pothole.lane - sim.egoLane) <= POTHOLE_LANE_BUFFER;
    setBoxStyle(el, rect, active ? 'rgba(255,99,90,OPACITY)' : 'rgba(255,194,71,OPACITY)', active ? 0.86 : detected ? 0.66 : 0.34);
  }

  for (let i = boxIndex; i < boxes.length; i++) boxes[i].style.display = 'none';
}
function findUpcomingRoadsideHazard(sim, roadsideLife) {
  if (!sim || !roadsideLife?.userData?.life) return null;

  const egoWorld = new THREE.Vector3();
  if (sim.egoRoot) sim.egoRoot.getWorldPosition(egoWorld);
  else egoWorld.copy(roadPoint(sim.egoS, laneCenterLateral(sim.egoLane), 0));

  const heading = roadHeading(sim.egoS);
  const forward = forwardVector(heading);
  let nearest = null;
  let nearestGap = Infinity;

  for (const actor of roadsideLife.userData.life) {
    const d = actor.userData || {};
    if (!actor.visible || !d.crossingActive) continue;

    const point = new THREE.Vector3();
    actor.getWorldPosition(point);
    const rel = point.clone().sub(egoWorld);
    rel.y = 0;
    const gap = rel.dot(forward);
    if (gap < CROSSING_CLEAR_GAP || gap > CROSSING_DETECT_GAP) continue;

    // Project the actor onto the local road normal at its current position.
    const road = roadPoint(d.s, 0, 0);
    const lateral = point.clone().sub(road);
    lateral.y = 0;
    const slope = roadSlope(d.s);
    const nx = 1 / Math.sqrt(1 + slope * slope);
    const nz = slope / Math.sqrt(1 + slope * slope);
    const actorLateral = lateral.x * nx + lateral.z * nz;
    const actorLane = Number.isFinite(d.crossingTargetLane)
      ? d.crossingTargetLane
      : actorLateral / LANE_WIDTH + 1.5;

    // Only trigger when the crossing is entering the road corridor and can
    // intersect the Dodge's current/target lane.
    const laneThreat = Math.abs(actorLane - sim.egoLane) < 0.78 ||
      (Number.isFinite(sim.targetLane) && Math.abs(actorLane - sim.targetLane) < 0.78);
    if (!laneThreat) continue;

    if (gap < nearestGap) {
      nearestGap = gap;
      nearest = { actor, gap, actorLane, actorLateral };
    }
  }

  return nearest;
}

function chooseRoadsideDodgeLane(sim, hazard) {
  const current = clamp(Math.round(sim.egoLane), 0, 3);
  const candidates = [current - 1, current + 1].filter(lane => lane >= 0 && lane <= 3);
  if (!candidates.length) return null;

  // Prefer the side opposite the crossing actor. If the actor is on the left
  // half of the road, dodge right; if on the right, dodge left.
  const preferred = hazard.actorLateral < 0 ? current + 1 : current - 1;
  const ordered = [preferred, ...candidates.filter(l => l !== preferred)];

  for (const lane of ordered) {
    const gaps = laneClearForTraffic(sim, lane, sim.egoS);
    if (gaps.frontGap < 18 || gaps.backGap < 14) continue;
    if (!egoLanePathSafe(sim, lane, null, 0.95)) continue;
    return lane;
  }

  // DEMO SAFETY FALLBACK ---------------------------------------------------
  // In the scripted prototype the surrounding traffic can temporarily make
  // the strict swept-path test reject both adjacent lanes.  Do not let that
  // make the ego car drive straight into the pedestrian/animal.  Pick the
  // least-conflicted adjacent lane and let the smooth lane controller move
  // the car there.  Real backend/controller logic can replace this fallback
  // later with a full trajectory planner.
  if (SCRIPTED_SIMULATION) {
    let fallback = null;
    let bestScore = -Infinity;
    for (const lane of candidates) {
      const gaps = laneClearForTraffic(sim, lane, sim.egoS);
      const score = Math.min(gaps.frontGap, gaps.backGap)
        - Math.abs(lane - current) * 1.5
        + (lane === preferred ? 8 : 0);
      if (score > bestScore) {
        bestScore = score;
        fallback = lane;
      }
    }
    if (fallback !== null) return fallback;
  }

  return null;
}

function updateRoadsideHazardPriority(sim, roadsideLife) {
  if (!sim || !roadsideLife) return;

  const hazard = findUpcomingRoadsideHazard(sim, roadsideLife);

  if (!sim.roadsideAvoidanceActive && hazard) {
    const dodgeLane = chooseRoadsideDodgeLane(sim, hazard);
    if (dodgeLane !== null && dodgeLane !== Math.round(sim.egoLane)) {
      sim.roadsideAvoidanceActive = true;
      sim.roadsideHazardActor = hazard.actor.uuid;
      sim.roadsideHazardType = hazard.actor.userData?.type || 'person';
      sim.roadsideHazardGap = hazard.gap;
      sim.roadsideReturnLane = Math.round(sim.egoLane);
      sim.roadsideTargetLane = dodgeLane;
      sim.targetLane = dodgeLane;
      sim.controllerMode = 'ROAD_CROSSING_AVOID';
      updateUI(
        sim.roadsideHazardType === 'dog' ? 'ANIMAL DETECTED — DODGING' : 'PEDESTRIAN DETECTED — DODGING',
        `ROAD HAZARD • ${hazard.gap.toFixed(0)}m AHEAD • LANE ${Math.round(hazard.actorLane) + 1}`
      );
    }
  }

  if (sim.roadsideAvoidanceActive) {
    const actor = roadsideLife.userData.life.find(a => a.uuid === sim.roadsideHazardActor);
    if (!actor || !actor.visible || !actor.userData?.crossingActive) {
      sim.roadsideAvoidanceActive = false;
      sim.roadsideHazardActor = null;
      sim.roadsideHazardType = null;
      sim.roadsideTargetLane = null;
      sim.targetLane = Number.isFinite(sim.roadsideReturnLane) ? sim.roadsideReturnLane : Math.round(sim.egoLane);
      sim.controllerMode = 'MERGE';
      updateUI('ROAD CROSSING CLEARED', `RETURNING • LANE ${sim.targetLane + 1}`);
    } else {
      sim.roadsideHazardGap = findUpcomingRoadsideHazard(sim, roadsideLife)?.gap ?? -20;
      sim.targetLane = sim.roadsideTargetLane;
      sim.controllerMode = 'ROAD_CROSSING_AVOID';
    }
  }
}

function updateEgoVehicleLinks(lines, sim, roadsideLife, radar) {
  if (!lines || !sim) return;

  // Perception links are rebuilt every render frame.  IMPORTANT: roadside
  // actors get dedicated slots so a growing stream of traffic can NEVER push
  // a newly appearing pedestrian/animal out of the link list.
  const attr = lines.geometry.getAttribute('position');
  const max = lines.userData.maxLinks || 40;
  const radarOriginWorld = new THREE.Vector3();
  if (radar) radar.getWorldPosition(radarOriginWorld);
  else radarOriginWorld.copy(roadPoint(sim.egoS, (sim.egoLane - 1.5) * LANE_WIDTH, 1.45));
  const localOrigin = lines.worldToLocal(radarOriginWorld.clone());

  const vehicles = [];
  const people = [];
  const animals = [];
  const potholes = [];

  // Vehicles: preserve the existing behavior, but don't let them consume the
  // reserved pedestrian/animal slots.
  for (const vehicle of sim.vehicles) {
    if (vehicle.scriptInactive || !vehicle.root?.visible) continue;
    const gap = signedTrackGap(vehicle.s, sim.egoS);
    if (gap < -8 || gap > 240) continue;
    if (Math.abs(vehicle.lane - sim.egoLane) > 2.5) continue;
    const point = new THREE.Vector3();
    vehicle.root.getWorldPosition(point);
    point.y += 1.15;
    vehicles.push({ type: 'vehicle', vehicle, gap, score: gap + Math.abs(vehicle.lane - sim.egoLane) * 8, point });
  }
  vehicles.sort((a, b) => a.score - b.score);

  // Every live person and dog gets evaluated independently every frame.
  // There is NO persistent candidate cache and NO global ranking that can
  // starve new actors.  The nearest upcoming actor of each type is reserved.
  if (roadsideLife?.userData?.life) {
    for (const actor of roadsideLife.userData.life) {
      if (!actor.visible) continue;
      const d = actor.userData || {};
      const point = new THREE.Vector3();
      actor.getWorldPosition(point);

      // IMPORTANT: do not use the simulation `s` value as the primary test for
      // roadside actors. Their lane is not the road lane and their recycled `s`
      // can wrap around TRACK_LENGTH. Use the actual 3D position relative to the
      // ego/radar origin instead. This makes every newly appearing pedestrian or
      // animal eligible no matter where in the 1600 m track it appears.
      const egoWorld = new THREE.Vector3();
      if (sim.egoRoot) sim.egoRoot.getWorldPosition(egoWorld);
      else egoWorld.copy(roadPoint(sim.egoS, (sim.egoLane - 1.5) * LANE_WIDTH, 0));
      const heading = roadHeading(sim.egoS);
      const forward = new THREE.Vector3(Math.sin(heading), 0, Math.cos(heading));
      const rel = point.clone().sub(egoWorld); rel.y = 0;
      const longitudinal = rel.dot(forward);
      const distance = rel.length();

      // Only detach when the actor is genuinely behind the ego. A generous
      // forward range ensures newly spawned targets later in the run are caught.
      if (longitudinal < -12 || longitudinal > 260 || distance > 285) continue;

      point.y += d.type === 'dog' ? 0.55 : 1.05;
      const lateralDistance = Math.abs(rel.x * Math.cos(heading) - rel.z * Math.sin(heading));
      if (lateralDistance > ROAD_WIDTH * 2.2) continue;

      // Use actual forward distance for ranking, never the recycled track `s`.
      const item = { type: d.type === 'dog' ? 'dog' : 'person', actor, gap: longitudinal, score: longitudinal, point };
      if (item.type === 'person') people.push(item);
      else animals.push(item);
    }
  }
  people.sort((a, b) => a.score - b.score);
  animals.sort((a, b) => a.score - b.score);

  for (const pothole of POTHOLES) {
    const gap = signedTrackGap(pothole.s, sim.egoS);
    if (gap < -8 || gap > 120) continue;
    const lateral = (pothole.lane - 1.5) * LANE_WIDTH;
    potholes.push({ type: 'pothole', pothole, gap, score: gap, point: roadPoint(pothole.s, lateral, 0.22) });
  }
  potholes.sort((a, b) => a.score - b.score);

  const selected = [];
  const keys = new Set();
  const add = item => {
    if (!item || selected.length >= max) return;
    const key = item.vehicle ? `v:${item.vehicle.id}` : item.actor ? `a:${item.actor.uuid}` : item.pothole ? `p:${item.pothole.id}` : null;
    if (key && keys.has(key)) return;
    if (key) keys.add(key);
    selected.push(item);
  };

  // HARD RESERVATION: always give the closest upcoming pedestrian and dog a
  // link slot whenever one exists. This is what prevents "works at the start,
  // then only cars" after new actors enter the simulation.
  add(people[0]);
  add(animals[0]);

  // Keep additional life targets when capacity allows.
  for (const item of people.slice(1, 3)) add(item);
  for (const item of animals.slice(1, 3)) add(item);

  // Then fill remaining capacity with the normal vehicle links.
  for (const item of vehicles) add(item);

  // Finally add visible pothole links if there is still room.
  for (const item of potholes) add(item);

  for (let i = 0; i < selected.length; i++) {
    const item = selected[i];
    const localTarget = lines.worldToLocal(item.point.clone());
    attr.setXYZ(i * 2, localOrigin.x, localOrigin.y, localOrigin.z);
    attr.setXYZ(i * 2 + 1, localTarget.x, localTarget.y, localTarget.z);
  }

  for (let i = selected.length; i < max; i++) {
    attr.setXYZ(i * 2, 0, -1000, 0);
    attr.setXYZ(i * 2 + 1, 0, -1000, 0);
  }
  attr.needsUpdate = true;
  lines.geometry.setDrawRange(0, selected.length * 2);
}
function createPathRibbon(scene) {
  const mesh = new THREE.Mesh(
    new THREE.BufferGeometry(),
    new THREE.MeshBasicMaterial({
      color: 0x24e59a,
      transparent: true,
      opacity: 0.24,
      side: THREE.DoubleSide,
      depthWrite: false
    })
  );

  scene.add(mesh);
  return mesh;
}

function updatePathRibbon(mesh, points, width = 0.24) {
  if (points.length < 2) return;

  const neededVertices = points.length * 2;
  let position = mesh.geometry.getAttribute('position');

  if (!position || position.count !== neededVertices) {
    mesh.geometry.dispose();
    mesh.geometry = new THREE.BufferGeometry();
    position = new THREE.BufferAttribute(new Float32Array(neededVertices * 3), 3);
    mesh.geometry.setAttribute('position', position);

    const indices = [];
    for (let i = 0; i < points.length - 1; i++) {
      const a = i * 2;
      const b = a + 1;
      const c = a + 2;
      const d = a + 3;
      indices.push(a, b, c, b, d, c);
    }
    mesh.geometry.setIndex(indices);
  }

  const arr = position.array;
  for (let i = 0; i < points.length; i++) {
    const p = points[i];
    const before = points[Math.max(0, i - 1)];
    const after = points[Math.min(points.length - 1, i + 1)];
    const dx = after.x - before.x;
    const dz = after.z - before.z;
    const len = Math.sqrt(dx * dx + dz * dz) || 1;
    const nx = -dz / len;
    const nz = dx / len;
    const o = i * 6;
    arr[o] = p.x + nx * width / 2;
    arr[o + 1] = p.y;
    arr[o + 2] = p.z + nz * width / 2;
    arr[o + 3] = p.x - nx * width / 2;
    arr[o + 4] = p.y;
    arr[o + 5] = p.z - nz * width / 2;
  }

  position.needsUpdate = true;
  mesh.geometry.computeBoundingSphere();
}

function addTrafficIndicatorLights(root, color = 0xffa522) {
  const left = new THREE.Mesh(
    new THREE.SphereGeometry(0.055, 8, 6),
    new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.95 })
  );
  const right = left.clone();

  left.position.set(-0.9, 0.45, -0.9);
  right.position.set(0.9, 0.45, -0.9);
  left.visible = false;
  right.visible = false;

  root.add(left, right);
  root.userData.indicatorLights = { left, right };
}


function addSafetyZone(root, length, width, color = 0x51d7ff, ego = false) {
  const zone = new THREE.Mesh(
    new THREE.RingGeometry(0.88, 1.0, 24),
    new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: ego ? 0.20 : 0.10,
      side: THREE.DoubleSide,
      depthWrite: false
    })
  );
  zone.rotation.x = -Math.PI / 2;
  zone.position.y = 0.035;
  zone.scale.set(
    (width * 0.5 + 0.95) / 0.94,
    1,
    (length * 0.5 + 1.35) / 0.94
  );
  root.add(zone);
  root.userData.safetyZone = zone;
  root.userData.safetyLength = length;
  root.userData.safetyWidth = width;
  return zone;
}

function createVehicleFallback(def, main = false) {
  const group = new THREE.Group();
  const length = main ? 4.95 : def.targetLength;
  const width = def.heavy ? 2.55 : def.label.includes('Bike') ? 0.75 : 1.9;
  const bodyHeight = def.heavy ? 1.05 : def.label.includes('Bike') ? 0.95 : 0.72;
  const bodyColor = main ? 0xc91522 : new THREE.Color(def.color || '#71808a').getHex();

  const body = new THREE.Mesh(
    new THREE.BoxGeometry(width, bodyHeight, length * 0.72),
    new THREE.MeshBasicMaterial({ color: bodyColor })
  );
  body.position.set(0, def.label.includes('Bike') ? 0.58 : 0.63, length * 0.08);
  group.add(body);

  if (!def.label.includes('Bike')) {
    const hood = new THREE.Mesh(
      new THREE.BoxGeometry(width * 0.94, bodyHeight * 0.68, length * 0.25),
      new THREE.MeshBasicMaterial({ color: bodyColor })
    );
    hood.position.set(0, body.position.y + bodyHeight * 0.18, -length * 0.33);
    group.add(hood);

    const cabin = new THREE.Mesh(
      new THREE.BoxGeometry(width * 0.82, def.heavy ? 0.76 : 0.52, length * (def.heavy ? 0.32 : 0.40)),
      new THREE.MeshBasicMaterial({ color: 0x20252b })
    );
    cabin.position.set(0, body.position.y + bodyHeight * 0.68, def.heavy ? -length * 0.02 : -length * 0.04);
    group.add(cabin);
  } else {
    const rider = new THREE.Mesh(
      new THREE.SphereGeometry(0.22, 10, 10),
      new THREE.MeshBasicMaterial({ color: 0x22252a })
    );
    rider.position.set(0, 0.95, 0);
    group.add(rider);
  }

  const wheelRadius = def.label.includes('Bike') ? 0.27 : def.heavy ? 0.42 : 0.34;
  const wheelGeo = new THREE.CylinderGeometry(wheelRadius, wheelRadius, def.label.includes('Bike') ? 0.16 : 0.24, 12);
  const wheelMat = new THREE.MeshBasicMaterial({ color: 0x111111 });
  const wheelZ = length * 0.33;
  const wheelX = width * 0.54;

  for (const z of [-wheelZ, wheelZ]) {
    for (const x of [-wheelX, wheelX]) {
      const wheel = new THREE.Mesh(wheelGeo, wheelMat);
      wheel.rotation.z = Math.PI / 2;
      wheel.position.set(x, wheelRadius, z);
      group.add(wheel);
    }
  }

  return group;
}


function createRoadsideLife(scene) {
  const group = new THREE.Group();
  group.name = 'RoadsideLife';
  const life = [];

  const makePerson = (s, side, phase) => {
    const g = new THREE.Group();
    const skin = new THREE.MeshStandardMaterial({ color: 0xb87552, roughness: 0.9 });
    const shirt = new THREE.MeshStandardMaterial({ color: [0x3d6ea8, 0x9b4d62, 0xd18b35, 0x4c8f72][phase % 4], roughness: 0.85 });
    const dark = new THREE.MeshStandardMaterial({ color: 0x252936, roughness: 0.9 });
    const body = new THREE.Mesh(new THREE.CapsuleGeometry(0.22, 0.55, 4, 8), shirt);
    body.position.y = 0.82; g.add(body);
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.19, 12, 10), skin);
    head.position.y = 1.34; g.add(head);
    for (const x of [-0.11, 0.11]) {
      const leg = new THREE.Mesh(new THREE.CapsuleGeometry(0.055, 0.34, 4, 6), dark);
      leg.position.set(x, 0.36, 0); g.add(leg);
    }
    g.userData = { type: 'person', s, side, speed: 0.7 + (phase % 3) * 0.15, phase: phase * 1.7, spawnIndex: phase, lastSpawnS: s, crossingCandidate: phase === 0, crossingActive: false, crossingProgress: 0 };
    return g;
  };

  const makeDog = (s, side, phase) => {
    const g = new THREE.Group();
    const fur = new THREE.MeshStandardMaterial({ color: phase % 2 ? 0x9a6a45 : 0xe0c29a, roughness: 1 });
    const body = new THREE.Mesh(new THREE.SphereGeometry(0.34, 12, 8), fur);
    body.scale.set(1.45, 0.75, 0.75); body.position.y = 0.43; g.add(body);
    const head = new THREE.Mesh(new THREE.SphereGeometry(0.23, 12, 8), fur);
    head.position.set(0, 0.62, -0.42); g.add(head);
    for (const x of [-0.19, 0.19]) for (const z of [-0.15, 0.15]) {
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.045, 0.055, 0.34, 7), fur);
      leg.position.set(x, 0.2, z); g.add(leg);
    }
    g.userData = { type: 'dog', s, side, speed: 0.45 + (phase % 2) * 0.18, phase: phase * 1.3, spawnIndex: 10 + phase, lastSpawnS: s, crossingCandidate: phase === 0, crossingActive: false, crossingProgress: 0 };
    return g;
  };

  // Single roadside pedestrian scenario.
  const person = makePerson(55, -1, 0);
  group.add(person); life.push(person);

  scene.add(group);
  group.userData.life = life;
  return group;
}
function updateRoadsideLife(group, dt, sim) {
  if (!group?.userData?.life) return;

  if (sim) {
    sim.roadsideScenarioTime += dt;

    // Single pedestrian crossing event, placed ahead of the Dodge so the camera
    // has time to show detection, avoidance, and the return merge.
    if (sim.roadsideScenarioTime >= CROSSING_EVENT_INTERVAL && !sim.roadsideAvoidanceActive) {
      const candidates = group.userData.life.filter(a => a.userData?.type === 'person' && a.userData?.crossingCandidate);
      if (candidates.length) {
        const actor = candidates[0];
        const d = actor.userData;
        sim.roadsideScenarioIndex += 1;
        sim.roadsideScenarioTime = 0;

        d.crossingActive = true;
        d.crossingPhase = 'ENTER';
        d.crossingProgress = 0;
        d.crossingEntryDuration = CROSSING_ENTRY_DURATION;
        d.crossingHoldDuration = CROSSING_HOLD_DURATION;
        d.crossingExitDuration = CROSSING_EXIT_DURATION;
        d.crossingStartLateral = (sim.roadsideScenarioIndex % 2 === 0)
          ? -CROSSING_START_LATERAL
          : CROSSING_START_LATERAL;
        // Enter the lane the Dodge is currently travelling in. The actor then
        // remains centered in that lane instead of sweeping across the whole road.
        d.crossingTargetLane = clamp(Math.round(sim.egoLane), 0, 3);
        d.crossingTargetLateral = laneCenterLateral(d.crossingTargetLane);
        d.crossingEndLateral = -d.crossingStartLateral;
        d.side = d.crossingStartLateral < 0 ? -1 : 1;
        d.s = (sim.egoS + CROSSING_START_GAP) % TRACK_LENGTH;
        d.lastSpawnS = d.s;
        d.crossingStartedAtEgoS = sim.egoS;
      }
    }
  }

  for (const actor of group.userData.life) {
    const d = actor.userData;
    d.s = (d.s + d.speed * dt) % TRACK_LENGTH;

    if (d.crossingActive) {
      // Three-stage crossing:
      // 1) enter from the shoulder,
      // 2) STOP/SLOW in one road lane so the Dodge has to perceive and dodge,
      // 3) leave the road only after the hazard has been visible for a while.
      if (d.crossingPhase === 'ENTER') {
        d.crossingProgress = Math.min(1, d.crossingProgress + dt / Math.max(d.crossingEntryDuration || CROSSING_ENTRY_DURATION, 0.1));
        const p = d.crossingProgress;
        const smooth = p * p * (3 - 2 * p);
        d.currentLateral = THREE.MathUtils.lerp(d.crossingStartLateral, d.crossingTargetLateral, smooth);

        if (p >= 1) {
          d.crossingPhase = 'HOLD';
          d.crossingProgress = 0;
          d.currentLateral = d.crossingTargetLateral;
        }
      } else if (d.crossingPhase === 'HOLD') {
        // Stay in the selected lane. Very small longitudinal movement makes the
        // actor look alive without letting it run through the scene.
        d.currentLateral = d.crossingTargetLateral;
        d.crossingProgress += dt;
        d.speed = 0.12;

        if (d.crossingProgress >= (d.crossingHoldDuration || CROSSING_HOLD_DURATION)) {
          d.crossingPhase = 'EXIT';
          d.crossingProgress = 0;
        }
      } else if (d.crossingPhase === 'EXIT') {
        d.crossingProgress = Math.min(1, d.crossingProgress + dt / Math.max(d.crossingExitDuration || CROSSING_EXIT_DURATION, 0.1));
        const p = d.crossingProgress;
        const smooth = p * p * (3 - 2 * p);
        d.currentLateral = THREE.MathUtils.lerp(d.crossingTargetLateral, d.crossingEndLateral, smooth);

        if (p >= 1) {
          d.crossingActive = false;
          d.crossingPhase = null;
          d.crossingProgress = 0;
          d.currentLateral = d.crossingEndLateral;
          d.speed = d.type === 'dog' ? 0.45 + (d.spawnIndex % 2) * 0.18 : 0.7 + (d.spawnIndex % 3) * 0.15;
        }
      }
    } else {
      d.currentLateral = d.side * (ROAD_WIDTH / 2 + (d.type === 'dog' ? 2.3 : 3.0));
    }

    // Recycle normal roadside actors after they pass the ego. Crossing actors are
    // allowed to finish the full road-crossing animation before recycling.
    if (sim && !d.crossingActive) {
      const gapToEgo = signedTrackGap(d.s, sim.egoS);
      if (gapToEgo < -28) {
        const respawnGap = 95 + (d.spawnIndex % 5) * 28;
        d.s = (sim.egoS + respawnGap) % TRACK_LENGTH;
        d.side = d.spawnIndex % 2 === 0 ? -1 : 1;
        d.lastSpawnS = d.s;
      }
    }

    const lateral = Number.isFinite(d.currentLateral)
      ? d.currentLateral
      : d.side * (ROAD_WIDTH / 2 + (d.type === 'dog' ? 2.3 : 3.0));
    const pos = roadPoint(d.s, lateral, d.type === 'dog' ? 0 : 0.02);
    actor.position.copy(pos);
    actor.rotation.y = roadHeading(d.s) + (d.side > 0 ? Math.PI : 0);
    actor.position.y += Math.sin(performance.now() * 0.006 + d.phase) * (d.type === 'dog' ? 0.015 : 0.025);
  }
}
export default function App() {
  const mountRef = useRef(null);

  const simRef = useRef({
    running: true,
    ready: false,
    egoS: 70,
    egoLane: 1,
    targetLane: 1,
    laneChangeCooldown: 0,
    overtakingId: null,
    planLane: null,
    planTimer: 0,
    planTargetId: null,
    waitLane: null,
    waitBlockerId: null,
    vehicles: [],
    templates: {},
    last: 0,
    collisionHold: 0,
    radarZone: 'NONE',
    radarLeadId: null,
    radarGap: Infinity,
    radarRelativeSpeed: 0,
    radarTtc: Infinity,
    predictedOvertakeLane: null,
    controllerMode: 'CRUISE',
    waitingForTargetClear: false,
    mergeCooldown: 0,
    overtakeStartS: 70,
    overtakeFromLane: 1,
    uiPhase: 'CRUISING',
    uiTarget: 'CLEAR',
    aiAccumulator: 0,
    safetyAccumulator: 0,
    lastErrorAt: 0,
    visibleBudget: 12,
    scriptedMode: SCRIPTED_SIMULATION,
    scriptTime: 0,
    scriptPhaseIndex: -1,
    scriptScenario: null,
    potholeAvoidanceActive: false,
    potholeId: null,
    potholeTargetLane: null,
    potholeReturnLane: 1,
    potholePreviousMode: 'CRUISE',
    potholeDetectedGap: Infinity,
    roadsideScenarioTime: 0,
    roadsideScenarioIndex: 0,
    roadsideAvoidanceActive: false,
    roadsideHazardActor: null,
    roadsideHazardType: null,
    roadsideHazardGap: Infinity,
    roadsideTargetLane: null,
    roadsideReturnLane: 1,
    lastDashboardPaint: 0
  });

  const [phase, setPhase] = useState('CRUISING');
  const [target, setTarget] = useState('CLEAR');
  const [ready, setReady] = useState(false);
  const [dashboardTick, setDashboardTick] = useState(0);
  const [activeNav, setActiveNav] = useState('SIMULATION');

  const handleNavClick = (section) => {
    setActiveNav(section);
  };

  useEffect(() => {
    let cleanup = null;

    const initialize = async () => {
      const mount = mountRef.current;
      if (!mount) return;

      mount.style.background =
        'linear-gradient(to bottom, #07152f 0%, #193b63 30%, #557b91 62%, #d29a78 84%, #f0c58f 100%)';

      const renderer = new THREE.WebGLRenderer({
        antialias: false,
        alpha: true,
        powerPreference: 'high-performance'
      });

      renderer.setPixelRatio(1);
      renderer.setSize(mount.clientWidth, mount.clientHeight);
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      mount.appendChild(renderer.domElement);

      const scene = new THREE.Scene();

      // Sunset sky: deep violet overhead fading through warm orange/yellow near the horizon.
      const skyCanvas = document.createElement('canvas');
      skyCanvas.width = 2;
      skyCanvas.height = 512;
      const skyCtx = skyCanvas.getContext('2d');
      const skyGradient = skyCtx.createLinearGradient(0, 0, 0, skyCanvas.height);
      skyGradient.addColorStop(0.00, '#24164f');
      skyGradient.addColorStop(0.24, '#55306f');
      skyGradient.addColorStop(0.48, '#9a4f76');
      skyGradient.addColorStop(0.68, '#e17b55');
      skyGradient.addColorStop(0.84, '#f5a84f');
      skyGradient.addColorStop(1.00, '#ffe08a');
      skyCtx.fillStyle = skyGradient;
      skyCtx.fillRect(0, 0, skyCanvas.width, skyCanvas.height);
      const skyTexture = new THREE.CanvasTexture(skyCanvas);
      skyTexture.colorSpace = THREE.SRGBColorSpace;
      skyTexture.needsUpdate = true;
      scene.background = skyTexture;
      scene.fog = new THREE.Fog(0xd28a78, 180, 650);

      // Soft layered sunset clouds. They stay high above the road and do not affect traffic.
      const cloudGroup = new THREE.Group();
      cloudGroup.name = 'SunsetClouds';
      const cloudColors = [0x6b4b86, 0x8b587b, 0xc16f70, 0xe18a6d];
      const cloudSets = [
        { x: -42, y: 48, z: -120, scale: 1.8, color: 0x6b4b86 },
        { x: 34, y: 55, z: -170, scale: 2.2, color: 0x553d79 },
        { x: 4, y: 43, z: -230, scale: 2.8, color: 0x9a5f78 },
        { x: -65, y: 57, z: -260, scale: 1.5, color: 0x7b527e },
        { x: 62, y: 45, z: -300, scale: 1.9, color: 0xc17373 }
      ];
      for (const cloud of cloudSets) {
        const group = new THREE.Group();
        const mat = new THREE.MeshBasicMaterial({
          color: cloud.color,
          transparent: true,
          opacity: 0.52,
          depthWrite: false,
          fog: false
        });
        const blobs = [
          [-2.4, 0, 0, 1.5], [-1.0, 0.35, 0, 1.8], [0.5, 0.15, 0, 2.0],
          [1.8, -0.05, 0, 1.45], [3.0, -0.12, 0, 1.0]
        ];
        for (const [bx, by, bz, bs] of blobs) {
          const mesh = new THREE.Mesh(new THREE.SphereGeometry(bs, 12, 8), mat);
          mesh.scale.set(1.7, 0.55, 0.55);
          mesh.position.set(bx, by, bz);
          group.add(mesh);
        }
        group.position.set(cloud.x, cloud.y, cloud.z);
        group.scale.setScalar(cloud.scale);
        cloudGroup.add(group);
      }
      scene.add(cloudGroup);

      const camera = new THREE.PerspectiveCamera(
        54,
        mount.clientWidth / Math.max(mount.clientHeight, 1),
        0.05,
        1000
      );

      camera.position.set(0, 8, 14);

      scene.add(
        new THREE.HemisphereLight(
          0xffe3c7,
          0x243223,
          2.3
        )
      );

      const sun = new THREE.DirectionalLight(
        0xffc58e,
        1.25
      );
      sun.position.set(-40, 55, -80);
      scene.add(sun);

      createRoad(scene);
      const potholeGroup = createPotholes(scene);
      const roadsideLife = createRoadsideLife(scene);

      // Real main/ego car: McLaren W1.
      const egoCar = new THREE.Group();
      scene.add(egoCar);

      // Camera is physically mounted to the ego vehicle so its movement is
      // guaranteed to follow the McLaren rather than relying on a fixed world
      // coordinate or a slowly interpolated fallback.
      const cameraMount = new THREE.Object3D();
      cameraMount.position.set(0, 4.6, 10.5);
      egoCar.add(cameraMount);
      cameraMount.add(camera);
      camera.position.set(0, 0, 0);

      const radar = createRadar();
      const vehicleLinks = createEgoVehicleLinks(scene);
      const detectionBoxes = createDetectionBoxes(scene, mount);
      const pathRibbon = createPathRibbon(scene);
      egoCar.add(radar);

      // Visible red identifier under/around the ego car.
      const egoGlow = new THREE.Mesh(
        new THREE.RingGeometry(1.6, 2.0, 48),
        new THREE.MeshBasicMaterial({
          color: 0xff3d4f,
          transparent: true,
          opacity: 0.65,
          side: THREE.DoubleSide,
          depthWrite: false
        })
      );

      egoGlow.rotation.x = -Math.PI / 2;
      egoGlow.position.y = 0.08;
      egoCar.add(egoGlow);
      addSafetyZone(egoCar, MAIN_CAR.length, EGO_WIDTH, 0x31e6ff, true);

      scene.add(egoCar);

      // Put a visible McLaren fallback in the scene immediately. The real GLB
      // replaces it automatically when/if the asset finishes loading.
      const egoFallback = createVehicleFallback({
        label: 'McLaren W1',
        targetLength: MAIN_CAR.length,
        color: '#f36f21'
      }, true);
      egoCar.add(egoFallback);

      loadModel(
        MAIN_CAR.path,
        'gltf',
        MAIN_CAR.length,
        MAIN_CAR.rotateY,
        true
      ).then(model => {
        // Keep the visible proxy until the real model has valid geometry.
        const checkBox = new THREE.Box3().setFromObject(model);
        const size = new THREE.Vector3();
        checkBox.getSize(size);
        if (!Number.isFinite(size.x) || !Number.isFinite(size.y) || !Number.isFinite(size.z) || Math.max(size.x, size.y, size.z) < 0.2) {
          throw new Error('Loaded McLaren model has invalid/empty bounds');
        }
        egoCar.remove(egoFallback);
        egoFallback.traverse(n => {
          if (n.geometry) n.geometry.dispose();
          if (n.material) {
            const mats = Array.isArray(n.material) ? n.material : [n.material];
            mats.forEach(m => m.dispose?.());
          }
        });
        egoCar.add(model);
        egoCar.userData.realModelLoaded = true;
      }).catch(error => {
        console.warn('[NAVION] Real McLaren GLB did not load; keeping visible fallback.', error);
        egoCar.userData.realModelLoaded = false;
      });

      // LOAD REAL TRAFFIC MODELS FIRST, BUT IN PARALLEL GROUPS.
      // This avoids the previous problem where one slow/broken asset blocked every
      // later vehicle type from ever appearing.
      const vehicles = [];
      const priority = ['hilux', 'bmw', 'ford', 'tataTruck', 'dodgeChallenger', 'gt3', 'ferrariF40', 'ferrariF40B', 'bike'];
      const loadStatus = {};

      const loadTrafficKey = async key => {
        // McLaren W1 is reserved exclusively for the ego vehicle. It is never loaded
        // into the normal traffic pool.
        if (key === 'mclaren') return null;
        const def = TRAFFIC_DEFS[key];
        loadStatus[key] = 'loading';
        try {
          const template = await loadModel(
            def.paths,
            def.loader,
            def.targetLength,
            def.rotateY,
            false
          );
          simRef.current.templates[key] = template;
          loadStatus[key] = 'loaded';
          return key;
        } catch (error) {
          loadStatus[key] = 'failed';
          console.warn(`[NAVION] ${def.label} skipped:`, error);
          return null;
        }
      };

      // Bootstrap traffic from the FIRST SUCCESSFUL REAL GLB instead of waiting
      // for every priority asset. The old version could sit on an empty highway
      // for many seconds when one model path timed out.
      const firstBatch = SCRIPTED_SIMULATION ? ['hilux', 'ford', 'tataTruck', 'dodgeChallenger'] : ['hilux', 'bmw', 'ford', 'tataTruck', 'dodgeChallenger'];
      const secondBatch = ['dodgeChallenger', 'gt3', 'ferrariF40', 'ferrariF40B', 'bike'];
      const firstPromises = firstBatch.map(loadTrafficKey);
      const secondPromises = secondBatch.map(loadTrafficKey);

      const waitForFirstRealModel = async () => {
        let remaining = firstPromises.length;
        return new Promise(resolve => {
          let resolved = false;
          firstPromises.forEach(p => {
            p.then(key => {
              remaining -= 1;
              if (!resolved && key) {
                resolved = true;
                resolve(key);
              } else if (!resolved && remaining === 0) {
                resolve(null);
              }
            }).catch(() => {
              remaining -= 1;
              if (!resolved && remaining === 0) resolve(null);
            });
          });
        });
      };

      // IMPORTANT: do not await model loading here. The simulation must start
      // immediately with visible traffic proxies; the real GLBs are swapped in
      // asynchronously as soon as each one finishes loading.
      void Promise.race([
        waitForFirstRealModel(),
        new Promise(resolve => setTimeout(() => resolve(null), 4500))
      ]);

      let loadedKeys = firstBatch.filter(k => loadStatus[k] === 'loaded');
      if (!loadedKeys.length) {
        loadedKeys = [...firstBatch];
      }
      loadedKeys = [...new Set(loadedKeys)];

      // Continue waiting for additional practical models in the background. As
      // each one arrives, it is injected into the live highway instead of leaving
      // the initial traffic as one repeated model forever.
      const replaceFallbackTraffic = key => {
        const sim = simRef.current;
        const template = sim.templates[key];
        if (!template) return;

        for (const vehicle of sim.vehicles) {
          if (vehicle.scriptInactive) continue;
          if (vehicle.key !== key || !vehicle.root) continue;
          if (!vehicle.root.userData.isFallbackVehicle) continue;

          const oldModel = vehicle.root.userData.vehicleModel;
          if (oldModel) {
            vehicle.root.remove(oldModel);
            oldModel.traverse(n => {
              if (n.geometry) n.geometry.dispose?.();
              if (n.material) {
                const mats = Array.isArray(n.material) ? n.material : [n.material];
                mats.forEach(m => m.dispose?.());
              }
            });
          }

          const realModel = cloneSkeleton(template);
          vehicle.root.add(realModel);
          vehicle.root.userData.vehicleModel = realModel;
          vehicle.root.userData.isFallbackVehicle = false;
          vehicle.root.visible = true;
        }
      };

      const spawnedExotics = new Set(SCRIPTED_SIMULATION ? Object.keys(EXOTIC_TARGETS) : []);
      const spawnLoadedVehicle = (key, count = 1) => {
        const sim = simRef.current;
        const template = sim.templates[key];
        const def = TRAFFIC_DEFS[key];
        if (!template || !def || vehicles.length >= TRAFFIC_COUNT) return;

        const existingKeyCount = () => vehicles.reduce((n, v) => n + (v.key === key ? 1 : 0), 0);
        const keyCap = def.heavy ? MAX_TATA_TRUCKS : (def.sports ? (EXOTIC_TARGETS[key] || 2) : 99);
        const allowed = Math.max(0, keyCap - existingKeyCount());

        for (let n = 0; n < Math.min(count, allowed) && vehicles.length < TRAFFIC_COUNT; n++) {
          const laneChoices = [0, 1, 2, 3].sort(() => Math.random() - 0.5);
          let lane = laneChoices[0];
          let spawnS = (sim.egoS + 180 + Math.random() * 360) % TRACK_LENGTH;
          let found = false;
          for (const candidateLane of laneChoices) {
            for (let attempt = 0; attempt < 10; attempt++) {
              const candidateS = (sim.egoS + 180 + Math.random() * 360 + attempt * 12) % TRACK_LENGTH;
              const gaps = laneClearForTraffic(sim, candidateLane, candidateS, null);
              if (gaps.frontGap > 28 && gaps.backGap > 24 &&
                !wouldCreateEgoSideBySide(sim, candidateLane, candidateS, null, 65) &&
                !trafficConflictsWithEgoCorridor(sim, null, candidateLane, candidateS)) {
                lane = candidateLane;
                spawnS = candidateS;
                found = true;
                break;
              }
            }
            if (found) break;
          }

          const root = new THREE.Group();
          const vehicleModel = cloneSkeleton(template);
          root.add(vehicleModel);
          root.userData.vehicleModel = vehicleModel;
          root.userData.isFallbackVehicle = false;
          addTrafficIndicatorLights(root, 0xffb02e);
          addSafetyZone(root, def.targetLength, def.targetWidth, 0x7f66ff, false);
          scene.add(root);

          let speed = def.speedMin + Math.random() * (def.speedMax - def.speedMin);
          const driverRoll = Math.random();
          const driverType = driverRoll < 0.12 ? 'reckless' : driverRoll < 0.42 ? 'assertive' : 'normal';
          if (driverType === 'reckless') speed *= 1.10 + Math.random() * 0.12;
          if (driverType === 'assertive') speed *= 1.03 + Math.random() * 0.05;

          vehicles.push({
            id: Math.max(0, ...vehicles.map(v => v.id)) + 1,
            key,
            lane,
            originalLane: lane,
            targetLane: undefined,
            laneChangeTimer: 0,
            laneChangeDuration: 1.35,
            laneChangeCooldown: 2.0 + Math.random() * 2.0,
            indicator: 0,
            indicatorTimer: 0,
            speed,
            cruiseSpeed: speed,
            driverType,
            overtakeTargetId: null,
            s: spawnS,
            collisionRadius: Math.max(2.8, vehicleModel.userData.collisionRadius || physicalCollisionDistance(def, 0.0) * 0.56),
            root
          });
        }
      };

      const lateModelTimer = setInterval(() => {
        ['hilux', 'bmw', 'ford', 'tataTruck', 'dodgeChallenger', 'gt3', 'ferrariF40', 'ferrariF40B', 'bike'].forEach(key => {
          if (loadStatus[key] === 'loaded') {
            replaceFallbackTraffic(key);
            if (!SCRIPTED_SIMULATION && TRAFFIC_DEFS[key].sports && !spawnedExotics.has(key)) {
              const amount = EXOTIC_TARGETS[key] || 1;
              spawnLoadedVehicle(key, amount);
              spawnedExotics.add(key);
            }
          }
        });
      }, 500);
      setTimeout(() => clearInterval(lateModelTimer), 60000);

      Promise.allSettled(firstPromises.concat(secondPromises)).then(() => {
        ['hilux', 'bmw', 'ford', 'tataTruck', 'dodgeChallenger', 'gt3', 'ferrariF40', 'ferrariF40B', 'bike'].forEach(key => {
          if (loadStatus[key] === 'loaded') {
            replaceFallbackTraffic(key);
            if (!SCRIPTED_SIMULATION && TRAFFIC_DEFS[key].sports && !spawnedExotics.has(key)) {
              spawnLoadedVehicle(key, EXOTIC_TARGETS[key] || 1);
              spawnedExotics.add(key);
            }
          }
        });
      });

      // Use the full weighted pool so exotic cars can actually appear.
      // Truck density is controlled by the Tata spawnWeight rather than by excluding
      // the sports/exotic models entirely.
      const spawnPool = loadedKeys;

      const lanePattern = [0, 2, 3, 1, 0, 3, 2, 1];
      const baseS = 145;
      const slotSpacing = 54;

      const isSpawnClear = (candidateLane, candidateS, minGap) => {
        const egoGap = Math.abs(signedTrackGap(candidateS, simRef.current.egoS));
        if (egoGap < 135) return false;
        for (const other of vehicles) {
          if (Math.abs(other.lane - candidateLane) > 0.45) continue;
          if (Math.abs(signedTrackGap(candidateS, other.s)) < minGap) return false;
        }
        return true;
      };

      const pickInitialSpawnKey = () => {
        const truckCount = vehicles.filter(v => v.key === 'tataTruck').length;
        const pool = truckCount >= MAX_TATA_TRUCKS
          ? spawnPool.filter(k => k !== 'tataTruck')
          : spawnPool;
        return pickWeightedTrafficKey(pool.length ? pool : spawnPool);
      };

      for (let i = 0; i < INITIAL_TRAFFIC_COUNT; i++) {
        const key = pickInitialSpawnKey();
        const def = TRAFFIC_DEFS[key];
        const template = simRef.current.templates[key];

        const root = new THREE.Group();
        const vehicleModel = template
          ? cloneSkeleton(template)
          : createVehicleFallback(def, false);
        root.add(vehicleModel);
        root.userData.vehicleModel = vehicleModel;
        root.userData.isFallbackVehicle = !template;
        // Never show the crude box fallback. Keep the traffic slot hidden until its real GLB loads.
        root.visible = Boolean(template);
        addTrafficIndicatorLights(root, 0xffb02e);
        addSafetyZone(root, def.targetLength, def.targetWidth, def.heavy ? 0xffb45c : 0x4bc7ff, false);
        scene.add(root);

        let lane = lanePattern[i % lanePattern.length];
        let spawnS = baseS + i * slotSpacing + Math.random() * 12;
        const minGap = def.heavy ? 38 : 26;

        for (let attempt = 0; attempt < 20 && (!isSpawnClear(lane, spawnS, minGap) || trafficConflictsWithEgoCorridor(simRef.current, null, lane, spawnS)); attempt++) {
          lane = (lane + 1) % 4;
          spawnS += 8 + attempt * 1.5;
        }

        let trafficSpeed = def.speedMin + Math.random() * (def.speedMax - def.speedMin);
        const driverRoll = Math.random();
        const driverType = driverRoll < 0.09
          ? 'reckless'
          : driverRoll < 0.32
            ? 'assertive'
            : 'normal';

        if (driverType === 'reckless' && !def.heavy) {
          trafficSpeed *= 1.12 + Math.random() * 0.12;
        } else if (driverType === 'assertive') {
          trafficSpeed *= 1.03 + Math.random() * 0.06;
        }

        if (def.sports) trafficSpeed = Math.min(trafficSpeed, 90);

        vehicles.push({
          id: i + 1,
          key,
          lane,
          originalLane: lane,
          targetLane: undefined,
          laneChangeTimer: 0,
          laneChangeDuration: 1.45,
          laneChangeCooldown: 1.8 + Math.random() * 2.5,
          indicator: 0,
          indicatorTimer: 0,
          speed: trafficSpeed,
          cruiseSpeed: trafficSpeed,
          driverType,
          overtakeTargetId: null,
          s: spawnS,
          collisionRadius: Math.max(2.8, vehicleModel.userData.collisionRadius || physicalCollisionDistance(def, 0.0) * 0.56),
          root
        });
      }

      // Guarantee a realistic overtaking opportunity near the ego vehicle so the
      // adaptive planner demonstrates DETECT -> THINK -> LANE CHANGE early rather
      // than spending the first long stretch of the demo with an empty lane.
      const practical = vehicles.filter(v => !TRAFFIC_DEFS[v.key].sports);
      const egoLaneInit = Math.round(simRef.current.egoLane);
      const demoLead = practical.find(v => v.lane === egoLaneInit);
      if (demoLead) {
        demoLead.s = (simRef.current.egoS + 62) % TRACK_LENGTH;
        demoLead.speed = TRAFFIC_DEFS[demoLead.key].heavy ? 38 : 44;
        demoLead.cruiseSpeed = demoLead.speed;

        // Keep one adjacent lane clearly open for the first 3-layer demonstration.
        const preferredEscape = egoLaneInit > 0 ? egoLaneInit - 1 : egoLaneInit + 1;
        vehicles.forEach(v => {
          if (v.id === demoLead.id) return;
          if (Math.abs(v.lane - preferredEscape) > 0.35) return;
          if (Math.abs(signedTrackGap(v.s, simRef.current.egoS)) < 70) {
            v.s = (simRef.current.egoS + 260 + Math.random() * 120) % TRACK_LENGTH;
          }
        });
      } else if (practical.length) {
        const v = practical[0];
        v.lane = egoLaneInit;
        v.originalLane = v.lane;
        v.targetLane = undefined;
        v.s = (simRef.current.egoS + 58) % TRACK_LENGTH;
        v.speed = TRAFFIC_DEFS[v.key].heavy ? 40 : 46;
        v.cruiseSpeed = v.speed;
      }

      // Create a deterministic multi-overtake demonstration: several slower
      // practical vehicles are staggered through different lanes. This guarantees
      // the Dodge has more than one real overtaking opportunity during a demo.
      const egoLaneDemo = Math.round(simRef.current.egoLane);
      const chainLeads = [
        { lane: egoLaneDemo, gap: 62, speed: 42, truck: false },
        { lane: egoLaneDemo > 0 ? egoLaneDemo - 1 : egoLaneDemo + 1, gap: 138, speed: 74, truck: false },
        { lane: egoLaneDemo < 3 ? egoLaneDemo + 1 : egoLaneDemo - 1, gap: 228, speed: 88, truck: false },
        { lane: 3 - egoLaneDemo, gap: 330, speed: 78, truck: false },
        { lane: egoLaneDemo, gap: 455, speed: 50, truck: false },
        { lane: egoLaneDemo < 3 ? egoLaneDemo + 1 : egoLaneDemo - 1, gap: 575, speed: 62, truck: false }
      ];

      chainLeads.forEach((cfg, idx) => {
        const desiredKey = cfg.speed >= 70
          ? (loadedKeys.find(k => k === 'gt3') || loadedKeys.find(k => k === 'ferrariF40') || loadedKeys.find(k => k === 'ferrariF40B') || loadedKeys.find(k => TRAFFIC_DEFS[k].sports) || loadedKeys.find(k => !TRAFFIC_DEFS[k].heavy) || loadedKeys[0])
          : (loadedKeys.find(k => !TRAFFIC_DEFS[k].sports && k !== 'tataTruck') || loadedKeys.find(k => TRAFFIC_DEFS[k].sports) || loadedKeys[0]);
        const template = simRef.current.templates[desiredKey];
        const def = TRAFFIC_DEFS[desiredKey];
        const existing = vehicles.find(v => v.lane === cfg.lane && Math.abs(signedTrackGap(v.s, simRef.current.egoS) - cfg.gap) < 18);
        if (existing) {
          existing.s = (simRef.current.egoS + cfg.gap) % TRACK_LENGTH;
          existing.lane = cfg.lane;
          existing.originalLane = cfg.lane;
          existing.speed = Math.min(existing.speed, cfg.speed);
          existing.cruiseSpeed = existing.speed;
          return;
        }

        if (vehicles.length >= TRAFFIC_COUNT || !template) return;

        const root = new THREE.Group();
        const vehicleModel = template
          ? cloneSkeleton(template)
          : createVehicleFallback(def, false);
        root.add(vehicleModel);
        root.userData.vehicleModel = vehicleModel;
        root.userData.isFallbackVehicle = !template;
        // Never show the crude box fallback. Keep the traffic slot hidden until its real GLB loads.
        root.visible = Boolean(template);
        addTrafficIndicatorLights(root, 0xffb02e);
        addSafetyZone(root, def.targetLength, def.targetWidth, def.heavy ? 0xffb45c : 0x4bc7ff, false);
        scene.add(root);
        vehicles.push({
          id: Math.max(0, ...vehicles.map(v => v.id)) + 1,
          key: desiredKey,
          lane: cfg.lane,
          originalLane: cfg.lane,
          targetLane: undefined,
          laneChangeTimer: 0,
          laneChangeDuration: 1.45,
          laneChangeCooldown: 3.0,
          indicator: 0,
          indicatorTimer: 0,
          speed: cfg.speed,
          cruiseSpeed: cfg.speed,
          driverType: 'normal',
          overtakeTargetId: null,
          s: (simRef.current.egoS + cfg.gap) % TRACK_LENGTH,
          collisionRadius: Math.max(2.8, vehicleModel.userData.collisionRadius || physicalCollisionDistance(def, 0.0) * 0.56),
          root
        });
      });

      // If only one practical model finished quickly, keep the road visibly populated
      // instead of waiting for every asset. All of these are still REAL GLB clones.
      if (!SCRIPTED_SIMULATION && vehicles.length > 0 && vehicles.length < TRAFFIC_COUNT) {
        const seedVehicles = vehicles.slice();
        let nextId = vehicles.length + 1;
        while (vehicles.length < TRAFFIC_COUNT) {
          const seed = seedVehicles[(vehicles.length - seedVehicles.length) % seedVehicles.length];
          const key = seed.key;
          const def = TRAFFIC_DEFS[key];
          const template = simRef.current.templates[key];
          if (!template) break;

          const root = new THREE.Group();
          const vehicleModel = cloneSkeleton(template);
          root.add(vehicleModel);
          root.userData.vehicleModel = vehicleModel;
          addTrafficIndicatorLights(root, 0xffb02e);
          addSafetyZone(root, def.targetLength, def.targetWidth, def.heavy ? 0xffb45c : 0x4bc7ff, false);
          scene.add(root);

          const lane = vehicles.length % 4;
          const spawnS = (simRef.current.egoS + 120 + (vehicles.length - seedVehicles.length) * 52) % TRACK_LENGTH;
          const cloneSpeed = def.speedMin + Math.random() * (def.speedMax - def.speedMin);
          vehicles.push({
            id: nextId++,
            key,
            lane,
            originalLane: lane,
            targetLane: undefined,
            laneChangeTimer: 0,
            laneChangeDuration: 1.45,
            laneChangeCooldown: 2.2,
            indicator: 0,
            indicatorTimer: 0,
            speed: cloneSpeed,
            cruiseSpeed: cloneSpeed,
            driverType: 'normal',
            overtakeTargetId: null,
            s: spawnS,
            collisionRadius: Math.max(2.8, vehicleModel.userData.collisionRadius || physicalCollisionDistance(def, 0.0) * 0.56),
            root
          });
        }
      }

      simRef.current.vehicles = vehicles;

      // Fixed scenario setup: keep a small, repeatable cast of traffic vehicles.
      // The vehicles themselves still move naturally; only the starting positions,
      // speeds and scripted lane events are predetermined.
      if (SCRIPTED_SIMULATION) {
        const scriptedVehicles = simRef.current.vehicles.slice(0, 9);
        simRef.current.vehicles.slice(6).forEach(v => { v.root.visible = false; v.scriptInactive = true; });
        scriptedVehicles.forEach((v, i) => {
          v.scriptInactive = false;
          v.targetLane = undefined;
          v.indicator = 0;
          v.indicatorTimer = 0;
          v.laneChangeCooldown = 999;
          v.driverType = 'normal';
          v.speed = 95 + i * 4;
          v.cruiseSpeed = v.speed;
          v.root.visible = true;
        });
      }

      simRef.current.ready = true;
      setReady(true);

      let frame = 0;
      let pathUpdateFrame = 0;
      let radarUpdateFrame = 0;
      let vehicleLinkFrame = 0;

      // Avoid React state updates on every animation frame. The previous version
      // could call setPhase/setTarget 30–60 times per second, causing visible
      // stutter and making the controller feel less responsive.
      const updateUI = (nextPhase, nextTarget) => {
        const sim = simRef.current;
        if (nextPhase !== undefined && nextPhase !== sim.uiPhase) {
          sim.uiPhase = nextPhase;
          setPhase(nextPhase);
        }
        if (nextTarget !== undefined && nextTarget !== sim.uiTarget) {
          sim.uiTarget = nextTarget;
          setTarget(nextTarget);
        }
      };

      const configureScriptPhase = (phaseIndex) => {
        const sim = simRef.current;
        const phase = SCRIPT_PHASES[phaseIndex % SCRIPT_PHASES.length];
        const active = sim.vehicles.filter(v => !v.scriptInactive).slice(0, 6);
        const lead = active[0];
        if (!lead) return;

        // Reset the scripted cast relative to the moving Dodge. No per-frame teleporting:
        // this happens only at the beginning of a new demonstration scenario.
        lead.lane = phase.leadLane;
        lead.originalLane = phase.leadLane;
        lead.targetLane = undefined;
        lead.s = (sim.egoS + (phase.leadGap ?? 62)) % TRACK_LENGTH;
        lead.speed = phase.leadSpeed;
        lead.cruiseSpeed = phase.leadSpeed;
        lead.indicator = 0;
        lead.indicatorTimer = 0;

        const placements = [
          { lane: 0, gap: 170, speed: 108 },
          { lane: 2, gap: 190, speed: 112 },
          { lane: 3, gap: 265, speed: 118 },
          { lane: 0, gap: 335, speed: 102 },
          { lane: 2, gap: 405, speed: 120 }
        ];
        active.slice(1).forEach((v, i) => {
          const cfg = placements[i];
          v.lane = cfg.lane;
          v.originalLane = cfg.lane;
          v.targetLane = undefined;
          v.s = (sim.egoS + cfg.gap) % TRACK_LENGTH;
          v.speed = cfg.speed;
          v.cruiseSpeed = cfg.speed;
          v.indicator = 0;
          v.indicatorTimer = 0;
          v.overtakeTargetId = null;
        });

        // For the two-lane scenario, keep the intermediate lane and destination lane
        // deliberately clear. For the single-lane scenarios, the chosen side is clear.
        if (phaseIndex === 0) {
          const left = active[1];
          if (left) left.s = (sim.egoS + 320) % TRACK_LENGTH;
        } else if (phaseIndex === 1) {
          const right = active[2];
          if (right) right.s = (sim.egoS + 320) % TRACK_LENGTH;
        } else if (phaseIndex === 2) {
          const middle = active[2];
          const far = active[3];
          if (middle) { middle.lane = 2; middle.s = (sim.egoS + 360) % TRACK_LENGTH; }
          if (far) { far.lane = 3; far.s = (sim.egoS + 430) % TRACK_LENGTH; }
        } else if (phaseIndex === 3) {
          // Keep adjacent lanes clear while the lead vehicle performs the yield.
          active.slice(1, 3).forEach(v => { v.s = (sim.egoS + 320 + active.indexOf(v) * 55) % TRACK_LENGTH; });
        }

        sim.targetLane = 1;
        sim.egoLane = 1;
        sim.controllerMode = 'SCRIPTED';
        sim.overtakingId = lead.id;
        sim.waitingForTargetClear = false;
        sim.scriptScenario = phase;
        updateUI(phase.name, phaseIndex === 3 ? 'LEAD VEHICLE WILL YIELD' : 'RADAR PATH CLEAR');
      };

      const restartSimulation = () => {
        const sim = simRef.current;
        sim.egoS = 70;
        sim.egoLane = 1;
        sim.targetLane = 1;
        sim.laneChangeCooldown = 0;
        sim.overtakingId = null;
        sim.planLane = null;
        sim.planTimer = 0;
        sim.planTargetId = null;
        sim.waitLane = null;
        sim.waitBlockerId = null;
        sim.collisionHold = 0;
        sim.radarZone = 'NONE';
        sim.radarLeadId = null;
        sim.radarGap = Infinity;
        sim.radarRelativeSpeed = 0;
        sim.radarTtc = Infinity;
        sim.predictedOvertakeLane = null;
        sim.controllerMode = 'SCRIPTED';
        sim.waitingForTargetClear = false;
        sim.mergeCooldown = 0;
        sim.overtakeStartS = 70;
        sim.overtakeFromLane = 1;
        sim.uiPhase = 'CRUISING';
        sim.uiTarget = 'CLEAR';
        sim.potholeAvoidanceActive = false;
        sim.potholeId = null;
        sim.potholeTargetLane = null;
        sim.potholeReturnLane = 1;
        sim.potholePreviousMode = 'CRUISE';
        sim.potholeDetectedGap = Infinity;
        sim.roadsideScenarioTime = 0;
        sim.roadsideScenarioIndex = 0;
        sim.roadsideAvoidanceActive = false;
        sim.roadsideHazardActor = null;
        sim.roadsideHazardType = null;
        sim.roadsideHazardGap = Infinity;
        sim.roadsideTargetLane = null;
        sim.roadsideReturnLane = 1;
        sim.scriptTime = 0;
        sim.scriptPhaseIndex = -1;
        sim.scriptScenario = null;

        const active = sim.vehicles.filter(v => !v.scriptInactive).slice(0, 6);
        const restartPlacements = [
          { lane: 1, gap: 62, speed: 42 },
          { lane: 0, gap: 170, speed: 108 },
          { lane: 2, gap: 190, speed: 112 },
          { lane: 3, gap: 265, speed: 118 },
          { lane: 0, gap: 335, speed: 102 },
          { lane: 2, gap: 405, speed: 120 }
        ];
        active.forEach((v, i) => {
          const cfg = restartPlacements[i] || restartPlacements[restartPlacements.length - 1];
          v.lane = cfg.lane;
          v.originalLane = cfg.lane;
          v.targetLane = undefined;
          v.s = (sim.egoS + cfg.gap) % TRACK_LENGTH;
          v.speed = cfg.speed;
          v.cruiseSpeed = cfg.speed;
          v.indicator = 0;
          v.indicatorTimer = 0;
          v.laneChangeCooldown = 999;
          v.overtakeTargetId = null;
          v.root.visible = true;
        });

        sim.vehicles.slice(6).forEach(v => { v.root.visible = false; v.scriptInactive = true; });

        const life = roadsideLife?.userData?.life || [];
        life.forEach((actor, i) => {
          const d = actor.userData;
          d.crossingActive = false;
          d.crossingPhase = null;
          d.crossingProgress = 0;
          d.currentLateral = d.side * (ROAD_WIDTH / 2 + (d.type === 'dog' ? 2.3 : 3.0));
          d.s = (sim.egoS + 55 + i * 120) % TRACK_LENGTH;
          d.lastSpawnS = d.s;
          actor.visible = true;
        });

        configureScriptPhase(0);
        sim.scriptTime = 0;
        sim.scriptPhaseIndex = 0;
        updateUI('SIMULATION RESTARTED', 'RESET TO START');
      };

      const runScriptedTick = (dt) => {
        const sim = simRef.current;
        const nextEgoS = sim.egoS + EGO_SPEED_MS * dt;
        if (nextEgoS >= TRACK_LENGTH) {
          restartSimulation();
          return;
        }
        sim.scriptTime += dt;
        const phaseIndex = Math.floor(sim.scriptTime / SCRIPT_PHASE_DURATION) % SCRIPT_PHASES.length;
        const local = sim.scriptTime % SCRIPT_PHASE_DURATION;

        if (phaseIndex !== sim.scriptPhaseIndex) {
          sim.scriptPhaseIndex = phaseIndex;
          configureScriptPhase(phaseIndex);
        }

        const phase = SCRIPT_PHASES[phaseIndex];
        const active = sim.vehicles.filter(v => !v.scriptInactive);
        const lead = active[0];

        // Script the maneuver timing, but let the lane transition itself remain smooth.
        if (local >= phase.maneuverAt && phaseIndex !== 3) {
          sim.targetLane = phase.targetLane;
          sim.controllerMode = 'OVERTAKE';
          sim.overtakingId = lead?.id ?? null;
          updateUI('AI OVERTAKE — EXECUTING', `SAFE CORRIDOR • LANE ${phase.targetLane + 1}`);
        }

        if (phaseIndex === 3 && lead) {
          if (local >= phase.maneuverAt && lead.targetLane === undefined) {
            lead.targetLane = 0;
            lead.indicator = -1;
            lead.indicatorTimer = 1.3;
            updateUI('LEAD VEHICLE YIELDING', 'ADJACENT LANE CLEAR');
          }
        }

        if (phase.returnAt !== null && local >= phase.returnAt) {
          sim.targetLane = 1;
          sim.controllerMode = 'MERGE';
          updateUI('OVERTAKE COMPLETE', 'RETURNING TO CRUISE LANE');
        }

        // POTHOLE AVOIDANCE LAYER -------------------------------------------------
        // Potholes use the same decision pattern as vehicle avoidance:
        // DETECT -> CHECK ADJACENT LANE -> COMMIT TO LANE CHANGE -> CLEAR -> MERGE.
        // The scripted traffic scenario does not disable this hazard layer.
        const detectedPothole = findUpcomingPothole(sim);

        if (!sim.potholeAvoidanceActive && detectedPothole) {
          const dodgeLane = choosePotholeAvoidanceLane(sim, detectedPothole.pothole);

          if (dodgeLane !== null && dodgeLane !== Math.round(sim.egoLane)) {
            sim.potholeAvoidanceActive = true;
            sim.potholeId = detectedPothole.pothole.id;
            sim.potholeTargetLane = dodgeLane;
            sim.potholeDetectedGap = detectedPothole.gap;
            sim.potholeReturnLane = Math.round(sim.egoLane);
            sim.potholePreviousMode = sim.controllerMode;
            sim.targetLane = dodgeLane;
            sim.controllerMode = 'POTHOLE_AVOID';
            updateUI(
              dodgeLane < sim.egoLane
                ? 'POTHOLE DETECTED — DODGING LEFT'
                : 'POTHOLE DETECTED — DODGING RIGHT',
              `ROAD HAZARD • ${detectedPothole.gap.toFixed(0)}m AHEAD`
            );
          } else if (dodgeLane === null) {
            // No safe lane yet: behave like the car-following controller and wait
            // rather than driving over the pothole or cutting through traffic.
            updateUI('POTHOLE DETECTED — WAITING', 'NO SAFE LANE YET');
          }
        }

        if (sim.potholeAvoidanceActive) {
          const activePothole = POTHOLES.find(p => p.id === sim.potholeId);
          const activeGap = activePothole
            ? signedTrackGap(activePothole.s, sim.egoS)
            : -Infinity;
          sim.potholeDetectedGap = activeGap;

          // Only release the maneuver once the Dodge has physically cleared the
          // full pothole length plus a safety margin.
          const clearDistance = activePothole
            ? Math.max(12, activePothole.length * 0.75 + POTHOLE_CLEAR_GAP)
            : 12;

          if (!activePothole || activeGap < -clearDistance) {
            sim.potholeAvoidanceActive = false;
            sim.potholeId = null;
            sim.potholeTargetLane = null;
            sim.targetLane = Number.isFinite(sim.potholeReturnLane)
              ? sim.potholeReturnLane
              : Math.round(sim.egoLane);
            sim.controllerMode = 'MERGE';
            updateUI('POTHOLE CLEARED — RETURNING', `LANE ${sim.targetLane + 1}`);
          } else {
            // Reassert the pothole target every frame. Scripted overtakes/merges run
            // before this block and must never pull the Dodge back over the hazard.
            if (Number.isFinite(sim.potholeTargetLane) &&
              Math.abs(sim.targetLane - sim.potholeTargetLane) > 0.001) {
              sim.targetLane = sim.potholeTargetLane;
            }
            sim.controllerMode = 'POTHOLE_AVOID';
            updateUI(
              sim.targetLane < sim.egoLane
                ? 'POTHOLE DETECTED — DODGING LEFT'
                : 'POTHOLE DETECTED — DODGING RIGHT',
              `ROAD HAZARD • ${Math.max(0, activeGap).toFixed(0)}m AHEAD`
            );
          }
        }

        // HARD POTHOLE PRIORITY ----------------------------------------------------
        // The scripted presentation may request an overtake/merge every frame.
        // Once a pothole dodge is committed, that request must never overwrite the
        // hazard target. Re-apply the pothole lane immediately before movement.
        if (sim.potholeAvoidanceActive && Number.isFinite(sim.potholeTargetLane)) {
          // HARD GUARANTEE: while avoiding a pothole, the Dodge can never be
          // commanded back onto the hazard lane by the scripted overtaking logic.
          const activePothole = POTHOLES.find(p => p.id === sim.potholeId);
          if (activePothole && Math.abs(sim.potholeTargetLane - activePothole.lane) <= POTHOLE_LANE_BUFFER) {
            const current = clamp(Math.round(sim.egoLane), 0, 3);
            const safeSide = current > activePothole.lane ? current + 1 : current - 1;
            const alternatives = [safeSide, activePothole.lane < 1 ? 2 : 1, activePothole.lane > 2 ? 1 : 2]
              .filter(lane => lane >= 0 && lane <= 3 && Math.abs(lane - activePothole.lane) > POTHOLE_LANE_BUFFER);
            sim.potholeTargetLane = alternatives[0] ?? current;
          }
          sim.targetLane = sim.potholeTargetLane;
          sim.controllerMode = 'POTHOLE_AVOID';
          sim.waitingForTargetClear = false;
        }

        // Smooth Dodge lane motion. Road-crossing hazards are reasserted here
        // after the scripted phase logic so the Dodge always moves away from the
        // pedestrian/animal rather than following an overtake command.
        updateRoadsideHazardPriority(sim, roadsideLife);

        // Smooth Dodge lane motion.
        const laneDistance = Math.abs(sim.targetLane - sim.egoLane);
        if (laneDistance > 0.002) {
          const rate = sim.potholeAvoidanceActive ? 7.0 : 5.2;
          sim.egoLane += (sim.targetLane - sim.egoLane) * (1 - Math.exp(-rate * dt));
        }
        if (Math.abs(sim.targetLane - sim.egoLane) < 0.008) sim.egoLane = sim.targetLane;

        // Fixed simulation: vehicles keep predetermined speeds and only scripted
        // lane changes occur. No random overtakes, random lane cuts or random braking.
        for (const v of active) {
          if (v.targetLane !== undefined) {
            v.lane += (v.targetLane - v.lane) * (1 - Math.exp(-2.4 * dt));
            if (Math.abs(v.targetLane - v.lane) < 0.012) {
              v.lane = v.targetLane;
              v.targetLane = undefined;
              v.indicator = 0;
            }
          }
          v.s = (v.s + (v.speed / 3.6) * dt) % TRACK_LENGTH;
          v.indicatorTimer = Math.max(0, v.indicatorTimer - dt);
        }

        const nextScriptEgoS = sim.egoS + EGO_SPEED_MS * dt;
        if (nextScriptEgoS >= TRACK_LENGTH) {
          restartSimulation();
          return;
        }
        sim.egoS = nextScriptEgoS % TRACK_LENGTH;
        sim.controllerMode = sim.controllerMode === 'MERGE' && Math.abs(sim.targetLane - sim.egoLane) < 0.02
          ? 'CRUISE'
          : sim.controllerMode;
      };

      const sensorCanvas = id => document.getElementById(id);
      const fitSensorCanvas = canvas => {
        if (!canvas) return null;
        const w = Math.max(120, canvas.clientWidth || 160);
        const h = Math.max(70, canvas.clientHeight || 82);
        const dpr = Math.min(window.devicePixelRatio || 1, 1.5);
        const tw = Math.round(w * dpr), th = Math.round(h * dpr);
        if (canvas.width !== tw || canvas.height !== th) { canvas.width = tw; canvas.height = th; }
        const ctx = canvas.getContext('2d');
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
        return { ctx, w, h };
      };
      const worldSensorPoint = (point, sim) => {
        const ego = sim.egoRoot ? new THREE.Vector3() : roadPoint(sim.egoS, (sim.egoLane - 1.5) * LANE_WIDTH, 0);
        if (sim.egoRoot) sim.egoRoot.getWorldPosition(ego);
        const heading = roadHeading(sim.egoS);
        const f = new THREE.Vector3(Math.sin(heading), 0, -Math.cos(heading));
        const r = new THREE.Vector3(Math.cos(heading), 0, Math.sin(heading));
        const rel = point.clone().sub(ego); rel.y = 0;
        return { forward: rel.dot(f), lateral: rel.dot(r), distance: rel.length() };
      };
      const collectSensorTargets = sim => {
        const out = [];
        const addTarget = (kind, point, id = '') => {
          const q = worldSensorPoint(point, sim);
          if (q.forward > -15 && q.forward < 120 && Math.abs(q.lateral) < 35) {
            out.push({ kind, id, forward: q.forward, lateral: q.lateral, distance: q.distance });
          }
        };
        for (const v of sim.vehicles || []) {
          if (v.scriptInactive || !v.root?.visible) continue;
          const p = new THREE.Vector3(); v.root.getWorldPosition(p);
          addTarget('vehicle', p, v.id || 'vehicle');
        }
        const life = roadsideLife?.userData?.life || [];
        for (const a of life) {
          if (!a.visible) continue;
          const p = new THREE.Vector3(); a.getWorldPosition(p);
          addTarget(a.userData?.type === 'person' ? 'person' : 'animal', p, a.userData?.id || a.userData?.type || 'actor');
        }
        for (const p of POTHOLES) {
          const qPoint = roadPoint(p.s, (p.lane - 1.5) * LANE_WIDTH, .2);
          addTarget('pothole', qPoint, `pothole-${p.s}-${p.lane}`);
        }
        return out.sort((a, b) => a.forward - b.forward);
      };

      const getSensorViews = targets => {
        // All four feeds are derived from the same live world-space targets, but each
        // feed applies its own physically-motivated range/FOV instead of sharing one
        // generic object list. This keeps the readouts consistent and deterministic.
        const camera = targets.filter(t =>
          t.forward >= 0 && t.forward <= 120 && Math.abs(t.lateral) <= 34
        );
        const lidar = targets.filter(t =>
          t.forward >= -4 && t.forward <= 100 && Math.abs(t.lateral) <= 32
        );
        const radar = targets.filter(t => {
          if (t.forward < 0 || t.forward > 120 || t.kind === 'pothole' || t.kind === 'animal') return false;
          const halfWidth = Math.min(30, Math.max(3.5, t.forward * 0.72));
          return Math.abs(t.lateral) <= halfWidth;
        });
        const ultrasonic = targets.filter(t =>
          t.forward >= 0 && t.forward <= 18 && Math.abs(t.lateral) <= 2.8
        );
        return { camera, lidar, radar, ultrasonic };
      };
      const updatePlannerTelemetry = sim => {
        // Presentation-only telemetry: derive the same live radar/planner values
        // from the rendered traffic state without changing any driving decisions.
        let nearest = null;
        let nearestGap = Infinity;
        const egoLane = sim.egoLane;
        for (const vehicle of sim.vehicles || []) {
          if (vehicle.scriptInactive || !vehicle.root?.visible) continue;
          const lane = trafficLaneAtTime(vehicle, 0);
          if (Math.abs(lane - egoLane) > 0.48) continue;
          const gap = signedTrackGap(vehicle.s, sim.egoS);
          if (gap > 3 && gap < nearestGap) {
            nearest = vehicle;
            nearestGap = gap;
          }
        }

        sim.radarGap = nearest ? nearestGap : Infinity;
        sim.radarLeadId = nearest?.id ?? null;
        if (nearestGap <= RADAR_INNER_RANGE) sim.radarZone = 'INNER';
        else if (nearestGap <= RADAR_MIDDLE_RANGE) sim.radarZone = 'MIDDLE';
        else if (nearestGap <= RADAR_OUTER_RANGE) sim.radarZone = 'OUTER';
        else sim.radarZone = 'NONE';

        const currentLane = Math.round(sim.egoLane);
        const targetLane = Math.round(sim.targetLane);
        sim.predictedOvertakeLane = currentLane !== targetLane
          ? targetLane
          : (sim.controllerMode === 'OVERTAKE' && Number.isFinite(sim.planLane) ? Math.round(sim.planLane) : null);

        // Re-render the dashboard at ~10 FPS so React-rendered fields (not only
        // DOM data-live fields) stay synchronized with the simulation.
        const now = performance.now();
        if (now - sim.lastDashboardPaint >= 100) {
          sim.lastDashboardPaint = now;
          setDashboardTick(t => t + 1);
        }
      };

      const updateSensorFeeds = sim => {
        const targets = collectSensorTargets(sim);
        const views = getSensorViews(targets);
        const cam = sensorCanvas('navion-sensor-camera');
        const lidar = sensorCanvas('navion-sensor-lidar');
        const radar = sensorCanvas('navion-sensor-radar');
        const ultra = sensorCanvas('navion-sensor-ultrasonic');
        const drawDot = (ctx, x, y, r, kind) => {
          ctx.fillStyle = kind === 'person' ? '#ffd35a' : kind === 'pothole' ? '#ff6b61' : kind === 'animal' ? '#9b8cff' : '#38e58d';
          ctx.beginPath(); ctx.arc(x, y, r, 0, Math.PI * 2); ctx.fill();
        };
        const cf = fitSensorCanvas(cam);
        if (cf) {
          try { cf.ctx.drawImage(renderer.domElement, 0, 0, cf.w, cf.h); } catch (e) { }
          cf.ctx.fillStyle = 'rgba(3,15,23,.86)';
          cf.ctx.font = '6px monospace';
          cf.ctx.fillText(`${views.camera.length} VISIBLE OBJECTS • 120m`, 6, 9);
        }
        const lf = fitSensorCanvas(lidar);
        if (lf) {
          const { ctx, w, h } = lf;
          ctx.fillStyle = '#031018'; ctx.fillRect(0, 0, w, h);
          ctx.strokeStyle = 'rgba(75,220,255,.14)';
          ctx.beginPath(); ctx.moveTo(w / 2, 0); ctx.lineTo(w / 2, h); ctx.moveTo(0, h - 15); ctx.lineTo(w, h - 15); ctx.stroke();
          for (const t of views.lidar) {
            const baseX = w / 2 + (t.lateral / 32) * (w * .43);
            const baseY = h - 14 - (Math.min(100, Math.max(0, t.forward)) / 100) * (h - 24);
            const spread = t.kind === 'vehicle' ? 4.5 : t.kind === 'person' ? 2.2 : t.kind === 'pothole' ? 3.2 : 2.6;
            const count = t.kind === 'vehicle' ? 16 : t.kind === 'person' ? 9 : 7;
            const phase = ((Math.round(t.forward * 10) + Math.round(t.lateral * 10)) % 17) * 0.14;
            for (let j = 0; j < count; j++) {
              const a = phase + j * 2.39996;
              const rr = spread * (0.35 + (j % 4) * 0.18);
              drawDot(ctx, baseX + Math.cos(a) * rr, baseY + Math.sin(a) * rr * .58, 1.05, t.kind);
            }
            drawDot(ctx, baseX, baseY, t.forward < 18 ? 2 : 1.5, t.kind);
          }
          ctx.fillStyle = '#8aa2af'; ctx.font = '6px monospace'; ctx.fillText(`${views.lidar.length} TARGETS • 100m`, 6, 9);
        }
        const rf = fitSensorCanvas(radar);
        if (rf) {
          const { ctx, w, h } = rf;
          ctx.fillStyle = '#020c14'; ctx.fillRect(0, 0, w, h);
          ctx.strokeStyle = 'rgba(75,220,255,.22)';
          [0.72, 0.48, 0.24].forEach(scale => { ctx.beginPath(); ctx.moveTo(w / 2, h - 8); ctx.arc(w / 2, h - 8, w * scale, Math.PI * 1.2, Math.PI * 1.8); ctx.stroke(); });
          for (const t of views.radar) {
            const x = w / 2 + (t.lateral / 30) * (w * .45);
            const y = h - 10 - (Math.min(120, Math.max(0, t.forward)) / 120) * (h - 20);
            drawDot(ctx, x, y, t.forward < 14 ? 3 : 2, t.kind);
          }
          const nearestRadar = views.radar[0];
          ctx.fillStyle = '#8aa2af'; ctx.font = '6px monospace';
          ctx.fillText(`RANGE 120m • ${views.radar.length} OBJECTS`, 6, 9);
          if (nearestRadar) ctx.fillText(`NEAREST ${nearestRadar.forward.toFixed(1)}m`, 6, h - 5);
        }
        const uf = fitSensorCanvas(ultra);
        if (uf) {
          const { ctx, w, h } = uf;
          ctx.fillStyle = '#07131c'; ctx.fillRect(0, 0, w, h);
          ctx.strokeStyle = 'rgba(255,211,90,.2)';
          for (let x = 0; x < 4; x++) { ctx.beginPath(); ctx.arc(w / 2, h - 6, 12 + x * 11, Math.PI, Math.PI * 2); ctx.stroke(); }
          for (const t of views.ultrasonic) {
            const x = w / 2 + (t.lateral / 2.8) * (w * .40);
            const y = h - 13 - (Math.min(18, Math.max(0, t.forward)) / 18) * (h - 26);
            drawDot(ctx, x, y, t.forward < 6 ? 3 : 2, t.kind);
          }
          const nearestUltra = views.ultrasonic[0];
          ctx.fillStyle = '#8aa2af'; ctx.font = '6px monospace';
          ctx.fillText(`0–18m • ${views.ultrasonic.length} NEAR`, 6, 9);
          if (nearestUltra) ctx.fillText(`NEAREST ${nearestUltra.forward.toFixed(1)}m`, 6, h - 5);
        }
        const set = (id, val) => { document.querySelectorAll(`[data-live="${id}"]`).forEach(e => { e.textContent = val; }); };
        const front = views.radar.filter(t => t.forward > 0).sort((a, b) => a.forward - b.forward)[0];
        const livePhase = sim.uiPhase || sim.controllerMode || 'CRUISE';
        const liveTarget = sim.uiTarget || `LANE ${Math.round(sim.targetLane) + 1}`;
        const liveSpeed = Number.isFinite(sim.egoSpeedKmh) ? sim.egoSpeedKmh : EGO_SPEED_KMH;
        set('live-phase', livePhase); set('live-target', liveTarget); set('live-ego-lane', `LANE ${Math.round(sim.egoLane) + 1}`); set('live-target-lane', `LANE ${Math.round(sim.targetLane) + 1}`);
        set('live-radar-gap', front ? `${front.forward.toFixed(1)} m` : 'CLEAR');
        set('live-object-count', String(views.camera.length));
        set('live-vehicle-count', String(targets.filter(t => t.kind === 'vehicle' && t.forward >= 0 && t.forward <= 120).length));
        set('live-person-count', String(targets.filter(t => t.kind === 'person' && t.forward >= 0 && t.forward <= 100).length));
        set('live-pothole-count', String(targets.filter(t => t.kind === 'pothole' && t.forward >= 0 && t.forward <= 120).length));
        set('live-lidar-count', String(views.lidar.length));
        set('live-radar-count', String(views.radar.length));
        set('live-ultra-count', String(views.ultrasonic.length));
        set('live-ultra-gap', views.ultrasonic[0] ? `${views.ultrasonic[0].forward.toFixed(1)} m` : 'CLEAR');
        set('live-controller', sim.controllerMode || 'CRUISE'); set('live-radar-zone', sim.radarZone || 'NONE');
        set('live-predicted-lane', Number.isFinite(sim.predictedOvertakeLane) ? `LANE ${Math.round(sim.predictedOvertakeLane) + 1}` : '—');
        let liveTtc = Number.isFinite(sim.radarTtc) ? sim.radarTtc : Infinity;
        const radarLead = front?.kind === 'vehicle' ? (sim.vehicles || []).find(v => v.id === front.id) : null;
        if (front && radarLead) {
          const relativeSpeed = liveSpeed - radarLead.speed;
          liveTtc = relativeSpeed > 0.5 ? front.forward / (relativeSpeed / 3.6) : Infinity;
        }
        set('live-ttc', Number.isFinite(liveTtc) && liveTtc < 999 ? `${liveTtc.toFixed(1)} s` : '—');
        set('live-ego-speed', `${liveSpeed.toFixed(0)}`); set('live-header-speed', `${liveSpeed.toFixed(0)} KM/H`);
        set('live-log-ai', livePhase); set('live-path-state', livePhase);
        const camNearest = views.camera[0];
        const radarNearest = views.radar[0];
        const nearestAgree = camNearest && radarNearest ? Math.abs(camNearest.forward - radarNearest.forward) <= 4 : !camNearest && !radarNearest;
        let fusion = views.camera.length === 0 ? 96 : 84;
        if (nearestAgree) fusion += 7;
        if (views.lidar.length >= views.radar.length) fusion += 3;
        if (views.ultrasonic[0] && radarNearest && Math.abs(views.ultrasonic[0].forward - radarNearest.forward) <= 3) fusion += 4;
        fusion = Math.max(75, Math.min(99, fusion));
        set('live-fusion', `${fusion}%`);
        const fb = document.getElementById('live-fusion-bar'); if (fb) fb.style.width = `${fusion}%`;
        set('live-system-state', sim.running ? 'RUNNING' : 'PAUSED');
      };

      const animate = now => {
        const sim = simRef.current;

        if (!sim.last) {
          sim.last = now;
        }

        const dt = Math.min(
          (now - sim.last) / 1000,
          0.032
        );

        sim.last = now;
        sim.aiAccumulator += dt;
        sim.safetyAccumulator += dt;
        if (sim.collisionHold > 0) sim.collisionHold = Math.max(0, sim.collisionHold - dt);

        try {
          // Drive the ego vehicle and camera immediately. Traffic models may still be
          // loading, but the road scene should already feel alive and the camera must
          // stay locked to the moving Dodge rather than falling back to a distant view.
          if (sim.running) {
            if (!SCRIPTED_SIMULATION) {
              // ROAD-HAZARD DETECTION / AVOIDANCE --------------------------------------
              // This is deliberately independent of the traffic-density and radar
              // settings. The radar cones remain visible and continue detecting traffic.
              // The pothole detector is a separate road-surface perception layer.
              const detectedPothole = findUpcomingPothole(sim);

              if (!sim.potholeAvoidanceActive && detectedPothole) {
                const dodgeLane = choosePotholeAvoidanceLane(sim, detectedPothole.pothole);
                if (dodgeLane !== null && dodgeLane !== Math.round(sim.egoLane)) {
                  sim.potholeAvoidanceActive = true;
                  sim.potholeId = detectedPothole.pothole.id;
                  sim.potholeDetectedGap = detectedPothole.gap;
                  sim.potholeReturnLane = Number.isFinite(sim.targetLane)
                    ? sim.targetLane
                    : Math.round(sim.egoLane);
                  sim.potholePreviousMode = sim.controllerMode;
                  sim.targetLane = dodgeLane;
                  sim.controllerMode = 'POTHOLE_AVOID';
                  updateUI(
                    dodgeLane < sim.egoLane
                      ? 'POTHOLE DETECTED — DODGING LEFT'
                      : 'POTHOLE DETECTED — DODGING RIGHT',
                    `ROAD HAZARD • ${detectedPothole.gap.toFixed(0)}m AHEAD`
                  );
                }
              }

              if (sim.potholeAvoidanceActive) {
                const activePothole = POTHOLES.find(p => p.id === sim.potholeId);
                const activeGap = activePothole
                  ? signedTrackGap(activePothole.s, sim.egoS)
                  : -Infinity;
                sim.potholeDetectedGap = activeGap;

                if (!activePothole || activeGap < -12) {
                  sim.potholeAvoidanceActive = false;
                  sim.potholeId = null;
                  sim.targetLane = sim.potholeReturnLane;
                  sim.controllerMode = 'MERGE';
                  updateUI('POTHOLE CLEARED — RETURNING', `LANE ${sim.potholeReturnLane + 1}`);
                } else {
                  // Keep the hazard avoidance committed while the Dodge crosses the
                  // pothole position; traffic logic must not overwrite this target.
                  sim.controllerMode = 'POTHOLE_AVOID';
                  updateUI(
                    sim.targetLane < sim.egoLane
                      ? 'POTHOLE DETECTED — DODGING LEFT'
                      : 'POTHOLE DETECTED — DODGING RIGHT',
                    `ROAD HAZARD • ${Math.max(0, activeGap).toFixed(0)}m AHEAD`
                  );
                }
              }

              // MAIN CAR / OVERTAKE CONTROLLER ------------------------------------------
              // Clean state machine: CRUISE -> THINK -> OVERTAKE -> MERGE.
              // The radar is visual only. Lane decisions are driven by traffic geometry.
              const currentLane = clamp(Math.round(sim.egoLane), 0, 3);

              let lead = null;
              let leadGap = Infinity;
              for (const vehicle of sim.vehicles) {
                const liveLane = trafficLaneAtTime(vehicle, 0);
                if (Math.abs(liveLane - sim.egoLane) > 0.46) continue;
                const gap = signedTrackGap(vehicle.s, sim.egoS);
                if (gap > 4 && gap < leadGap) {
                  lead = vehicle;
                  leadGap = gap;
                }
              }

              // Three-layer radar visualization only.
              let radarZone = 'NONE';
              let radarRelativeSpeed = 0;
              let radarTtc = Infinity;
              if (lead) {
                if (leadGap <= RADAR_INNER_RANGE) radarZone = 'INNER';
                else if (leadGap <= RADAR_MIDDLE_RANGE) radarZone = 'MIDDLE';
                else if (leadGap <= RADAR_OUTER_RANGE) radarZone = 'OUTER';
                radarRelativeSpeed = EGO_SPEED_KMH - lead.speed;
                if (radarRelativeSpeed > 0.5) {
                  radarTtc = leadGap / (radarRelativeSpeed / 3.6);
                }
              }
              sim.radarZone = radarZone;
              sim.radarLeadId = lead?.id ?? null;
              sim.radarGap = leadGap;
              sim.radarRelativeSpeed = radarRelativeSpeed;
              sim.radarTtc = radarTtc;

              const laneGap = (lane, ignoreId = null) => {
                if (lane < 0 || lane > 3) return null;
                let front = Infinity;
                let rear = Infinity;
                for (const v of sim.vehicles) {
                  if (v.id === ignoreId) continue;
                  const vLane = trafficLaneAtTime(v, 0);
                  if (Math.abs(vLane - lane) > 0.48) continue;
                  const gap = signedTrackGap(v.s, sim.egoS);
                  if (gap >= 0) front = Math.min(front, gap);
                  else rear = Math.min(rear, Math.abs(gap));
                }
                return { lane, front, rear };
              };

              const chooseOvertakeLane = () => {
                const candidates = [
                  currentLane - 1,
                  currentLane + 1,
                  currentLane - 2,
                  currentLane + 2
                ].filter(lane => lane >= 0 && lane <= 3);

                const scored = candidates.map(lane => {
                  const laneDistance = Math.abs(lane - currentLane);
                  const g = laneGap(lane, lead?.id ?? null);
                  if (!g) return null;

                  // A lane that merely has a 10–15 m gap is NOT considered a usable
                  // passing lane. If a vehicle is visibly beside/near the Dodge, the
                  // Dodge must choose the other side instead of cutting across it.
                  // For a two-lane jump, every lane in the swept sequence must also be
                  // clear enough before the maneuver is committed.
                  const minFront = laneDistance > 1 ? 32 : 28;
                  const minRear = laneDistance > 1 ? 28 : 24;
                  if (g.front < minFront || g.rear < minRear) return null;

                  const step = lane > currentLane ? 1 : -1;
                  for (let transitionLane = currentLane + step;
                    transitionLane !== lane + step;
                    transitionLane += step) {
                    const transitionGap = laneGap(transitionLane, lead?.id ?? null);
                    if (!transitionGap) return null;
                    const transitionFront = laneDistance > 1 ? 32 : 28;
                    const transitionRear = laneDistance > 1 ? 28 : 24;
                    if (transitionGap.front < transitionFront || transitionGap.rear < transitionRear) {
                      return null;
                    }
                  }

                  // Never box the Dodge between two side-by-side traffic vehicles.
                  if (wouldCreateEgoSideBySide(sim, lane, sim.egoS, lead?.id ?? null, 65)) return null;

                  // Do not choose a lane that is already occupied inside the projected
                  // green passing corridor. The corridor must remain physically usable
                  // for the Dodge's swept lane change.
                  for (const v of sim.vehicles) {
                    if (v.id === lead?.id) continue;
                    const vLane = trafficLaneAtTime(v, 0.45);
                    const vS = (v.s + (v.speed / 3.6) * 0.45) % TRACK_LENGTH;
                    if (trafficConflictsWithEgoCorridor(sim, v, vLane, vS)) return null;
                  }

                  // IMPORTANT: ignore the lead being overtaken while checking the
                  // swept path. It remains in the old lane; treating it as a blocker
                  // here was the reason the Dodge repeatedly chose not to change lanes.
                  const pathSafe = egoLanePathSafe(
                    sim,
                    lane,
                    lead?.id ?? null,
                    laneDistance > 1 ? 1.35 : 0.92
                  );
                  if (!pathSafe) return null;

                  const score =
                    Math.min(g.front, g.rear) +
                    (g.front > 30 ? 8 : 0) +
                    (g.rear > 22 ? 5 : 0) +
                    (lane === 1 || lane === 2 ? 3 : 0) +
                    (laneDistance > 1 ? 1.5 : 0);

                  return { ...g, lane, score, laneDistance };
                }).filter(Boolean).sort((a, b) => b.score - a.score);

                return scored[0]?.lane ?? null;
              };

              if (sim.mergeCooldown > 0) sim.mergeCooldown -= dt;
              if (sim.planTimer > 0) sim.planTimer = Math.max(0, sim.planTimer - dt);
              if (sim.laneChangeCooldown > 0) sim.laneChangeCooldown -= dt;

              // Recover cleanly if the selected target disappears.
              if (sim.overtakingId != null && !sim.vehicles.some(v => v.id === sim.overtakingId)) {
                sim.overtakingId = null;
                sim.controllerMode = 'CRUISE';
                sim.targetLane = Math.round(sim.egoLane);
                sim.waitingForTargetClear = false;
                sim.waitBlockerId = null;
              }

              // CRUISE: notice a materially slower lead early.
              if (
                sim.controllerMode === 'CRUISE' &&
                lead &&
                leadGap < 105 &&
                lead.speed < EGO_SPEED_KMH - 8
              ) {
                sim.controllerMode = 'THINK';
                sim.planTimer = Math.max(sim.planTimer, 0.28);
                sim.planTargetId = lead.id;
                updateUI('THINKING — SLOW VEHICLE DETECTED', `${leadGap.toFixed(0)}m • CHECKING ADJACENT LANES`);
              }

              // THINK: pick a target lane. If the best lane is temporarily occupied,
              // switch into FOLLOW_WAIT instead of either stopping completely or blindly
              // entering the neighbouring car. The Dodge deliberately slows to ~60 km/h
              // until the blocking car moves ahead and the target lane opens.
              if (sim.controllerMode === 'THINK' && sim.planTimer <= 0) {
                const target = sim.vehicles.find(v => v.id === sim.planTargetId) || lead;
                const chosen = chooseOvertakeLane();

                if (target && chosen !== null) {
                  sim.overtakeFromLane = currentLane;
                  sim.targetLane = chosen;
                  sim.overtakingId = target.id;
                  sim.controllerMode = 'OVERTAKE';
                  sim.waitingForTargetClear = false;
                  sim.waitBlockerId = null;
                  sim.laneChangeCooldown = 0;
                  sim.mergeCooldown = 1.0;
                  sim.overtakeStartS = sim.egoS;
                  sim.demoOvertakeDone = true;
                  updateUI(
                    chosen < sim.egoLane ? 'DODGING LEFT — COMMITTED' : 'DODGING RIGHT — COMMITTED',
                    'LANE CHANGE EXECUTING'
                  );
                  if ((frame & 7) === 0) setReplans(v => v + 1);
                } else {
                  // No lane is currently safe. Pick the lane with the largest current
                  // gap and wait for the vehicle beside/ahead of that lane to move out.
                  const candidates = [currentLane - 1, currentLane + 1, currentLane - 2, currentLane + 2]
                    .filter(lane => lane >= 0 && lane <= 3);
                  let bestWaitLane = null;
                  let bestWaitScore = -Infinity;
                  for (const lane of candidates) {
                    const g = laneGap(lane, target?.id ?? null);
                    if (!g) continue;
                    const score = Math.min(g.front, g.rear) + (lane === 1 || lane === 2 ? 2 : 0);
                    if (score > bestWaitScore) {
                      bestWaitScore = score;
                      bestWaitLane = lane;
                    }
                  }

                  if (target && bestWaitLane !== null) {
                    sim.waitLane = bestWaitLane;
                    sim.overtakingId = target.id;

                    // Remember the vehicle currently occupying/blocking the intended
                    // passing lane. The Dodge will follow slowly until this specific
                    // vehicle is clearly ahead, then change lanes. This is deliberately
                    // simpler than re-running the entire safety planner every frame.
                    let blocker = null;
                    let blockerAbs = Infinity;
                    for (const v of sim.vehicles) {
                      if (v.id === target.id) continue;
                      const vLane = trafficLaneAtTime(v, 0);
                      if (Math.abs(vLane - bestWaitLane) > 0.45) continue;
                      const gap = signedTrackGap(v.s, sim.egoS);
                      if (Math.abs(gap) < blockerAbs) {
                        blocker = v;
                        blockerAbs = Math.abs(gap);
                      }
                    }
                    sim.waitBlockerId = blocker?.id ?? null;
                    sim.controllerMode = 'FOLLOW_WAIT';
                    sim.waitingForTargetClear = true;
                    sim.planTimer = 0;
                    updateUI('FOLLOWING SLOWLY — WAITING FOR SIDE CAR', `TARGET LANE ${bestWaitLane + 1}`);
                  } else {
                    sim.planTimer = 0.18;
                    updateUI('FOLLOWING SLOWLY — RECHECKING', 'SEARCHING FOR SAFE LANE');
                  }
                }
              }

              // FOLLOW_WAIT: intentionally slow the Dodge while a neighbouring vehicle
              // is beside/ahead of the intended passing lane. As soon as the blocking
              // vehicle moves ahead and the lane opens, commit to the lane change.
              if (sim.controllerMode === 'FOLLOW_WAIT') {
                const target = sim.overtakingId != null
                  ? sim.vehicles.find(v => v.id === sim.overtakingId)
                  : lead;
                const waitLane = Number.isFinite(sim.waitLane) ? sim.waitLane : null;
                const waitGap = waitLane == null ? null : laneGap(waitLane, target?.id ?? null);

                // Find the remembered side-lane blocker again. When it moves ahead of
                // the Dodge there is finally space to move over.
                const blocker = sim.waitBlockerId != null
                  ? sim.vehicles.find(v => v.id === sim.waitBlockerId)
                  : null;
                const blockerGap = blocker ? signedTrackGap(blocker.s, sim.egoS) : Infinity;

                // Main behavior requested: while a car is beside/just ahead in the
                // target lane, slow the Dodge heavily. Once that car has moved ahead
                // and a modest rear gap exists, immediately change lanes.
                const blockerHasGoneAhead = blocker
                  ? blockerGap > 8
                  : Boolean(waitGap && waitGap.front > 10);
                const gapOpen = waitGap && waitGap.front >= 12 && waitGap.rear >= 14;
                const frontCarHasMoved = leadGap > 18;
                const pathOpen = waitLane != null && egoLanePathSafe(
                  sim,
                  waitLane,
                  target?.id ?? null,
                  Math.abs(waitLane - sim.egoLane) > 1 ? 1.0 : 0.78
                );

                if (waitLane != null && pathOpen && (gapOpen || (blockerHasGoneAhead && frontCarHasMoved))) {
                  sim.overtakeFromLane = Math.round(sim.egoLane);
                  sim.targetLane = waitLane;
                  sim.controllerMode = 'OVERTAKE';
                  sim.waitingForTargetClear = false;
                  sim.waitBlockerId = null;
                  sim.laneChangeCooldown = 0;
                  sim.mergeCooldown = 1.0;
                  updateUI(
                    waitLane < sim.egoLane ? 'SIDE CAR AHEAD — DODGING LEFT' : 'SIDE CAR AHEAD — DODGING RIGHT',
                    'LANE CHANGE EXECUTING'
                  );
                } else {
                  updateUI('FOLLOWING SLOWLY — WAITING FOR GAP', blocker ? 'SIDE CAR MOVING AHEAD' : 'TARGET LANE BLOCKED');
                }
              }

              // OVERTAKE: move toward the chosen lane regardless of the vehicle being
              // overtaken. Only a DIFFERENT vehicle in the target corridor can pause it.
              if (sim.controllerMode === 'OVERTAKE') {
                const target = sim.overtakingId != null
                  ? sim.vehicles.find(v => v.id === sim.overtakingId)
                  : null;

                let targetLaneBlocked = false;
                for (const v of sim.vehicles) {
                  if (v.id === sim.overtakingId) continue;
                  const vLane = trafficLaneAtTime(v, 0.35);
                  if (Math.abs(vLane - sim.targetLane) > 0.50) continue;
                  const gap = signedTrackGap(v.s, sim.egoS);
                  if (gap > -4 && gap < 12) {
                    targetLaneBlocked = true;
                    break;
                  }
                }

                sim.waitingForTargetClear = targetLaneBlocked;
                if (targetLaneBlocked) {
                  updateUI('DODGE PAUSED — TARGET LANE BLOCKED', 'WAITING FOR TARGET LANE');
                } else {
                  updateUI(
                    sim.targetLane < sim.egoLane ? 'DODGING LEFT' : 'DODGING RIGHT',
                    target ? `PASSING ${TRAFFIC_DEFS[target.key].label}` : 'PASSING'
                  );
                }

                // Once fully in the passing lane and the old lead is clearly behind,
                // plan the merge back to the lane we started from.
                if (!targetLaneBlocked && target && Math.abs(sim.targetLane - sim.egoLane) < 0.045) {
                  const passedGap = signedTrackGap(sim.egoS, target.s);
                  if (passedGap > 14) {
                    // First ask: is there ANOTHER slower vehicle already ahead in the
                    // current passing lane? If yes, stay in this lane and chain another
                    // overtake instead of merging back and immediately getting trapped.
                    let nextLead = null;
                    let nextGap = Infinity;
                    for (const v of sim.vehicles) {
                      if (v.id === target.id) continue;
                      const vLane = trafficLaneAtTime(v, 0);
                      if (Math.abs(vLane - sim.egoLane) > 0.45) continue;
                      const gap = signedTrackGap(v.s, sim.egoS);
                      if (gap > 6 && gap < nextGap && v.speed < EGO_SPEED_KMH - 10) {
                        nextLead = v;
                        nextGap = gap;
                      }
                    }

                    if (nextLead && nextGap < 75) {
                      sim.overtakeFromLane = Math.round(sim.egoLane);
                      sim.overtakingId = nextLead.id;
                      sim.controllerMode = 'THINK';
                      sim.planTargetId = nextLead.id;
                      sim.planTimer = 0.12;
                      sim.mergeCooldown = 0.0;
                      sim.recentOvertakeTarget = target.id;
                      sim.completedOvertakes += 1;
                      updateUI('PASSED — PREPARING NEXT OVERTAKE', `${nextGap.toFixed(0)}m • CONTINUING`);
                    } else {
                      const returnLane = clamp(Math.round(sim.overtakeFromLane), 0, 3);
                      const returnGap = laneGap(returnLane, target.id);
                      // If the original lane is not immediately safe, simply remain in
                      // the current lane. Never merge back into a blocked lane just to
                      // satisfy a fixed route pattern.
                      if (returnGap && returnGap.front >= 24 && returnGap.rear >= 18) {
                        sim.targetLane = returnLane;
                        sim.controllerMode = 'MERGE';
                        sim.overtakingId = null;
                        sim.waitingForTargetClear = false;
                        sim.mergeCooldown = 0.8;
                        sim.completedOvertakes += 1;
                        updateUI('OVERTAKE COMPLETE — MERGING BACK', 'RETURN LANE CLEAR');
                      } else {
                        sim.controllerMode = 'CRUISE';
                        sim.overtakingId = null;
                        sim.waitingForTargetClear = false;
                        sim.completedOvertakes += 1;
                        updateUI('OVERTAKE COMPLETE — STAYING IN LANE', 'ORIGINAL LANE BLOCKED');
                      }
                    }
                  }
                }
              }

              if (sim.controllerMode === 'MERGE') {
                if (Math.abs(sim.targetLane - sim.egoLane) < 0.045) {
                  sim.targetLane = Math.round(sim.egoLane);
                  sim.controllerMode = 'CRUISE';
                  sim.waitingForTargetClear = false;
                  sim.waitBlockerId = null;
                  updateUI('CRUISING', 'PATH CLEAR');
                }
              }

              // EGO SPEED RULE: side-by-side traffic must NEVER make the Dodge slow.
              // A slower vehicle directly ahead should trigger the overtake planner,
              // not the old 50 km/h FOLLOW_WAIT behavior. Keep the requested speed
              // at the 150 km/h target; the physical safety guard below may still
              // reduce actual forward advance if a vehicle is genuinely too close.
              let effectiveEgoSpeed = EGO_SPEED_MS;

              // Robust lateral movement. Before each sideways step, sweep the complete
              // transition path through nearby traffic. If the path is clear, move; if
              // not, hold the lateral position without cancelling the overtake plan.
              if (Math.abs(sim.targetLane - sim.egoLane) > 0.002 && !sim.waitingForTargetClear) {
                const pathSafe = egoLanePathSafe(
                  sim,
                  sim.targetLane,
                  sim.controllerMode === 'OVERTAKE' ? sim.overtakingId : null,
                  sim.controllerMode === 'OVERTAKE' ? 0.90 : 1.05
                );

                if (pathSafe) {
                  const laneDistance = Math.abs(sim.targetLane - sim.egoLane);
                  const laneRate = sim.controllerMode === 'OVERTAKE'
                    ? (laneDistance > 1 ? 4.9 : 6.2)
                    : 3.8;
                  const alpha = 1 - Math.exp(-laneRate * dt);
                  sim.egoLane += (sim.targetLane - sim.egoLane) * alpha;
                } else if (sim.controllerMode === 'OVERTAKE') {
                  // The destination lane was already validated at commit time. If the
                  // conservative sweep test is the only thing blocking the animation,
                  // continue the lateral move rather than deadlocking the controller.
                  const laneGapNow = laneGap(sim.targetLane, sim.overtakingId);
                  if (laneGapNow && laneGapNow.front >= 7 && laneGapNow.rear >= 7) {
                    const alpha = 1 - Math.exp(-5.0 * dt);
                    sim.egoLane += (sim.targetLane - sim.egoLane) * alpha;
                    sim.waitingForTargetClear = false;
                  } else {
                    sim.waitingForTargetClear = true;
                    updateUI('DODGE PAUSED — SAFE GAP REQUIRED', 'WAITING FOR TARGET LANE');
                  }
                }
              }
              if (Math.abs(sim.targetLane - sim.egoLane) < 0.008) {
                sim.egoLane = sim.targetLane;
              }

              if (sim.waitingForTargetClear && sim.controllerMode === 'OVERTAKE') {
                if (egoLanePathSafe(sim, sim.targetLane, sim.overtakingId, 0.82)) {
                  sim.waitingForTargetClear = false;
                  updateUI('OVERTAKING — RESUMING', 'PATH CLEAR');
                }
              }

              // Move traffic with live lane behavior: normal drivers follow traffic,
              // assertive drivers overtake more often, and a few reckless drivers
              // make fast lane cuts. All lane changes check front/rear clearance first.
              sim.vehicles.forEach(vehicle => {
                const def = TRAFFIC_DEFS[vehicle.key];

                if (vehicle.laneChangeCooldown > 0) {
                  vehicle.laneChangeCooldown -= dt;
                }

                // Find the nearest vehicle in front of this vehicle's current lane.
                let leadVehicle = null;
                let leadGap = Infinity;
                for (const other of sim.vehicles) {
                  if (other.id === vehicle.id) continue;
                  if (Math.abs(other.lane - vehicle.lane) > 0.34) continue;

                  const gap = signedTrackGap(other.s, vehicle.s);
                  if (gap > 2 && gap < leadGap) {
                    leadGap = gap;
                    leadVehicle = other;
                  }
                }

                // Ego car can also be a slower/larger lead vehicle from the traffic
                // driver's perspective.
                if (Math.abs(sim.egoLane - vehicle.lane) <= 0.34) {
                  const egoGap = signedTrackGap(sim.egoS, vehicle.s);
                  if (egoGap > 2 && egoGap < leadGap) {
                    leadGap = egoGap;
                    leadVehicle = { id: 'ego', speed: EGO_SPEED_KMH, lane: sim.egoLane, key: 'ego' };
                  }
                }

                // EGO-YIELD RULE: if the Dodge has slowed to follow and this vehicle
                // is directly ahead in the same lane, make the lead vehicle yield into
                // a safe neighbouring lane. It then stays there until the Dodge passes.
                const egoGapAhead = Math.abs(vehicle.lane - sim.egoLane) <= 0.34
                  ? signedTrackGap(vehicle.s, sim.egoS)
                  : Infinity;
                const egoIsFollowingSlow = sim.controllerMode === 'FOLLOW_WAIT' ||
                  (effectiveEgoSpeed < EGO_SPEED_MS * 0.72 && leadVehicle?.id === 'ego');
                const shouldYieldToEgo =
                  vehicle.targetLane === undefined &&
                  vehicle.laneChangeCooldown <= 0 &&
                  egoIsFollowingSlow &&
                  egoGapAhead > 8 &&
                  egoGapAhead < EGO_YIELD_TRIGGER_GAP &&
                  leadVehicle?.id === 'ego';

                if (shouldYieldToEgo) {
                  const yieldLane = chooseYieldLaneForEgo(sim, vehicle);
                  if (yieldLane !== null && yieldLane !== undefined) {
                    vehicle.originalLane = Math.round(vehicle.lane);
                    vehicle.targetLane = yieldLane;
                    vehicle.laneChangeDuration = vehicle.driverType === 'reckless' ? 0.9 : 1.2;
                    vehicle.laneChangeTimer = vehicle.laneChangeDuration;
                    vehicle.laneChangeCooldown = 2.8;
                    vehicle.indicator = yieldLane < vehicle.lane ? -1 : 1;
                    vehicle.indicatorTimer = 1.25;
                    vehicle.overtakeTargetId = 'ego';
                    vehicle.yieldingToEgo = true;
                  }
                }

                // GREEN CORRIDOR: once the Dodge is thinking about or executing an
                // overtake, do not let unrelated traffic start a lane change into the
                // projected passing corridor. If the vehicle is already inside it,
                // try a safe neighbouring lane before allowing normal random movement.
                if (vehicle.targetLane === undefined &&
                  vehicle.laneChangeCooldown <= 0 &&
                  trafficConflictsWithEgoCorridor(sim, vehicle, vehicle.lane, vehicle.s) &&
                  vehicle.id !== sim.overtakingId) {
                  const escapeCandidates = [vehicle.lane - 1, vehicle.lane + 1]
                    .filter(lane => lane >= 0 && lane <= 3);
                  let escapeLane = null;
                  let escapeScore = -Infinity;
                  for (const candidate of escapeCandidates) {
                    if (trafficConflictsWithEgoCorridor(sim, vehicle, candidate, vehicle.s)) continue;
                    const gaps = laneClearForTraffic(sim, candidate, vehicle.s, vehicle.id);
                    if (gaps.frontGap < 18 || gaps.backGap < 14) continue;
                    const score = Math.min(gaps.frontGap, gaps.backGap);
                    if (score > escapeScore) {
                      escapeScore = score;
                      escapeLane = candidate;
                    }
                  }
                  if (escapeLane !== null) {
                    vehicle.originalLane = Math.round(vehicle.lane);
                    vehicle.targetLane = escapeLane;
                    vehicle.laneChangeDuration = vehicle.driverType === 'reckless' ? 0.9 : 1.25;
                    vehicle.laneChangeTimer = vehicle.laneChangeDuration;
                    vehicle.laneChangeCooldown = 2.2;
                    vehicle.indicator = escapeLane < vehicle.lane ? -1 : 1;
                    vehicle.indicatorTimer = 0.9;
                  }
                }

                // Overtake a slower lead vehicle instead of simply tailing it.
                const shouldOvertake =
                  vehicle.targetLane === undefined &&
                  vehicle.laneChangeCooldown <= 0 &&
                  leadGap < (vehicle.driverType === 'reckless' ? 62 : 46) &&
                  leadVehicle &&
                  vehicle.speed > leadVehicle.speed + (vehicle.driverType === 'reckless' ? 1.5 : 4);

                if (shouldOvertake) {
                  const chosenLane = chooseTrafficOvertakeLane(sim, vehicle);
                  if (chosenLane !== null && chosenLane !== undefined) {
                    vehicle.originalLane = Math.round(vehicle.lane);
                    vehicle.targetLane = chosenLane;
                    vehicle.laneChangeDuration =
                      vehicle.driverType === 'reckless'
                        ? 0.95
                        : 1.45;
                    vehicle.laneChangeTimer =
                      vehicle.laneChangeDuration;
                    vehicle.laneChangeCooldown = vehicle.driverType === 'reckless' ? 1.1 : 1.8;
                    vehicle.indicator = chosenLane < vehicle.lane ? -1 : 1;
                    vehicle.indicatorTimer = vehicle.driverType === 'reckless' ? 1.0 : 1.35;
                    vehicle.overtakeTargetId = leadVehicle.id;
                  }
                }

                // Additional random lane movement makes the highway feel alive even
                // when nobody is overtaking. Reckless drivers signal late and move fast.
                if (
                  vehicle.targetLane === undefined &&
                  vehicle.laneChangeCooldown <= 0
                ) {
                  const randomChance = vehicle.driverType === 'reckless'
                    ? dt * 0.075
                    : vehicle.driverType === 'assertive'
                      ? dt * 0.045
                      : dt * 0.018;

                  if (Math.random() < randomChance) {
                    const chosenLane = chooseTrafficOvertakeLane(sim, vehicle);
                    if (chosenLane !== null && chosenLane !== undefined) {
                      const aheadBehind = laneClearForTraffic(sim, chosenLane, vehicle.s, vehicle.id);
                      if (aheadBehind.frontGap < 24 || aheadBehind.backGap < 20) {
                        vehicle.indicator = 0;
                        return;
                      }
                      vehicle.originalLane = Math.round(vehicle.lane);
                      vehicle.targetLane = chosenLane;
                      vehicle.laneChangeDuration =
                        vehicle.driverType === 'reckless'
                          ? 1.0
                          : 1.8;
                      vehicle.laneChangeTimer =
                        vehicle.laneChangeDuration;
                      vehicle.laneChangeCooldown = vehicle.driverType === 'reckless' ? 1.2 : 2.0;
                      vehicle.indicator = chosenLane < vehicle.lane ? -1 : 1;
                      vehicle.indicatorTimer = vehicle.driverType === 'reckless' ? 0.55 : 0.9;
                    }
                  }
                }

                // If the lead is tight and there is no available lane yet, bleed speed
                // smoothly rather than snapping to a stop.
                let targetSpeed = vehicle.cruiseSpeed;
                if (leadVehicle && leadGap < 18 && vehicle.targetLane === undefined) {
                  targetSpeed = Math.min(targetSpeed, leadVehicle.speed - 2.0);
                }

                // Trucks keep a calmer cruise profile; reckless cars/bikes periodically
                // accelerate above the traffic flow.
                if (def.heavy) {
                  targetSpeed = Math.min(targetSpeed, def.speedMax);
                }

                const accel = vehicle.driverType === 'reckless' ? 18 : 10;
                if (vehicle.speed < targetSpeed) {
                  vehicle.speed = Math.min(targetSpeed, vehicle.speed + accel * dt);
                } else {
                  vehicle.speed = Math.max(targetSpeed, vehicle.speed - 14 * dt);
                }

                if (vehicle.driverType === 'reckless' && !def.heavy && Math.random() < dt * 0.045) {
                  vehicle.speed = Math.max(vehicle.speed, def.speedMax + 18 + Math.random() * 18);
                }

                if (vehicle.targetLane !== undefined) {
                  vehicle.lane +=
                    (vehicle.targetLane - vehicle.lane) *
                    Math.min(1, dt * (vehicle.driverType === 'reckless' ? 2.8 : 1.25));

                  vehicle.lane = clamp(vehicle.lane, 0, 3);

                  if (Math.abs(vehicle.targetLane - vehicle.lane) < 0.015) {
                    vehicle.lane = vehicle.targetLane;
                    vehicle.targetLane = undefined;
                    vehicle.indicator = 0;

                    // Stay in the overtaking lane until the slower vehicle is clearly behind.
                    if (vehicle.overtakeTargetId != null) {
                      vehicle.laneChangeTimer = Math.max(vehicle.laneChangeTimer, 0.2);
                    }
                  }
                }

                // IMPORTANT: Once the Dodge has passed a traffic vehicle, that vehicle
                // must NOT change its behavior because of the pass. It keeps its current
                // lane, speed, indicators and normal traffic logic exactly as before.
                // Only clear the internal 'yielding to ego' marker after the Dodge is
                // safely behind it; do not trigger a lane return or speed change.
                if (
                  vehicle.overtakeTargetId === 'ego' &&
                  signedTrackGap(vehicle.s, sim.egoS) < -8
                ) {
                  vehicle.overtakeTargetId = null;
                  vehicle.yieldingToEgo = false;
                }

                // For traffic vehicles overtaking OTHER traffic, retain their existing
                // normal pass-and-return behavior. This is unrelated to the Dodge pass.
                if (
                  vehicle.targetLane === undefined &&
                  vehicle.overtakeTargetId != null &&
                  vehicle.overtakeTargetId !== 'ego' &&
                  vehicle.laneChangeCooldown <= 0
                ) {
                  const target = sim.vehicles.find(v => v.id === vehicle.overtakeTargetId);
                  const passed = target
                    ? signedTrackGap(target.s, vehicle.s) < -8
                    : true;

                  if (passed && Math.abs(vehicle.lane - vehicle.originalLane) > 0.08) {
                    const { frontGap, backGap } = laneClearForTraffic(
                      sim,
                      vehicle.originalLane,
                      vehicle.s,
                      vehicle.id
                    );

                    const safeReturn = frontGap > 16 && backGap > 14;
                    if (safeReturn) {
                      vehicle.targetLane = vehicle.originalLane;
                      vehicle.laneChangeDuration = 1.35;
                      vehicle.laneChangeTimer = vehicle.laneChangeDuration;
                      vehicle.laneChangeCooldown = 1.7;
                      vehicle.indicator = vehicle.targetLane < vehicle.lane ? -1 : 1;
                      vehicle.indicatorTimer = 0.9;
                      vehicle.overtakeTargetId = null;
                      vehicle.yieldingToEgo = false;
                    }
                  }
                }

                if (vehicle.laneChangeTimer > 0) {
                  vehicle.laneChangeTimer -= dt;
                }

                vehicle.s =
                  (vehicle.s +
                    (vehicle.speed / 3.6) * dt) %
                  TRACK_LENGTH;

                const signedGapFromEgo = signedTrackGap(vehicle.s, sim.egoS);

                // Respawn only after a vehicle has genuinely fallen out of the rear
                // traffic window. Overtaken cars are otherwise left completely alone.
                if (signedGapFromEgo < -EGO_VISIBLE_TRAFFIC_BEHIND) {
                  vehicle.s =
                    (sim.egoS +
                      280 +
                      Math.random() * 420) %
                    TRACK_LENGTH;

                  const laneChoices = [0, 1, 2, 3].sort(() => Math.random() - 0.5);
                  vehicle.lane = laneChoices.find(candidate => {
                    if (sim.controllerMode === 'FOLLOW_WAIT' &&
                      candidate === Math.round(sim.egoLane)) return false;
                    const gaps = laneClearForTraffic(sim, candidate, vehicle.s, vehicle.id);
                    return gaps.frontGap > 24 && gaps.backGap > 18;
                  });
                  if (vehicle.lane == null) vehicle.lane = laneChoices[0];
                  vehicle.originalLane = vehicle.lane;
                  vehicle.targetLane = undefined;
                  vehicle.indicator = 0;
                  vehicle.indicatorTimer = 0;
                  vehicle.overtakeTargetId = null;
                  vehicle.yieldingToEgo = false;

                  vehicle.cruiseSpeed =
                    def.speedMin +
                    Math.random() *
                    (def.speedMax - def.speedMin);

                  vehicle.speed = vehicle.cruiseSpeed;
                  if (vehicle.driverType === 'reckless' && !def.heavy) {
                    vehicle.speed *= 1.18 + Math.random() * 0.16;
                  }
                }

                const lateral =
                  (vehicle.lane - 1.5) * LANE_WIDTH;

                const point = roadPoint(
                  vehicle.s,
                  lateral,
                  0
                );

                vehicle.root.visible = signedGapFromEgo > -EGO_VISIBLE_TRAFFIC_BEHIND && signedGapFromEgo < EGO_VISIBLE_TRAFFIC_AHEAD;
                vehicle.root.position.copy(point);
                vehicle.root.rotation.y = roadHeading(vehicle.s);

                const safetyZone = vehicle.root.userData.safetyZone;
                if (safetyZone) {
                  const activeColor = vehicle.targetLane !== undefined
                    ? 0xffd15c
                    : vehicle.driverType === 'reckless'
                      ? 0xff5b66
                      : TRAFFIC_DEFS[vehicle.key].heavy
                        ? 0xffb45c
                        : 0x4bc7ff;
                  safetyZone.material.color.set(activeColor);
                  safetyZone.material.opacity = vehicle.root.visible ? (vehicle.targetLane !== undefined ? 0.18 : 0.09) : 0;
                }

                const lights = vehicle.root.userData.indicatorLights;
                if (lights) {
                  const blinkOn = vehicle.indicatorTimer > 0
                    ? Math.floor(vehicle.indicatorTimer * 8) % 2 === 0
                    : false;
                  lights.left.visible = blinkOn && vehicle.indicator < 0;
                  lights.right.visible = blinkOn && vehicle.indicator > 0;
                }
              });

              // TRAFFIC-TO-TRAFFIC PHYSICAL SEPARATION -------------------------------
              // Keep every pair of vehicles inside its own safety envelope. If a lane
              // change would create a real overlap, cancel that lane change first; if
              // two vehicles are still too close longitudinally, separate the rear one
              // and bleed its speed rather than allowing a mesh-to-mesh collision.
              for (let i = 0; i < sim.vehicles.length; i++) {
                const a = sim.vehicles[i];
                const aLane = trafficLaneAtTime(a, 0);
                const aPoint = roadPoint(a.s, laneCenterLateral(aLane), 0.15);
                for (let j = i + 1; j < sim.vehicles.length; j++) {
                  const b = sim.vehicles[j];
                  const bLane = trafficLaneAtTime(b, 0);
                  const bPoint = roadPoint(b.s, laneCenterLateral(bLane), 0.15);
                  const clearance = (a.collisionRadius || 3.0) + (b.collisionRadius || 3.0) + 0.45;
                  if (aPoint.distanceTo(bPoint) >= clearance) continue;

                  // If the overlap is caused by a lane change, the entering vehicle gives way.
                  const aChanging = a.targetLane !== undefined;
                  const bChanging = b.targetLane !== undefined;
                  if (aChanging || bChanging) {
                    const yielding = aChanging ? a : bChanging ? b : null;
                    if (yielding) {
                      yielding.targetLane = undefined;
                      yielding.indicator = 0;
                      yielding.laneChangeTimer = 0;
                      yielding.laneChangeCooldown = Math.max(yielding.laneChangeCooldown, 1.2);
                    }
                  }

                  const signedAB = signedTrackGap(b.s, a.s);
                  const rear = signedAB > 0 ? a : b;
                  const front = signedAB > 0 ? b : a;
                  const requiredGap = Math.max(
                    8,
                    ((front.collisionRadius || 3) + (rear.collisionRadius || 3)) * 0.72
                  );
                  const physicalGap = Math.abs(signedTrackGap(front.s, rear.s));
                  if (physicalGap < requiredGap) {
                    rear.s = (front.s - requiredGap + TRACK_LENGTH) % TRACK_LENGTH;
                    rear.speed = Math.min(rear.speed, front.speed * 0.92);
                  }
                }
              }

              // PREDICTIVE EGO MOTION GUARD ------------------------------------------
              // Use the same physical road-space guard for the actual longitudinal
              // movement. This prevents mesh penetration without teleporting the Dodge
              // backward and without cancelling an already committed overtake.
              const requestedAdvance = Math.max(0, effectiveEgoSpeed) * dt;
              let safeAdvance = requestedAdvance;

              // During an active overtake, do NOT let the old lead vehicle's
              // longitudinal envelope drag the Dodge back into a slow-follow state.
              // Lane selection already checked the destination lane. Only perform a
              // hard longitudinal clamp when there is an immediate (< 4.5 m) conflict.
              if (sim.controllerMode !== 'OVERTAKE' || sim.waitingForTargetClear) {
                safeAdvance = constrainEgoAdvance(sim, effectiveEgoSpeed, dt);

                if (safeAdvance < 0.0005 && lead && sim.controllerMode !== 'OVERTAKE') {
                  const emergencyLane = chooseOvertakeLane();
                  if (emergencyLane !== null && emergencyLane !== currentLane) {
                    sim.overtakeFromLane = currentLane;
                    sim.targetLane = emergencyLane;
                    sim.overtakingId = lead.id;
                    sim.controllerMode = 'OVERTAKE';
                    sim.waitingForTargetClear = false;
                    updateUI('EMERGENCY OVERTAKE — EXECUTING', `LANE ${emergencyLane + 1}`);
                    safeAdvance = Math.min(requestedAdvance, 0.75);
                  }
                }
              } else {
                // During an active overtake, neighbouring-lane vehicles must not pull
                // the Dodge down to their speed. Only an actual same-corridor conflict
                // is allowed to clamp the current frame.
                for (const v of sim.vehicles) {
                  if (v.id === sim.overtakingId) continue;
                  const vS = (v.s + (v.speed / 3.6) * dt) % TRACK_LENGTH;
                  const vLane = trafficLaneAtTime(v, dt * 0.5);
                  const gap = signedTrackGap(vS, sim.egoS);
                  const lateral = Math.abs(vLane - sim.egoLane) * LANE_WIDTH;
                  if (gap > 0 && gap < 3.8 && lateral < 1.65) {
                    safeAdvance = Math.min(safeAdvance, Math.max(0.05, gap - 2.4));
                  }
                }
              }

              sim.egoS = (sim.egoS + safeAdvance) % TRACK_LENGTH;

              // EGO-TRAFFIC REPOSITIONING REMOVED -----------------------------------
              // Traffic vehicles are never repositioned relative to the Dodge here.
              // Each vehicle continues using its own lane, speed, and longitudinal
              // simulation. The ego advance guard above handles the Dodge's own
              // forward safety; traffic is not teleported/snapped around egoS.

            } else {
              // Advance roadside actors BEFORE the ego controller. This is important
              // for crossing hazards: the pedestrian/animal must enter the lane and
              // be detected before this frame's lane decision is made.
              updateRoadsideLife(roadsideLife, dt, sim);
              updateRoadsideHazardPriority(sim, roadsideLife);
              runScriptedTick(dt);
            }

            // Road-crossing perception has priority over scripted overtakes and
            // ordinary traffic decisions. It is reasserted every frame while a
            // pedestrian/animal is crossing so another scripted maneuver cannot
            // pull the Dodge back into the hazard.
            updateRoadsideHazardPriority(sim, roadsideLife);

            // Ego position.
            const egoLateral =
              (sim.egoLane - 1.5) *
              LANE_WIDTH;

            const egoPoint =
              roadPoint(
                sim.egoS,
                egoLateral,
                0
              );

            egoCar.position.copy(egoPoint);
            egoCar.rotation.y =
              roadHeading(sim.egoS);

            // Scripted-mode traffic rendering. Dynamic mode already renders traffic
            // inside its controller loop; the fixed demo needs this independent pass.
            if (SCRIPTED_SIMULATION) {
              sim.vehicles.forEach(vehicle => {
                if (vehicle.scriptInactive) {
                  vehicle.root.visible = false;
                  return;
                }
                const signedGapFromEgo = signedTrackGap(vehicle.s, sim.egoS);
                vehicle.root.visible = signedGapFromEgo > -EGO_VISIBLE_TRAFFIC_BEHIND && signedGapFromEgo < EGO_VISIBLE_TRAFFIC_AHEAD;
                const lateral = (vehicle.lane - 1.5) * LANE_WIDTH;
                const point = roadPoint(vehicle.s, lateral, 0);
                vehicle.root.position.copy(point);
                vehicle.root.rotation.y = roadHeading(vehicle.s);

                const safetyZone = vehicle.root.userData.safetyZone;
                if (safetyZone) {
                  const activeColor = vehicle.targetLane !== undefined
                    ? 0xffd15c
                    : TRAFFIC_DEFS[vehicle.key].heavy
                      ? 0xffb45c
                      : 0x4bc7ff;
                  safetyZone.material.color.set(activeColor);
                  safetyZone.material.opacity = vehicle.root.visible ? (vehicle.targetLane !== undefined ? 0.18 : 0.09) : 0;
                }

                const lights = vehicle.root.userData.indicatorLights;
                if (lights) {
                  const blinkOn = vehicle.indicatorTimer > 0
                    ? Math.floor(vehicle.indicatorTimer * 8) % 2 === 0
                    : false;
                  lights.left.visible = blinkOn && vehicle.indicator < 0;
                  lights.right.visible = blinkOn && vehicle.indicator > 0;
                }
              });
            }

            // Give the perception system the exact rendered ego position for its
            // roadside-target corridor calculations.
            sim.egoRoot = egoCar;

            // Roadside actors were already advanced before the ego controller so
            // crossing hazards can influence the current frame's path decision.
            // Update the perception links EVERY render frame using each target's
            // current world position. This keeps newly entering pedestrians/animals
            // eligible immediately and makes the blue line follow them continuously.
            updateEgoVehicleLinks(vehicleLinks, sim, roadsideLife, radar);
            updateDetectionBoxes(detectionBoxes, sim, roadsideLife, camera, mount);

            const egoSafetyZone = egoCar.userData.safetyZone;
            if (egoSafetyZone) {
              egoSafetyZone.material.color.set(0x31e6ff);
              egoSafetyZone.material.opacity = sim.controllerMode === 'OVERTAKE' ? 0.24 : 0.18;
            }

            // CAMERA: true chase-camera transform. It is parented to the ego car,
            // so the parent rotation/translation makes the camera move automatically.
            // Keep the camera orientation local so world-space lookAt does not fight
            // the ego-car rotation and make the view appear frozen.
            camera.position.set(0, 0, 0);
            camera.rotation.set(-0.115, 0, 0);
            camera.updateMatrixWorld();

            // POTHOLE IDENTIFICATION VISUALIZATION ----------------------------------
            // Highlight hazards in the Dodge's detection corridor. The active
            // pothole gets a stronger red ring so the audience can see exactly
            // which road defect the planner is reacting to.
            const potholeMarkers = potholeGroup?.userData?.detectionMarkers || {};
            for (const pothole of POTHOLES) {
              const marker = potholeMarkers[pothole.id];
              if (!marker) continue;

              const gap = signedTrackGap(pothole.s, sim.egoS);
              const detected = gap > POTHOLE_CLEAR_GAP && gap <= POTHOLE_DETECT_RANGE
                && Math.abs(pothole.lane - sim.egoLane) <= POTHOLE_LANE_BUFFER;
              const active = sim.potholeAvoidanceActive && sim.potholeId === pothole.id;

              marker.material.opacity = active ? 0.95 : detected ? 0.62 : 0.0;
              marker.material.color.set(active ? 0xff4b42 : 0xffc247);

              if (active) {
                const pulse = 0.98 + Math.sin(now * 0.012) * 0.10;
                marker.scale.setScalar(pulse);
              } else {
                marker.scale.setScalar(1.0);
              }
            }

            // RADAR: the three layers use one shared forward axis.
            // OUTER detects, MIDDLE predicts motion, INNER triggers the maneuver.
            radar.rotation.y = 0;

            const radarOrigin = new THREE.Vector3(0, 0, 0);
            const detected = [];

            sim.vehicles.forEach(vehicle => {
              if (vehicle.scriptInactive) return;
              const gap = (vehicle.s - sim.egoS + TRACK_LENGTH) % TRACK_LENGTH;
              if (gap > TRACK_LENGTH / 2 || gap > RADAR_OUTER_RANGE) return;

              const lateral = (vehicle.lane - 1.5) * LANE_WIDTH;
              const egoLateralNow = (sim.egoLane - 1.5) * LANE_WIDTH;

              // Outer layer sees a wider corridor; inner layers are evaluated from
              // the same detected object using range thresholds.
              if (Math.abs(lateral - egoLateralNow) > 8.5) return;

              const relativeSpeed = EGO_SPEED_KMH - vehicle.speed;
              const ttc = relativeSpeed > 0.5
                ? gap / (relativeSpeed / 3.6)
                : Infinity;

              let zone = 'OUTER';
              if (gap <= RADAR_INNER_RANGE) zone = 'INNER';
              else if (gap <= RADAR_MIDDLE_RANGE) zone = 'MIDDLE';

              detected.push({ vehicle, gap, relativeSpeed, ttc, zone });
            });

            detected.sort((a, b) => a.gap - b.gap);
            const nearest = detected.slice(0, 4);

            // Radar connectors and the green path do not need a 60 FPS refresh.
            // Updating them at ~30 FPS keeps the camera/vehicles smooth while
            // reducing CPU work from repeated allocations and buffer writes.
            if ((radarUpdateFrame++ % 3) === 0) {
              const markerList = radar.userData.detectionMarkers || [];
              const connectorAttr = radar.userData.targetLines.geometry.getAttribute('position');

              for (let i = 0; i < markerList.length; i++) {
                const marker = markerList[i];
                const item = nearest[i];

                if (!item) {
                  marker.visible = false;
                  continue;
                }

                const targetWorld = roadPoint(
                  item.vehicle.s,
                  (item.vehicle.lane - 1.5) * LANE_WIDTH,
                  1.25
                );

                marker.visible = true;
                marker.position.copy(targetWorld);
                marker.material.color.set(
                  item.zone === 'INNER' ? 0x48f0ff :
                    item.zone === 'MIDDLE' ? 0x249bff :
                      0x45cfff
                );
                marker.material.opacity =
                  item.zone === 'INNER' ? 1.0 :
                    item.zone === 'MIDDLE' ? 0.9 : 0.72;

                const localTarget = radar.worldToLocal(targetWorld);
                connectorAttr.setXYZ(i * 2, radarOrigin.x, radarOrigin.y, radarOrigin.z);
                connectorAttr.setXYZ(i * 2 + 1, localTarget.x, localTarget.y, localTarget.z);
              }

              for (let i = nearest.length; i < markerList.length; i++) {
                markerList[i].visible = false;
              }

              connectorAttr.needsUpdate = true;
              radar.userData.targetLines.geometry.setDrawRange(0, nearest.length * 2);
            }

            if ((pathUpdateFrame++ % 3) === 0) {
              const pathPoints = [];

              for (let i = 0; i <= 18; i++) {
                const lookAhead = EGO_CORRIDOR_START + i * (EGO_CORRIDOR_LENGTH / 18);
                const s = (sim.egoS + lookAhead) % TRACK_LENGTH;
                const t = clamp((lookAhead - EGO_CORRIDOR_START) / EGO_CORRIDOR_LENGTH, 0, 1);
                const lane = sim.egoLane + (sim.targetLane - sim.egoLane) * Math.min(1, t * 1.7);
                const lateral = (lane - 1.5) * LANE_WIDTH;
                pathPoints.push(roadPoint(s, lateral, 0.085));
              }

              updatePathRibbon(pathRibbon, pathPoints, EGO_CORRIDOR_WIDTH);
            }

          }
        } catch (error) {
          // Keep the animation loop alive if a transient traffic/model update ever
          // throws. A single bad vehicle must not freeze the entire demo.
          if (now - sim.lastErrorAt > 1500) {
            sim.lastErrorAt = now;
            console.warn('[NAVION] Recovered animation-loop error:', error);
          }
        }

        renderer.render(
          scene,
          camera
        );

        updatePlannerTelemetry(sim);
        updateSensorFeeds(sim);

        frame =
          requestAnimationFrame(
            animate
          );
      };

      frame =
        requestAnimationFrame(
          animate
        );

      const resize = () => {
        const w = mount.clientWidth;
        const h =
          Math.max(
            mount.clientHeight,
            1
          );

        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(
          w,
          h
        );
      };

      window.addEventListener(
        'resize',
        resize
      );

      resize();

      cleanup = () => {
        cancelAnimationFrame(frame);
        window.removeEventListener(
          'resize',
          resize
        );
        renderer.dispose();

        if (
          mount.contains(
            renderer.domElement
          )
        ) {
          mount.removeChild(
            renderer.domElement
          );
        }

        if (detectionBoxes?.overlay && mount.contains(detectionBoxes.overlay)) {
          mount.removeChild(detectionBoxes.overlay);
        }
      };

    };

    initialize().catch((error) => {
      console.error('[NAVION] Initialization failed:', error);
      setReady(false);
    });

    return () => {
      if (cleanup) cleanup();
    };
  }, []);

  const panelStyle = {
    background: 'linear-gradient(180deg, rgba(7,27,41,.98), rgba(5,18,29,.98))',
    border: '1px solid rgba(75,220,255,.18)',
    borderRadius: 9,
    padding: 11,
    boxSizing: 'border-box',
    overflow: 'hidden',
    boxShadow: 'inset 0 1px 0 rgba(255,255,255,.025), 0 8px 24px rgba(0,0,0,.18)'
  };
  const panelTitle = {
    color: '#dcecf5',
    fontSize: 7.5,
    fontWeight: 850,
    letterSpacing: .55,
    paddingBottom: 7,
    borderBottom: '1px solid rgba(75,220,255,.09)'
  };
  const smallKV = {
    display: 'flex',
    justifyContent: 'space-between',
    alignItems: 'center',
    padding: '7px 0',
    borderBottom: '1px solid rgba(75,220,255,.06)',
    fontSize: 7,
    color: '#718a98'
  };

  const dashboardCss = `
    /* FULL-VIEWPORT DASHBOARD RESET: use the entire browser viewport. */
    html,body,#root{width:100%;height:100%;margin:0;padding:0;overflow:hidden}
    body{display:block;min-width:0}
    *,*::before,*::after{box-sizing:border-box}
    .navion-ui{width:100%;height:100%;min-width:0;max-width:none;overflow:hidden;position:relative;background:#050b12;color:#dcecf5;font-family:Inter,Arial,sans-serif}
    .nav-head{height:64px;display:flex;align-items:center;justify-content:space-between;padding:0 20px;background:linear-gradient(180deg,#0a1b29,#07131e);border-bottom:1px solid rgba(79,214,255,.18);box-shadow:0 8px 28px rgba(0,0,0,.32);position:absolute;inset:0 0 auto;z-index:50}
    .brand{display:flex;align-items:center;gap:11px}.brand-logo{width:34px;height:34px;border-radius:9px;display:grid;place-items:center;color:#4bdcff;font-weight:900;font-size:18px;background:#092a3d;border:1px solid rgba(75,220,255,.62);box-shadow:0 0 18px rgba(75,220,255,.1)}
    .brand-title{font-size:18px;font-weight:900;letter-spacing:1.6px}.brand-sub{font-size:7px;color:#6f8796;letter-spacing:.9px;margin-top:2px}
    .nav-tabs{display:flex;align-items:stretch;height:100%;gap:3px}.nav-tab{appearance:none;border:0;background:transparent;display:flex;align-items:center;padding:0 13px;color:#718b9b;font:inherit;font-size:8px;letter-spacing:.35px;position:relative;cursor:pointer;white-space:nowrap}.nav-tab:hover{color:#cfeaf5;background:rgba(75,220,255,.05)}.nav-tab:focus-visible{outline:1px solid rgba(75,220,255,.65);outline-offset:-3px}.nav-tab.active{color:#4bdcff}.nav-tab.active:after{content:"";position:absolute;left:12px;right:12px;bottom:0;height:2px;background:#4bdcff;box-shadow:0 0 10px rgba(75,220,255,.55)}.nav-focus{border-color:rgba(75,220,255,.62)!important;box-shadow:0 0 0 1px rgba(75,220,255,.20),0 0 24px rgba(75,220,255,.13),inset 0 0 22px rgba(75,220,255,.035)!important}
    .head-right{display:flex;align-items:center;gap:7px}.chip{padding:6px 8px;border-radius:5px;background:#081a28;border:1px solid rgba(75,220,255,.16);font-size:6.5px;color:#7894a4}.chip.live{color:#38e58d;border-color:rgba(56,229,141,.28);background:#09271e}.chip.warn{color:#ffd35a;border-color:rgba(255,211,90,.24)}
    .dashboard-grid{position:absolute;left:10px;right:10px;top:76px;bottom:10px;display:grid;grid-template-columns:minmax(0,1.84fr) minmax(380px,.76fr);grid-template-rows:minmax(0,1fr) 315px;gap:12px}
    .card{background:linear-gradient(180deg,rgba(8,25,38,.98),rgba(5,16,26,.98));border:1px solid rgba(75,220,255,.14);border-radius:10px;box-shadow:inset 0 1px 0 rgba(255,255,255,.025),0 10px 28px rgba(0,0,0,.2);overflow:hidden}.card.accent{border-color:rgba(75,220,255,.24)}
    .card-head{height:36px;display:flex;align-items:center;justify-content:space-between;padding:0 11px;border-bottom:1px solid rgba(75,220,255,.09);background:linear-gradient(180deg,rgba(12,35,51,.72),rgba(8,23,35,.42));font-size:7.5px;font-weight:800;letter-spacing:.55px}.muted{color:#6b8493;font-size:6.5px;font-weight:600;letter-spacing:.2px}.cyan{color:#4bdcff}.green{color:#38e58d}.yellow{color:#ffd35a}.red{color:#ff6b61}
    .sim-card{position:relative;min-height:0}.sim-mount{position:absolute;inset:36px 0 0 0}.sim-overlay{position:absolute;z-index:8;left:10px;top:46px;display:flex;gap:5px}.sim-pill{padding:6px 8px;border-radius:5px;background:rgba(4,14,22,.86);border:1px solid rgba(75,220,255,.18);font-size:6.5px;color:#bdd0da}.sim-pill.live{color:#38e58d;border-color:rgba(56,229,141,.24)}.sim-corner{position:absolute;z-index:8;right:10px;top:46px;padding:6px 8px;border-radius:5px;background:rgba(4,14,22,.82);border:1px solid rgba(75,220,255,.14);font-size:6px;color:#718997}.sim-bottom{position:absolute;z-index:8;left:10px;right:10px;bottom:9px;display:flex;justify-content:space-between;align-items:center;pointer-events:none}.sim-status{padding:5px 8px;border-radius:5px;background:rgba(4,14,22,.84);border:1px solid rgba(56,229,141,.2);font-size:6px;color:#38e58d}.sim-legend{display:flex;gap:6px}.legend{padding:4px 6px;border-radius:4px;background:rgba(4,14,22,.78);font-size:5.5px;border:1px solid rgba(255,255,255,.06)}
    .right-stack{display:grid;grid-template-rows:1.02fr .88fr 1fr;gap:10px;min-height:0}.decision-body{padding:11px}.decision-main{display:grid;grid-template-columns:1fr 108px;gap:10px}.eyebrow{font-size:6px;color:#657f8e;letter-spacing:.75px}.decision-action{font-size:17px;font-weight:900;color:#38e58d;line-height:1.05;margin-top:5px}.decision-reason{font-size:7px;color:#7f98a6;margin-top:7px}.decision-reason strong{color:#dcecf5}.lane-mini{height:94px;border-radius:8px;background:#06141f;border:1px solid rgba(75,220,255,.15);position:relative;overflow:hidden}.lane-mini:before,.lane-mini:after{content:"";position:absolute;top:-8px;bottom:-8px;width:1px;background:rgba(255,255,255,.18)}.lane-mini:before{left:33%}.lane-mini:after{right:33%}.lane-arrow{position:absolute;left:50%;bottom:10px;width:3px;height:48px;background:#38e58d;transform:translateX(-50%);box-shadow:0 0 10px rgba(56,229,141,.48)}.lane-arrow:before{content:"";position:absolute;top:-2px;left:50%;border-left:6px solid transparent;border-right:6px solid transparent;border-bottom:10px solid #38e58d;transform:translateX(-50%)}.ego-mini{position:absolute;left:50%;bottom:8px;width:14px;height:26px;border-radius:4px;background:#d71920;transform:translateX(-50%);box-shadow:0 0 14px rgba(215,25,32,.34)}
    .decision-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:9px}.micro-card{padding:8px;border-radius:6px;background:#06141f;border:1px solid rgba(75,220,255,.09)}.micro-card .k{font-size:5.5px;color:#617b89}.micro-card .v{font-size:9px;color:#dcecf5;font-weight:800;margin-top:4px}.micro-card .v.cyan{color:#4bdcff}.micro-card .v.green{color:#38e58d}
    .objects-body{padding:9px}.object-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px}.obj-card{padding:8px;border-radius:6px;background:#06141f;border:1px solid rgba(75,220,255,.1)}.obj-label{font-size:5.5px;color:#6b8492;letter-spacing:.45px}.obj-value{font-size:17px;font-weight:900;margin-top:4px}.obj-sub{font-size:5.5px;margin-top:2px}.obj-card.vehicle .obj-value{color:#4bdcff}.obj-card.person{border-color:rgba(255,211,90,.16)}.obj-card.person .obj-value{color:#ffd35a}.obj-card.pothole{border-color:rgba(255,107,97,.16)}.obj-card.pothole .obj-value{color:#ff6b61}.object-foot{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:7px}.object-foot .mini-row{padding:6px 7px;background:#081a27;border-radius:5px;font-size:6px;color:#6f8795}.object-foot b{float:right;color:#dcecf5}
    .path-body{padding:9px}.path-layout{display:grid;grid-template-columns:1fr 104px;gap:8px}.kv-list{display:grid;gap:2px}.kv{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid rgba(75,220,255,.055);font-size:7px;color:#66808e}.kv strong{color:#dcecf5;font-size:7px}.corridor-mini{height:106px;border-radius:8px;border:1px solid rgba(75,220,255,.14);background:#06131f;position:relative;overflow:hidden}.corridor-road{position:absolute;left:25%;right:25%;top:-5px;bottom:-5px;border-left:1px dashed rgba(255,255,255,.22);border-right:1px dashed rgba(255,255,255,.22);background:linear-gradient(90deg,transparent 49%,rgba(255,255,255,.18) 50%,transparent 51%)}.corridor-path{position:absolute;left:50%;top:20px;bottom:17px;width:4px;background:#38e58d;transform:translateX(-50%);box-shadow:0 0 12px rgba(56,229,141,.4)}.corridor-car{position:absolute;left:50%;bottom:12px;width:13px;height:23px;border-radius:4px;background:#d71920;transform:translateX(-50%)}.corridor-text{position:absolute;top:6px;left:6px;font-size:5.5px;color:#607987}
    .bottom-grid{grid-column:1/3;display:grid;grid-template-columns:1.05fr 1.25fr 1.5fr 1.7fr 1.05fr;gap:10px;min-height:0}.bottom-card{min-width:0}.metric-big{padding:11px}.speed-line{display:flex;align-items:center;gap:10px}.speed-gauge{width:78px;height:78px;border-radius:50%;border:7px solid #153746;border-top-color:#4bdcff;border-right-color:#38e58d;display:grid;place-items:center;flex:none}.speed-num{font-size:21px;font-weight:900;text-align:center}.speed-unit{font-size:5.5px;color:#6d8796}.vehicle-name{font-size:8px;font-weight:800;margin-top:2px}.vehicle-meta{font-size:6px;color:#657f8d;margin-top:4px}.lane-bars{display:grid;grid-template-columns:repeat(4,1fr);gap:3px;margin-top:12px}.lane-bar{height:5px;border-radius:3px;background:#102735}.lane-bar.active{background:#4bdcff;box-shadow:0 0 8px rgba(75,220,255,.34)}
    .log-body{padding:7px}.log-item{display:grid;grid-template-columns:44px 48px 1fr;gap:6px;align-items:center;padding:5px 0;border-bottom:1px solid rgba(75,220,255,.055);font-size:5.9px;color:#8ba1ae}.log-time{color:#4f6977}.log-type{font-size:5.3px;font-weight:800}.log-msg{color:#b9cbd4;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    .sensor-grid{display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:7px}.sensor-tile{min-width:0;padding:5px;border:1px solid rgba(75,220,255,.1);border-radius:6px;background:#06131f}.sensor-name{display:flex;justify-content:space-between;font-size:5.7px;font-weight:800;color:#4bdcff;margin-bottom:4px}.sensor-live{color:#38e58d;font-weight:700}.sensor-tile canvas{display:block;width:100%;height:64px;border-radius:4px;background:#020b12}.sensor-sub{font-size:5px;color:#5f7886;margin-top:3px}.fusion-row{display:flex;justify-content:space-between;padding:0 7px 6px;font-size:5.7px;color:#657f8e}.fusion-row b{color:#38e58d}.fusion-track{height:4px;border-radius:4px;background:#102735;margin:0 7px 7px}.fusion-fill{height:100%;background:#38e58d;border-radius:4px;box-shadow:0 0 8px rgba(56,229,141,.24)}
    .status-body{padding:8px}.status-row{display:flex;justify-content:space-between;align-items:center;padding:5px 0;border-bottom:1px solid rgba(75,220,255,.055);font-size:5.8px}.status-row span{color:#6f8796}.status-row b{color:#38e58d;font-size:5.5px}.status-box{margin-top:7px;padding:7px;border-radius:6px;background:#06131f;border:1px solid rgba(75,220,255,.09)}.status-box .big{font-size:9px;font-weight:900;color:#38e58d}.status-box .small{font-size:5.3px;color:#617a89;margin-top:2px}
    .loading{position:absolute;z-index:100;left:50%;top:50%;transform:translate(-50%,-50%);padding:12px 18px;border-radius:7px;background:rgba(3,10,15,.95);border:1px solid rgba(75,220,255,.38);color:#fff;font:9px monospace;box-shadow:0 12px 35px rgba(0,0,0,.45)}
    /* Large, readable type + responsive full-width layout. */
    .brand-title{font-size:24px}.brand-sub{font-size:9px}
    .nav-tabs{gap:7px}.nav-tab{padding:0 14px;font-size:10px}.head-right{gap:9px}.chip{padding:8px 10px;font-size:8.5px}
    .card-head{height:40px;padding:0 13px;font-size:10px}.muted{font-size:8px}
    .sim-overlay{top:50px;gap:7px}.sim-pill{padding:7px 10px;font-size:9px}.sim-corner{top:50px;padding:7px 10px;font-size:8px}.sim-status{padding:7px 10px;font-size:8px}.legend{padding:5px 7px;font-size:7px}
    .decision-body{padding:13px}.eyebrow{font-size:8px}.decision-action{font-size:22px}.decision-reason{font-size:10px}.micro-card .k{font-size:7.5px}.micro-card .v{font-size:12px}
    .objects-body{padding:11px}.obj-card{padding:10px}.obj-label{font-size:8px}.obj-value{font-size:24px}.obj-sub{font-size:8px}.object-foot .mini-row{padding:8px 9px;font-size:8px}
    .path-body{padding:12px}.kv{padding:8px 0;font-size:9px}.corridor-text{font-size:7px}
    .bottom-grid{grid-template-columns:1.05fr 1.25fr 1.55fr 1.65fr 1.1fr;gap:12px}.metric-big{padding:14px}.speed-gauge{width:92px;height:92px}.speed-num{font-size:25px}.speed-unit{font-size:7px}.vehicle-name{font-size:10px}.vehicle-meta{font-size:8px}.lane-bar{height:6px}
    .log-body{padding:10px}.log-item{grid-template-columns:54px 60px 1fr;gap:8px;padding:7px 0;font-size:8px}.log-type{font-size:7px}
    .sensor-grid{gap:8px;padding:9px}.sensor-tile{padding:7px}.sensor-name{font-size:8px;margin-bottom:5px}.sensor-tile canvas{height:76px}.sensor-sub{font-size:7px}.fusion-row{padding:0 9px 8px;font-size:8px}.fusion-track{height:5px;margin:0 9px 9px}
    .status-body{padding:10px}.status-row{padding:7px 0;font-size:8px}.status-row b{font-size:7px}.status-box{padding:9px}.status-box .big{font-size:12px}.status-box .small{font-size:7px}
    /* NAVIGATION PAGES: keep the MAIN SIMULATION WINDOW exactly on the same
       dashboard grid cell and dimensions used by the SIMULATION tab. Only the
       surrounding information panels are swapped/animated. */
    @keyframes navPanelIn{from{opacity:0;transform:translate3d(24px,0,0)}to{opacity:1;transform:translate3d(0,0,0)}}

    /* Never move, resize, reframe, or remount the Three.js simulation when a tab changes. */
    .page-sensor-view .sim-card,
    .page-ai-decision .sim-card,
    .page-route-map .sim-card,
    .page-analytics .sim-card{
      position:relative!important;
      left:auto!important;
      top:auto!important;
      width:auto!important;
      height:auto!important;
      min-height:0!important;
      max-height:none!important;
      grid-column:1!important;
      grid-row:1!important;
      grid-area:auto!important;
      z-index:auto!important;
      opacity:1!important;
      pointer-events:auto!important;
    }
    .page-sensor-view .sim-mount,
    .page-ai-decision .sim-mount,
    .page-route-map .sim-mount,
    .page-analytics .sim-mount{inset:36px 0 0 0!important}

    /* On information tabs, the grid itself stays identical to the SIMULATION tab. */
    .page-sensor-view .right-stack,
    .page-ai-decision .right-stack,
    .page-route-map .right-stack,
    .page-analytics .right-stack,
    .page-sensor-view .bottom-grid,
    .page-ai-decision .bottom-grid,
    .page-route-map .bottom-grid,
    .page-analytics .bottom-grid{display:contents!important}

    /* Hide all surrounding panels first; then reveal only the panel for the active tab. */
    .page-sensor-view .right-stack > .card,
    .page-ai-decision .right-stack > .card,
    .page-route-map .right-stack > .card,
    .page-analytics .right-stack > .card,
    .page-sensor-view .bottom-grid > .bottom-card,
    .page-ai-decision .bottom-grid > .bottom-card,
    .page-route-map .bottom-grid > .bottom-card,
    .page-analytics .bottom-grid > .bottom-card{display:none!important}

    /* SENSOR VIEW: replace the right-side panel with the live sensor-fusion view. */
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3){
      display:block!important;
      grid-column:2;
      grid-row:1;
      height:100%;
      min-height:0;
      animation:navPanelIn .28s cubic-bezier(.2,.75,.25,1) both;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-grid{
      height:calc(100% - 76px);
      grid-template-columns:repeat(2,minmax(0,1fr));
      grid-template-rows:repeat(2,minmax(0,1fr));
      gap:10px;
      padding:10px;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile{
      display:flex;
      flex-direction:column;
      min-height:0;
      padding:9px;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile canvas{
      flex:1 1 auto;
      width:100%;
      height:auto;
      min-height:0;
      aspect-ratio:1.45 / 1;
    }

    /* SENSOR VIEW: keep the simulation in its normal left cell. Put SENSOR FUSION
       directly underneath it, while retaining the existing three live panels stacked
       on the right. This changes presentation only; the simulation mount is untouched. */
    .page-sensor-view .right-stack{
      display:grid!important;
      grid-column:2!important;
      grid-row:1!important;
      grid-template-rows:1.02fr .88fr 1fr!important;
      gap:10px!important;
      min-height:0!important;
    }
    .page-sensor-view .right-stack > .card{
      display:block!important;
      grid-column:auto!important;
      grid-row:auto!important;
      height:auto!important;
      min-height:0!important;
      animation:navPanelIn .25s cubic-bezier(.2,.75,.25,1) both;
    }
    .page-sensor-view .bottom-grid > .bottom-card{display:none!important}
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3){
      display:block!important;
      grid-column:1!important;
      grid-row:2!important;
      height:100%!important;
      min-height:0!important;
      animation:navPanelIn .28s cubic-bezier(.2,.75,.25,1) both;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-grid{
      height:calc(100% - 76px);
    }

    /* SENSOR VIEW FINAL LAYOUT: keep the exact simulation viewport in place.
       Only the sensor presentation changes: CAMERA below the simulation, while
       LiDAR / RADAR / ULTRASONIC occupy the three stacked panels on the right.
       The AI / Perception / Route panels are not shown on SENSOR VIEW. */
    .page-sensor-view .right-stack{
      display:none!important;
    }
    .page-sensor-view .bottom-grid{
      display:contents!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:not(:nth-child(3)){
      display:none!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3){
      display:contents!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) > .card-head,
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) > .fusion-row,
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) > .fusion-track{
      display:none!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-grid{
      display:contents!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile{
      position:absolute!important;
      z-index:18;
      margin:0!important;
      background:linear-gradient(180deg,rgba(8,25,38,.98),rgba(5,16,26,.98))!important;
      border:1px solid rgba(75,220,255,.14)!important;
      border-radius:10px!important;
      box-shadow:inset 0 1px 0 rgba(255,255,255,.025),0 10px 28px rgba(0,0,0,.2)!important;
      overflow:hidden!important;
      min-width:0!important;
      animation:navPanelIn .28s cubic-bezier(.2,.75,.25,1) both;
    }
    /* Camera occupies the original bottom-left dashboard cell. */
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile:nth-child(1){
      left:10px!important;
      top:calc(100vh - 325px)!important;
      width:calc((100vw - 32px) * .7076923)!important;
      height:315px!important;
      padding:12px!important;
    }
    /* Three remaining live sensors replace the right-side AI/perception/path panels. */
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile:nth-child(2),
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile:nth-child(3),
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile:nth-child(4){
      right:10px!important;
      width:calc((100vw - 32px) * .2923077)!important;
      height:calc((100vh - 433px) / 3)!important;
      padding:10px!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile:nth-child(2){top:76px!important;}
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile:nth-child(3){top:calc(76px + ((100vh - 433px) / 3) + 10px)!important;}
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile:nth-child(4){top:calc(76px + (((100vh - 433px) / 3) * 2) + 20px)!important;}
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile canvas{
      width:100%!important;
      height:calc(100% - 42px)!important;
      min-height:0!important;
      aspect-ratio:auto!important;
      flex:none!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-tile:first-child canvas{
      height:calc(100% - 48px)!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-name{
      font-size:10px!important;
      margin-bottom:7px!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3) .sensor-sub{
      font-size:7px!important;
      margin-top:5px!important;
    }

    /* AI DECISION: only the live planner panel changes. */
    .page-ai-decision .right-stack > .card:first-child{
      display:block!important;
      grid-column:2;
      grid-row:1;
      height:100%;
      min-height:0;
      animation:navPanelIn .28s cubic-bezier(.2,.75,.25,1) both;
    }

    /* ROUTE MAP: only the live path-planning panel changes. */
    .page-route-map .right-stack > .card:nth-child(3){
      display:block!important;
      grid-column:2;
      grid-row:1;
      height:100%;
      min-height:0;
      animation:navPanelIn .28s cubic-bezier(.2,.75,.25,1) both;
    }

    /* ANALYTICS: only the live event stream changes. */
    .page-analytics .bottom-grid > .bottom-card:nth-child(2){
      display:block!important;
      grid-column:2;
      grid-row:1;
      height:100%;
      min-height:0;
      animation:navPanelIn .28s cubic-bezier(.2,.75,.25,1) both;
    }
    .page-analytics .bottom-grid > .bottom-card:nth-child(2) .log-body{padding:18px 20px}
    .page-analytics .bottom-grid > .bottom-card:nth-child(2) .log-item{grid-template-columns:58px 70px 1fr;gap:10px;padding:10px 0;font-size:8px}

    @media(max-width:1200px){.dashboard-grid{grid-template-columns:minmax(0,1.7fr) minmax(330px,.8fr);grid-template-rows:minmax(0,1fr) 290px;gap:9px}.bottom-grid{grid-template-columns:repeat(5,minmax(0,1fr));gap:9px}.nav-tabs{gap:2px}.nav-tab{padding:0 8px;font-size:9px}.chip{padding:6px 8px;font-size:7.5px}}
    @media(max-width:980px){.dashboard-grid{grid-template-columns:1fr;grid-template-rows:minmax(430px,1fr) auto auto}.right-stack{grid-template-rows:auto auto auto}.bottom-grid{grid-column:1;grid-template-columns:repeat(2,minmax(0,1fr));grid-auto-rows:minmax(160px,auto);overflow:auto}.nav-tabs{display:none}.head-right .chip:nth-last-child(-n+2){display:none}}
    /* SENSOR VIEW REFINEMENT: preserve the main simulation cell exactly; use the existing right stack for three live sensors and the left-bottom cell for live telemetry. */
    .sensor-alt-panel,.panel-head-sensor,.sensor-data-alt-panel{display:none}
    .sensor-right-card .sensor-state{font-size:6px}
    .page-sensor-view .right-stack{
      display:grid!important;
      grid-column:2!important;
      grid-row:1!important;
      grid-template-rows:1fr 1fr 1fr!important;
      gap:10px!important;
      min-height:0!important;
    }
    .page-sensor-view .right-stack > .card{
      display:block!important;
      grid-column:auto!important;
      grid-row:auto!important;
      min-height:0!important;
      height:auto!important;
      animation:navPanelIn .24s ease both;
    }
    .page-sensor-view .right-stack .panel-normal,
    .page-sensor-view .right-stack .panel-head-normal{display:none!important}
    .page-sensor-view .right-stack .panel-head-sensor,
    .page-sensor-view .right-stack .sensor-alt-panel{display:block!important}
    .page-sensor-view .right-stack .sensor-alt-panel{height:calc(100% - 36px);padding:10px}
    .page-sensor-view .right-stack .sensor-alt-top{display:flex;align-items:center;gap:7px;font-size:7px;color:#7c99a8;text-transform:uppercase;letter-spacing:.45px;margin-bottom:7px}
    .page-sensor-view .right-stack .sensor-alt-top b{margin-left:auto;color:#4bdcff;font-size:12px}
    .page-sensor-view .right-stack .sensor-alt-panel canvas{display:block;width:100%;height:calc(100% - 62px);min-height:70px;border-radius:6px;background:#020b12}
    .page-sensor-view .right-stack .sensor-alt-foot{display:flex;justify-content:space-between;align-items:center;margin-top:6px;font-size:6.5px;color:#607987;text-transform:uppercase}
    .page-sensor-view .right-stack .sensor-alt-foot b{color:#dcecf5;font-size:8px}
    .page-sensor-view .right-stack .sensor-data-row{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:7px}
    .page-sensor-view .right-stack .sensor-data-row>div{padding:6px 7px;border-radius:5px;background:#06141f;border:1px solid rgba(75,220,255,.09);display:flex;justify-content:space-between;gap:6px;align-items:center}
    .page-sensor-view .right-stack .sensor-data-row span{font-size:5.5px;color:#647d8b}
    .page-sensor-view .right-stack .sensor-data-row b{font-size:7px;color:#dcecf5}
    .page-sensor-view .bottom-grid{
      display:grid!important;
      grid-column:1!important;
      grid-row:2!important;
      grid-template-columns:1fr!important;
      gap:12px!important;
      min-height:0!important;
    }
    .page-sensor-view .bottom-grid > .bottom-card:not(:nth-child(3)){display:none!important}
    .page-sensor-view .bottom-grid > .bottom-card:nth-child(3){display:block!important;grid-column:1!important;grid-row:1!important;height:100%!important;min-height:0!important;animation:navPanelIn .28s cubic-bezier(.2,.75,.25,1) both}
    .page-sensor-view .sensor-normal-panel{display:none!important}
    .page-sensor-view .sensor-data-alt-panel{display:block!important;height:100%}
    .page-sensor-view .telemetry-grid{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:8px;padding:10px}
    .page-sensor-view .telemetry-card{padding:10px;border-radius:7px;background:#06141f;border:1px solid rgba(75,220,255,.1);min-width:0}
    .page-sensor-view .telemetry-card span{display:block;font-size:6px;color:#6b8492;letter-spacing:.45px;text-transform:uppercase}
    .page-sensor-view .telemetry-card b{display:block;margin-top:5px;color:#dcecf5;font-size:16px;font-weight:900;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
    .page-sensor-view .telemetry-card small{display:block;margin-top:3px;font-size:5.5px;color:#38e58d;text-transform:uppercase}
    .page-sensor-view .telemetry-footer{display:grid;grid-template-columns:repeat(6,minmax(0,1fr));gap:7px;padding:0 10px 10px}
    .page-sensor-view .telemetry-footer>div{padding:7px;border-radius:6px;background:#081a27;border:1px solid rgba(75,220,255,.07);display:flex;flex-direction:column;gap:4px;min-width:0}
    .page-sensor-view .telemetry-footer span{font-size:5.5px;color:#607987;text-transform:uppercase}
    .page-sensor-view .telemetry-footer b{font-size:8px;color:#dcecf5;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}

  `;

  return (
    <div className={`navion-ui page-${activeNav.toLowerCase().replace(/\s+/g, "-")}`}>
      <style>{dashboardCss}</style>

      <header className="nav-head">
        <div className="brand">
          <div className="brand-logo">N</div>
          <div><div className="brand-title">NAVION</div><div className="brand-sub">ADAPTIVE PATH PLANNING FOR AUTONOMOUS VEHICLES</div></div>
        </div>
        <nav className="nav-tabs" aria-label="Dashboard sections">
          {['SIMULATION', 'SENSOR VIEW', 'AI DECISION', 'ROUTE MAP', 'ANALYTICS'].map(section => (
            <button
              key={section}
              type="button"
              className={`nav-tab ${activeNav === section ? 'active' : ''}`}
              onClick={() => handleNavClick(section)}
              aria-current={activeNav === section ? 'page' : undefined}
            >
              {section}
            </button>
          ))}
        </nav>
        <div className="head-right"><div className="chip live">● LIVE</div><div className="chip">150 KM/H</div><div className="chip">4-LANE</div><div className="chip">AI ONLINE</div></div>
      </header>

      <main className="dashboard-grid">
        <section className={`card sim-card accent ${activeNav === 'SIMULATION' ? 'nav-focus' : ''}`}>
          <div className="card-head"><span>◈ LIVE SIMULATION / EGO VIEW</span><span className="muted"><span className="green">●</span> 3D ENGINE • 60 FPS</span></div>
          <div ref={mountRef} className="sim-mount" />
          <div className="sim-overlay"><span className="sim-pill">☀ DAY</span><span className="sim-pill">150 KM/H</span><span className="sim-pill live">● AUTONOMOUS</span></div>
          <div className="sim-corner">CAMERA • LIVE &nbsp;|&nbsp; PERCEPTION • ONLINE</div>
          <div className="sim-bottom"><span className="sim-status">● PATH {phase || 'CRUISING'} &nbsp;•&nbsp; TARGET {target || 'CLEAR'}</span><span className="sim-legend"><span className="legend cyan">VEHICLES</span><span className="legend yellow">PEDESTRIANS</span><span className="legend red">POTHOLE</span></span></div>
        </section>

        <aside className="right-stack">
          <section className={`card sensor-right-card ${activeNav === 'AI DECISION' ? 'nav-focus' : ''}`}>
            <div className="card-head">
              <span className="panel-head-normal">◉ AI DECISION</span>
              <span className="panel-head-sensor cyan">◎ LiDAR / 3D POINT CLOUD</span>
              <span className="muted panel-head-normal">LIVE PLANNER</span>
              <span className="green panel-head-sensor sensor-state">● LIVE</span>
            </div>
            <div className="panel-normal">
              <div className="decision-body"><div className="decision-main"><div><div className="eyebrow">CURRENT ACTION</div><div data-live="live-phase" className="decision-action">{phase || 'CRUISING'}</div><div className="decision-reason">Target <strong data-live="live-target">{target || 'CLEAR'}</strong></div><div className="decision-reason">Controller <strong data-live="live-controller">{simRef.current.controllerMode || 'CRUISE'}</strong></div></div><div className="lane-mini"><span className="eyebrow" style={{ position: 'absolute', left: 7, top: 6 }}>PLANNED PATH</span><div className="lane-arrow"></div><div className="ego-mini"></div></div></div><div className="decision-grid"><div className="micro-card"><div className="k">RADAR GAP</div><div data-live="live-radar-gap" className="v cyan">CLEAR</div></div><div className="micro-card"><div className="k">TARGET LANE</div><div data-live="live-target-lane" className="v cyan">LANE {Math.round(simRef.current.targetLane) + 1}</div></div></div></div>
            </div>
            <div className="sensor-alt-panel">
              <div className="sensor-alt-top"><span>LIVE POINT CLOUD</span><b data-live="live-lidar-count">0</b><span className="muted">TARGETS • 100M</span></div>
              <canvas id="navion-sensor-lidar"></canvas>
              <div className="sensor-alt-foot"><span>OBJECTS TRACKED</span><b data-live="live-lidar-count">0</b></div>
            </div>
          </section>

          <section className={`card sensor-right-card ${activeNav === 'SENSOR VIEW' ? 'nav-focus' : ''}`}>
            <div className="card-head">
              <span className="panel-head-normal">◎ PERCEPTION</span>
              <span className="panel-head-sensor cyan">◎ RADAR / FORWARD RANGE</span>
              <span className="muted panel-head-normal">3D FUSION</span>
              <span className="green panel-head-sensor sensor-state">● LIVE</span>
            </div>
            <div className="panel-normal">
              <div className="objects-body"><div className="object-grid"><div className="obj-card vehicle"><div className="obj-label">VEHICLES</div><div data-live="live-vehicle-count" className="obj-value">0</div><div className="obj-sub green">● TRACKING</div></div><div className="obj-card person"><div className="obj-label">PEDESTRIAN</div><div data-live="live-person-count" className="obj-value">0</div><div className="obj-sub yellow">● TRACKING</div></div><div className="obj-card pothole"><div className="obj-label">POTHOLES</div><div data-live="live-pothole-count" className="obj-value">0</div><div className="obj-sub red">● HAZARD</div></div></div><div className="object-foot"><div className="mini-row">OBJECTS <b data-live="live-object-count">0</b></div><div className="mini-row">RANGE <b data-live="live-radar-gap">CLEAR</b></div></div></div>
            </div>
            <div className="sensor-alt-panel">
              <div className="sensor-alt-top"><span>FORWARD TARGETS</span><b data-live="live-radar-count">0</b><span className="muted">OBJECTS • 120M</span></div>
              <canvas id="navion-sensor-radar"></canvas>
              <div className="sensor-data-row"><div><span>NEAREST</span><b data-live="live-radar-gap">CLEAR</b></div><div><span>ZONE</span><b data-live="live-radar-zone">NONE</b></div><div><span>TTC</span><b data-live="live-ttc">—</b></div></div>
            </div>
          </section>

          <section className={`card sensor-right-card ${activeNav === 'ROUTE MAP' ? 'nav-focus' : ''}`}>
            <div className="card-head">
              <span className="panel-head-normal">⌁ PATH PLANNING / SAFETY CORRIDOR</span>
              <span className="panel-head-sensor cyan">◎ ULTRASONIC / PROXIMITY</span>
              <span className="green panel-head-normal" style={{ fontSize: 6 }}>ACTIVE</span>
              <span className="green panel-head-sensor sensor-state">● LIVE</span>
            </div>
            <div className="panel-normal">
              <div className="path-body"><div className="path-layout"><div className="kv-list"><div className="kv"><span>CURRENT LANE</span><strong data-live="live-ego-lane">LANE {Math.round(simRef.current.egoLane) + 1}</strong></div><div className="kv"><span>TARGET LANE</span><strong data-live="live-target-lane">LANE {Math.round(simRef.current.targetLane) + 1}</strong></div><div className="kv"><span>RADAR ZONE</span><strong data-live="live-radar-zone">{simRef.current.radarZone || 'NONE'}</strong></div><div className="kv"><span>CONTROLLER</span><strong data-live="live-controller">{simRef.current.controllerMode || 'CRUISE'}</strong></div><div className="kv"><span>PREDICTED LANE</span><strong data-live="live-predicted-lane">LANE {Number.isFinite(simRef.current.predictedOvertakeLane) ? Math.round(simRef.current.predictedOvertakeLane) + 1 : '—'}</strong></div></div><div className="corridor-mini"><span className="corridor-text">PLANNER CORRIDOR</span><div className="corridor-road"></div><div className="corridor-path" style={{ left: `${((Math.round(simRef.current.targetLane) + 0.5) / 4) * 100}%` }}></div><div className="corridor-car" style={{ left: `${((Math.round(simRef.current.egoLane) + 0.5) / 4) * 100}%` }}></div></div></div></div>
            </div>
            <div className="sensor-alt-panel">
              <div className="sensor-alt-top"><span>NEAR-FIELD TARGETS</span><b data-live="live-ultra-count">0</b><span className="muted">OBJECTS • 0–18M</span></div>
              <canvas id="navion-sensor-ultrasonic"></canvas>
              <div className="sensor-alt-foot"><span>CLOSEST OBJECT</span><b data-live="live-ultra-gap">CLEAR</b></div>
            </div>
          </section>
        </aside>

        <section className="bottom-grid">
          <section className="card bottom-card"><div className="card-head"><span>◉ VEHICLE TELEMETRY</span><span className="muted">EGO</span></div><div className="metric-big"><div className="speed-line"><div className="speed-gauge"><div><div className="speed-num" data-live="live-ego-speed">150</div><div className="speed-unit">KM/H</div></div></div><div><div className="muted">EGO VEHICLE</div><div className="vehicle-name">DODGE CHALLENGER</div><div className="green" style={{ fontSize: 6, marginTop: 7 }}>● AUTONOMOUS MODE</div></div></div><div className="lane-bars"><span className="lane-bar"></span><span className="lane-bar active"></span><span className="lane-bar"></span><span className="lane-bar"></span></div><div style={{ display: 'flex', justifyContent: 'space-between', marginTop: 5, fontSize: 5.5, color: '#617a89' }}><span>LANE 1</span><span data-live="live-ego-lane">LANE 2</span><span>LANE 3</span><span>LANE 4</span></div></div></section>

          <section className={`card bottom-card ${activeNav === 'ANALYTICS' ? 'nav-focus' : ''}`}><div className="card-head"><span>▣ AI ANALYSIS / EVENT STREAM</span><span className="cyan" style={{ fontSize: 6 }}>LIVE FEED</span></div><div className="log-body"><div className="log-item"><span className="log-time">LIVE</span><b className="log-type cyan">PERCEPTION</b><span className="log-msg">Objects synchronized with 3D scene</span></div><div className="log-item"><span className="log-time">LIVE</span><b className="log-type green">PLANNER</b><span className="log-msg">Current action: {phase || 'CRUISING'}</span></div><div className="log-item"><span className="log-time">LIVE</span><b className="log-type yellow">PERSON</b><span className="log-msg">Pedestrian tracking active</span></div><div className="log-item"><span className="log-time">LIVE</span><b className="log-type red">HAZARD</b><span className="log-msg">Pothole layer continuously monitored</span></div><div className="log-item"><span className="log-time">LIVE</span><b className="log-type cyan">RADAR</b><span className="log-msg">Forward range / TTC evaluation active</span></div><div className="log-item"><span className="log-time">LIVE</span><b className="log-type green">FUSION</b><span className="log-msg">Camera + LiDAR + Radar + proximity</span></div></div></section>

          <section className="card bottom-card sensor-data-card">
            <div className="sensor-normal-panel">
              <div className="card-head"><span>◎ SENSOR FUSION CENTER</span><span className="green" style={{ fontSize: 6 }}>SYNCED</span></div>
              <div className="sensor-grid"><div className="sensor-tile"><div className="sensor-name"><span>CAMERA</span><span className="sensor-live">● LIVE</span></div><canvas id="navion-sensor-camera"></canvas><div className="sensor-sub">ACTUAL SIMULATION VIEW</div></div><div className="sensor-tile"><div className="sensor-name"><span>LiDAR</span><span className="sensor-live">● LIVE</span></div><div className="sensor-sub">LIVE POINT CLOUD</div></div><div className="sensor-tile"><div className="sensor-name"><span>RADAR</span><span className="sensor-live">● LIVE</span></div><div className="sensor-sub">FORWARD RANGE / OBJECTS</div></div><div className="sensor-tile"><div className="sensor-name"><span>ULTRASONIC</span><span className="sensor-live">● LIVE</span></div><div className="sensor-sub">0–18M PROXIMITY</div></div></div>
              <div className="fusion-row"><span>FUSION CONFIDENCE</span><b data-live="live-fusion">91%</b></div><div className="fusion-track"><div id="live-fusion-bar" className="fusion-fill" style={{ width: '91%' }}></div></div>
            </div>
            <div className="sensor-data-alt-panel">
              <div className="card-head"><span>◉ LIVE PERCEPTION / SENSOR TELEMETRY</span><span className="green" style={{ fontSize: 6 }}>REAL-TIME</span></div>
              <div className="telemetry-grid">
                <div className="telemetry-card"><span>EGO SPEED</span><b data-live="live-ego-speed">150</b><small>KM/H</small></div>
                <div className="telemetry-card"><span>CURRENT LANE</span><b data-live="live-ego-lane">LANE 2</b><small>ACTIVE</small></div>
                <div className="telemetry-card"><span>VEHICLES</span><b data-live="live-vehicle-count">0</b><small>TRACKED</small></div>
                <div className="telemetry-card"><span>PEDESTRIANS</span><b data-live="live-person-count">0</b><small>TRACKED</small></div>
                <div className="telemetry-card"><span>POTHOLES</span><b data-live="live-pothole-count">0</b><small>HAZARDS</small></div>
                <div className="telemetry-card"><span>TOTAL OBJECTS</span><b data-live="live-object-count">0</b><small>VISIBLE</small></div>
                <div className="telemetry-card"><span>RADAR GAP</span><b data-live="live-radar-gap">CLEAR</b><small data-live="live-radar-zone">NONE</small></div>
                <div className="telemetry-card"><span>FUSION</span><b data-live="live-fusion">91%</b><small>4-SENSOR STACK</small></div>
              </div>
              <div className="telemetry-footer">
                <div><span>LIDAR TARGETS</span><b data-live="live-lidar-count">0</b></div>
                <div><span>RADAR TARGETS</span><b data-live="live-radar-count">0</b></div>
                <div><span>ULTRASONIC NEAR</span><b data-live="live-ultra-count">0</b></div>
                <div><span>CLOSEST OBJECT</span><b data-live="live-ultra-gap">CLEAR</b></div>
                <div><span>CONTROLLER</span><b data-live="live-controller">CRUISE</b></div>
                <div><span>TTC</span><b data-live="live-ttc">—</b></div>
              </div>
            </div>
          </section>

          <section className="card bottom-card"><div className="card-head"><span>⚙ SYSTEM STATUS</span><span className="green" style={{ fontSize: 6 }}>HEALTHY</span></div><div className="status-body"><div className="status-row"><span>Autonomous Drive</span><b>● ON</b></div><div className="status-row"><span>Sensor Stack</span><b>● ACTIVE</b></div><div className="status-row"><span>AI Planner</span><b>● RUNNING</b></div><div className="status-row"><span>Traffic Engine</span><b>● LIVE</b></div><div className="status-row"><span>Path Planner</span><b>● READY</b></div><div className="status-box"><div className="muted">SIMULATION STATE</div><div data-live="live-system-state" className="big">RUNNING</div><div className="small">Real-time perception loop active</div></div></div></section>
        </section>
      </main>

      {!ready && <div className="loading">LOADING REAL 3D TRAFFIC MODELS...</div>}
    </div>
  );
}
