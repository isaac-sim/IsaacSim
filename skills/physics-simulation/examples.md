# Examples

## Worked Examples

Five validated mechanism templates, each chosen for a *non-obvious* technique that
isn't in Parts 1–8. If the only thing an example would teach you is "use CCD" or
"set stabilization=False", it's been merged into the parts above instead.

### 1. Impact / Crash (Vehicle vs Bollard)

**Non-obvious value:** the spring-return-vs-rigid-bollard architecture choice and
the impact-energy math.

**Setup:** 120 Hz physics + 2–4 substeps + CCD; TGS 32/8.

**Two architectures:**
1. **Fixed bollard** — rigid `CollisionAPI` mount. Full force transmission, ~65 kN
   peak for a 2000 kg / 8 km/h hit.
2. **Spring-return bollard** — `CollisionAPI` mount on a prismatic joint with a
   stiff PD drive and damping. ~95% energy absorbed, <0.5 m vehicle rebound, peak
   force reduced > 90%.

**Velocity injection:** set linear velocity via `RigidPrim.set_linear_velocities()`
**after** `timeline.play()`. The USD `physics:velocity` attribute is authored, not
runtime — this is documented in Part 8 as well.

**Analysis:**
```
Impact energy:  E = 0.5 * m * v²        # 2000 kg @ 8 km/h = 4938 J
Peak force:     F = Δp / Δt
Restitution:    e = v_rebound / v_impact
```

**Standards (when you need real numbers):** ASTM F2656, PAS 68, IWA 14-1.

### 2. Vibratory Bowl Feeder (kinematic high-frequency animation)

**Non-obvious value:** the per-physics-step kinematic update pattern with
phase-offset radial-leads-vertical motion that produces net upward part transport.

**Vibration parameters (typical M8-bolt feeder):**
- Frequency 60 Hz, vertical amplitude 0.3 mm, radial amplitude 0.1 mm
- Phase offset: radial **leads** vertical by π/4 (this asymmetry is what climbs parts)

**Physics Hz: 480** (8× vibration frequency; see Hz table in Part 1). CCD on, per-part
iters 16/4, `EnableStabilization=False` on the bowl.

```python
import math

def update_bowl_vibration(bowl_prim, frame, hz=60.0,
                          amp_z=0.0003, amp_r=0.0001, physics_hz=480):
    t = frame / physics_hz
    omega = 2 * math.pi * hz
    dz = amp_z * math.sin(omega * t)
    dr = amp_r * math.sin(omega * t + math.pi / 4)   # radial leads by π/4
    xf = UsdGeom.Xformable(bowl_prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(dr, 0, dz))
```

Call this **every physics step**, not every render frame — the oscillation aliases otherwise.

### 3. Spinning Top — Gyroscopic Stability Formula

**Non-obvious value:** the gyroscopic stability ratio that tells you in advance
whether your top will precess cleanly or just fall over.

**Stability condition** — must be > 1, ideally > 2:
```
ratio = (Izz * ω_spin) / (m * g * d_com * sin(tilt))
```

| Body | I_zz | Stability ratio |
|---|---|---|
| Cone | 3/10 m r² | ~0.6 → topples |
| Disk flywheel | ½ m r² | high → precesses |

**Reference design that works** (ratio = 2.01 at 100 rad/s, 10° tilt, precession ≈ 0.5 rad/s):
- Flywheel: Ø120 mm × 25 mm brass disk, ~0.4 kg
- Stem: Ø8 mm × 40 mm steel
- Tip: Ø6 mm polished steel sphere
- I_zz = 7.2e-4 kg·m², CoM 52.5 mm above tip

**Tunneling failure mode** (logged here so it doesn't surprise you): compound
colliders tunnel through surfaces above ~50 rad/s after 5–6 s. For longer runs use
simpler convex hulls or raise physics Hz beyond 480.

### 4. Newton's Cradle — PhysX Same-Island Contact Limitation

**Non-obvious value:** this is a permanent bug-report on PhysX. **Don't waste time
trying to brute-force it.**

**Symptom:** the classic Newton's cradle (one ball hits, one ball flies off the
opposite end) cannot be reproduced with stock PhysX. TGS and PGS both resolve all
contacts in a single island simultaneously, so momentum gets distributed equally
across all 5 spheres. Result: all 5 oscillate in unison at ~5° instead of sequential
transfer.

**Confirmed not-fixable by tuning:**
- TGS or PGS, 64/32 iterations
- 120 / 240 / 480 Hz
- Tiny gaps between spheres (0.1–0.2 mm)
- Restitution 1.0 with `restitutionCombineMode=max`
- CCD on

**Workaround:** keep the pendulum-chain physics real (revolute joints, gravity,
contact reporting), but **scripted analytical momentum transfer** between the end
spheres on detected impact. The cradle-specific impact resolution is the only
non-physical part.

**Implication for other scenes:** any same-island contact chain where you need
**sequential** momentum propagation has this limitation. Plan accordingly.

### 5. Escapement Clock — Hybrid Physics + Scripted Mechanism

**Non-obvious value:** the architecture template for any sub-10 mm mechanical
contact (escapements, ratchets, latches, watch movements). Pure PhysX collision is
unreliable below ~10 mm; this hybrid is the proven approach.

**Architecture:**

| Component | Mode | Role |
|---|---|---|
| Pendulum | **Dynamic** rigid body, revolute joint, gravity-driven | The real physics |
| Escape wheel | **Kinematic**, rotation set by controller | Scripted mechanism |
| Anchor | **Visual-only** (rotation derived from pendulum angle) | Cosmetic |
| Frame | **Static** | Housing |

**4-state mechanism controller:**
```
LOCKED_LEFT → RELEASING_RIGHT → LOCKED_RIGHT → RELEASING_LEFT → LOCKED_LEFT
```
- `LOCKED_*`: wait for pendulum to swing past `RELEASE_ANGLE`
- `RELEASING_*`: pendulum returns past `LOCK_ANGLE` → kinematic wheel advances by
  `HALF_TOOTH_DEG` (e.g., 18° for a 10-tooth wheel) → next `LOCKED_*` state

One full tick = two half-ticks = one tooth.

**Physics on the pendulum** (critical for accurate period):
- `enableStabilization=False`, `angularDamping=0.0`, `jointFriction=0.0`
- `diagonalInertia = m_bob·L² + m_rod·L²/3`
- 32/16 iterations
- **No collision on the bob** — frame clipping at high amplitude breaks the joint

**Initialization:** set initial tilt via `AddRotateYOp(7°)`, **not** via angular
velocity (DC's `set_rigid_body_angular_velocity` is unreliable for init). Apply
small impulse kicks (~0.015 rad/s) at each tick to compensate numerical damping —
PhysX revolute joints lose ~40% energy over 15 s even with all mitigations.

**Reusability:** any mechanism where one body's *contact* drives another body's
discrete *state advance* fits this template. Replace the pendulum + escape wheel
with whatever pair you need.

---

## Integration Points

- **RECEIVES from:** `urdf-mjcf-to-usd-conversion` — articulated robot USD with drive config
- **RECEIVES from:** `usd-articulation` — multi-link chain assembly
- **PRODUCES for:** `data-collection-sim` — physically grounded scenes for SDG
- **PRODUCES for:** `manipulation-ik` — grasping physics setup

## When Stuck

- `isaac-sim-troubleshooting` — Kit 110 hang/freeze/perf reference
