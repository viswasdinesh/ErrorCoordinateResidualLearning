#!/usr/bin/env python3
"""collect_figure8_windy.py -- orchestrates ONE held-out ECRL data-collection
run: the built-in analytic figure-8 reference (ommpc_node's
`trajectory=figure8`), flown in the windy EVAL Gazebo scene
(sim_worlds/ecrl_windy_eval.sdf), with fresh schema_version=2 raw logging via
ecrl_data_collector_node.cpp. Pairs with evaluate_figure8_windy.py, which
scores the ecrl/ambient trained models against the resulting CSV.

Pipeline (each stage is a long-lived subprocess this script owns and tears
down at the end -- nothing is left running):
  1. MicroXRCEAgent udp4 -p 8888        (PX4 <-> ROS2 DDS bridge; PX4's own
     rcS startup script runs `uxrce_dds_client start` as a UDP *client*, so
     something has to be listening on the other end first)
  2. PX4 SITL + Gazebo (ecrl_windy_eval world), HEADLESS=1 (no GPU here;
     llvmpipe software rendering, so boot can be slow -- generous timeout)
       - waits for "Ready for takeoff!" in the PX4 log
       - sets PX4 param NAV_DLL_ACT=0 over the interactive pxh> console --
         fixes the "no GCS connection" arming block in this no-QGroundControl
         sandbox (see docs/wind_generalization_experiment.md, Infrastructure
         issues). This is NOT a code change, just a per-SITL-instance param.
  3. live wind injection via `gz topic -t /world/<world>/wind ...` --
     WindEffects subscribes to this at runtime, no SITL restart needed.
  4. ommpc_node, CLI-overridden with `-p trajectory:=figure8` (this param is
     read once at startup in declareParameters(), no runtime callback -- see
     ommpc_node.cpp -- so it MUST be set at launch, not via `ros2 param set`
     afterward). ommpc_node arms and requests OFFBOARD itself
     (triggerArmAndOffboard(), called every ~1s while in WAIT_OFFBOARD) --
     no external GCS/mavlink-shell arm sequence is needed once NAV_DLL_ACT=0
     is set.
  5. ecrl_data_collector_node, `csv_filepath` pointed at a fresh path (its
     default is a stale leftover from earlier work -- always pass this
     explicitly).
  6. waits for "Transitioning from HOVER to TRACKING" in the ommpc_node log,
     then lets TRACKING run for `--periods` x `--period_s` seconds (--period_s
     is also forwarded to ommpc_node as figure8_period_s, a runtime param --
     shorten it for a more agile, higher-velocity/acceleration reference
     through the same fixed spatial amplitude). No commanded stop is needed for figure8/circle
     trajectories -- the FSM loops TRACKING forever until torn down.
  7. tears everything down: SIGKILL to each subprocess's *process group*
     (every subprocess is started via start_new_session=True so its whole
     descendant tree -- including `gz sim`, which reparents to init but
     keeps the same pgid -- is killable as one unit), verified by checking
     `ps -g <pgid>` membership, retried. Plain SIGTERM/`pkill -f` was found
     unreliable in prior sessions of this same experiment (see docs) --
     direct SIGKILL in a retry-and-verify loop is what's used here. A
     pattern-based sweep runs first (idempotent cleanup of anything left
     over from a previous crashed run) and again at the very end as a
     safety net.

IMPORTANT lesson learned while building this script (empirically, not from
the docs) and encoded below: PX4's pxh> console, when its stdin is not a
real TTY, spins printing its prompt at tens of MB/s if stdin ever sees EOF
while nothing holds the write end open -- confirmed by accident during
development (a boot log hit 500+ MB in under 30s). Consequences for this
script's design:
  - PX4's stdin must be a pipe this script holds open for the process's
    *entire* lifetime -- never /dev/null, never closed early.
  - This script must never exit (crash or otherwise) while PX4 is still
    alive without running teardown first -- enforced via try/finally plus
    SIGINT/SIGTERM handlers.
  - Every subprocess log is additionally opened with RLIMIT_FSIZE capped
    (see _limit_fsize) as a last-resort safety net against exactly this
    failure mode re-occurring and filling the disk.

Usage:
    python3 scripts/collect_figure8_windy.py \\
        --csv_path /home/dinesh/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv \\
        --wind_x 1.3 --wind_y 0.7 --periods 6
"""
import argparse
import glob
import os
import random
import resource
import signal
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Small process-management helpers
# ---------------------------------------------------------------------------
_START_TIME = time.time()


def log(msg: str):
    print(f"[{time.time() - _START_TIME:8.1f}s] {msg}", flush=True)


def _limit_fsize(max_bytes: int):
    """preexec_fn: cap the child's own max file size (RLIMIT_FSIZE). If the
    child ever writes past this, the kernel sends it SIGXFSZ (default action:
    terminate) -- a safety net against the pxh>-console runaway-spin failure
    mode described in the module docstring, without affecting any normal
    (KB-scale) boot/run log."""
    def _setter():
        resource.setrlimit(resource.RLIMIT_FSIZE, (max_bytes, max_bytes))
    return _setter


def is_running(pid: int) -> bool:
    """True iff /proc/<pid> exists AND the process is not a zombie. A zombie
    still shows up in `ps`/`/proc` until its parent reaps it via wait() --
    kill -9 on one is a no-op, so any alive-check that doesn't filter zombies
    out will spin for its whole retry budget on a process that's already
    dead in every sense that matters."""
    try:
        with open(f"/proc/{pid}/stat", "r") as f:
            # comm field can contain spaces/parens; state is right after the
            # last ')'.
            state = f.read().rsplit(")", 1)[-1].split()[0]
        return state != "Z"
    except (FileNotFoundError, ProcessLookupError, IndexError):
        return False


def pgid_members(pgid: int):
    """Non-zombie PIDs in this process group (see is_running)."""
    try:
        out = subprocess.run(["ps", "-o", "pid=", "-g", str(pgid)],
                              capture_output=True, text=True, timeout=5)
        pids = [int(p) for p in out.stdout.split()]
    except Exception:
        return []
    return [pid for pid in pids if is_running(pid)]


def kill_group(name: str, proc: subprocess.Popen, tries: int = 15, wait_s: float = 1.0):
    """SIGKILL every non-zombie member of proc's process group (proc.pid,
    since every subprocess here is started with start_new_session=True and
    is therefore its own pgid leader), retrying and re-checking until
    nothing is left. Also calls proc.wait() each round: proc is OUR direct
    child (the immediate Popen result), so without reaping it via wait(),
    it becomes a zombie the instant it's killed and both `ps` and our own
    alive-check keep reporting it as "still there" forever -- descendants
    that got reparented to init (e.g. `gz sim` after its parent `make`
    dies) don't have this problem since init reaps its own adoptees."""
    if proc is None:
        return
    pgid = proc.pid
    for attempt in range(1, tries + 1):
        try:
            proc.wait(timeout=0.01)
        except subprocess.TimeoutExpired:
            pass
        alive = pgid_members(pgid)
        if not alive:
            log(f"[teardown] {name}: process group {pgid} confirmed dead (check {attempt})")
            return
        log(f"[teardown] {name}: pgid {pgid} still has {alive}, sending SIGKILL (attempt {attempt}/{tries})")
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        except PermissionError as e:
            log(f"[teardown] {name}: killpg PermissionError: {e}")
        try:
            proc.wait(timeout=wait_s)
        except subprocess.TimeoutExpired:
            pass
    alive = pgid_members(pgid)
    if alive:
        log(f"[teardown] WARNING: {name} pgid {pgid} still has members after {tries} attempts: {alive}")
    else:
        log(f"[teardown] {name}: confirmed dead (post-loop check)")


PATTERNS = [
    "MicroXRCEAgent",
    "px4_sitl_default/bin/px4",
    "gz sim --verbose",
    "make px4_sitl gz_x500",
    "lib/ommpc_ros2/ecrl_data_collector_node",
    "lib/ommpc_ros2/ommpc_node",
]


def find_pids_by_pattern(patterns):
    out = subprocess.run(["ps", "-eo", "pid,args"], capture_output=True, text=True, timeout=10)
    pids = []
    lines = out.stdout.splitlines()[1:]
    for line in lines:
        line = line.strip()
        if not line:
            continue
        pid_str, _, rest = line.partition(" ")
        try:
            pid = int(pid_str)
        except ValueError:
            continue
        if pid == os.getpid():
            continue
        for pat in patterns:
            if pat in rest:
                pids.append(pid)
                break
    return pids


def kill_pids(name: str, pids, tries: int = 15, wait_s: float = 1.0):
    remaining = set(pids)
    for attempt in range(1, tries + 1):
        remaining = {p for p in remaining if is_running(p)}
        if not remaining:
            log(f"[teardown] {name}: all matched pids confirmed dead (check {attempt})")
            return
        log(f"[teardown] {name}: killing {sorted(remaining)} (attempt {attempt}/{tries})")
        for p in remaining:
            try:
                os.kill(p, signal.SIGKILL)
            except ProcessLookupError:
                pass
        time.sleep(wait_s)
    remaining = {p for p in remaining if is_running(p)}
    if remaining:
        log(f"[teardown] WARNING: {name} pids still alive after retries: {sorted(remaining)}")


def preflight_cleanup():
    pids = find_pids_by_pattern(PATTERNS)
    if pids:
        log(f"[preflight] leftover processes from a previous run found: {pids} -- cleaning up first")
        kill_pids("preflight-sweep", pids)
    else:
        log("[preflight] no leftover px4/gz/ommpc processes found, clean start")


def wait_for_pattern(log_path: str, pattern: str, timeout_s: float, proc: subprocess.Popen = None,
                      poll_s: float = 2.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if os.path.exists(log_path):
            with open(log_path, "r", errors="replace") as f:
                if pattern in f.read():
                    return True
        if proc is not None and proc.poll() is not None:
            log(f"  process exited early (code {proc.returncode}) while waiting for '{pattern}'")
            return False
        time.sleep(poll_s)
    return False


def gust_loop(stop_event: threading.Event, wind_topic: str, base_x: float, base_y: float,
              gust_std: float, interval_s: float, seed: int):
    """Background thread: re-publishes the world's wind topic every
    interval_s with a fresh independent Gaussian perturbation around
    (base_x, base_y). Deliberately independent draws each tick (not a
    random walk) so consecutive gusts aren't autocorrelated -- the point is
    disturbance content faster than integral_trim_tau's leaky-integrator
    time constant (default 10s), which the base wind_x/wind_y (constant)
    plus the eval world's own baked-in sin+noise modulation (9-13s periods,
    comparable to tau -- see sim_worlds/ecrl_windy_eval.sdf) mostly is not.
    Runs until stop_event is set (from teardown()); errors from individual
    `gz topic` publishes are logged but don't abort the run."""
    rng = random.Random(seed)
    tick = 0
    while not stop_event.wait(interval_s):
        tick += 1
        gx = base_x + rng.gauss(0.0, gust_std)
        gy = base_y + rng.gauss(0.0, gust_std)
        cmd = ["gz", "topic", "-t", wind_topic, "-m", "gz.msgs.Wind", "-p",
               f"linear_velocity: {{x: {gx:.3f}, y: {gy:.3f}, z: 0.0}}, enable_wind: true"]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if res.returncode != 0:
                log(f"[gust] tick {tick}: gz topic returned {res.returncode}: {res.stdout} {res.stderr}")
        except Exception as e:
            log(f"[gust] tick {tick}: gz topic call failed: {e}")


# ---------------------------------------------------------------------------
# Main orchestration
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--csv_path", default="/home/dinesh/ecrl_datasets_v2_eval_windy/figure8/figure8_windy_run1.csv",
                     help="Fresh output path for ecrl_data_collector_node's schema_version=2 raw CSV")
    ap.add_argument("--px4_dir", default="/home/dinesh/PX4-Autopilot")
    ap.add_argument("--ws_dir", default="/home/dinesh/ecrl_ws")
    ap.add_argument("--world", default="ecrl_windy_eval")
    ap.add_argument("--wind_x", type=float, default=1.3,
                     help="Wind linear_velocity.x, m/s. Default direction (1.3, 0.7) is deliberately "
                          "different from both sim_worlds/*.sdf baked-in defaults "
                          "(train=[1.3,-0.7], eval=[-0.8,1.4]) for a genuinely held-out direction.")
    ap.add_argument("--wind_y", type=float, default=0.7)
    ap.add_argument("--gusty", action="store_true",
                     help="On top of the constant --wind_x/--wind_y base and the eval world's own "
                          "baked-in sin+noise modulation, actively re-publish the wind topic every "
                          "--gust_interval_s with a fresh independent Gaussian perturbation "
                          "(std --gust_std) -- disturbance content faster than integral_trim_tau's "
                          "default 10s time constant, to test whether the ecrl NN correction adds "
                          "anything beyond the disturbance-observer trim once the disturbance isn't "
                          "well-approximated by a slow-tracked constant bias.")
    ap.add_argument("--gust_std", type=float, default=1.5,
                     help="Std dev (m/s) of each gust perturbation around the base wind vector. Only used with --gusty.")
    ap.add_argument("--gust_interval_s", type=float, default=2.0,
                     help="Seconds between gust re-publishes. Only used with --gusty.")
    ap.add_argument("--gust_seed", type=int, default=42, help="RNG seed for gust perturbations (reproducibility).")
    ap.add_argument("--periods", type=int, default=6, help="Number of figure-8 periods to record in TRACKING")
    ap.add_argument("--period_s", type=float, default=20.0,
                     help="Figure-8/circle period, s -- forwarded to ommpc_node as -p "
                          "figure8_period_s:=X (runtime param, was hardcoded to 20.0 pre-agile-"
                          "trajectory support). Also used for this script's own recording-duration "
                          "math (record_s = periods * period_s), so the two always stay in sync. "
                          "Shorter = higher peak velocity/acceleration through the same spatial "
                          "amplitude (A=B=1.5m figure8, R=1.5m circle, both fixed in ommpc_node.cpp).")
    ap.add_argument("--px4_ready_timeout", type=float, default=300.0)
    ap.add_argument("--tracking_timeout", type=float, default=180.0,
                     help="Max time to wait for the HOVER->TRACKING transition after node launch")
    ap.add_argument("--no_teardown", action="store_true", help="Debug only: skip teardown, leave everything running")
    ap.add_argument("--enable_residual_correction", action="store_true",
                     help="Enable the live ecrl residual QP correction (ommpc_node param, off by "
                          "default -- see docs/residual_conventions.md 'Relationship to Task 4's "
                          "affine QP term'). Omitting this flag reproduces today's baseline behavior "
                          "unchanged.")
    ap.add_argument("--enable_ambient_correction", action="store_true",
                     help="Enable the live ambient-adapter residual QP correction (8-D body-frame "
                          "vel/rate/thrust features, no reference trajectory -- the alternative "
                          "adapter to --enable_residual_correction's ecrl features). Mutually "
                          "exclusive in practice: if both are set, ecrl takes priority and ambient "
                          "will not run (ommpc_node logs a warning).")
    ap.add_argument("--residual_clip_norm", type=float, default=0.08,
                     help="Shared cap on the combined NN+trim correction norm, and on trim's own "
                          "internal d_hat_ state. Applied whenever --enable_residual_correction, "
                          "--enable_ambient_correction, and/or --enable_integral_trim is set (a "
                          "no-op otherwise).")
    ap.add_argument("--enable_integral_trim", action="store_true",
                     help="Enable the live disturbance-observer integral trim (ommpc_node param, off "
                          "by default). Complementary to --enable_residual_correction: targets the "
                          "steady-state offset a stateless per-cycle correction can't close. Can be "
                          "combined with --enable_residual_correction.")
    ap.add_argument("--integral_trim_tau", type=float, default=10.0,
                     help="Only used when --enable_integral_trim is set. Leaky-integrator time "
                          "constant, seconds.")
    ap.add_argument("--mpc_param", action="append", default=[], metavar="KEY=VALUE",
                     help="Override an MPC_params.* field from config/params.yaml at launch, e.g. "
                          "--mpc_param Q_pos_xy=10.0 --mpc_param R_thrust=0.005. Repeatable. "
                          "Forwarded as -p MPC_params.KEY:=VALUE (read once at ommpc_node startup, "
                          "same as the correction/trim flags -- not settable afterward). See "
                          "config/params.yaml MPC_params: block for valid field names.")
    args = ap.parse_args()

    csv_dir = os.path.dirname(os.path.abspath(args.csv_path))
    os.makedirs(csv_dir, exist_ok=True)
    log_dir = os.path.join(csv_dir, "logs")
    os.makedirs(log_dir, exist_ok=True)

    ros_setup_candidates = sorted(glob.glob("/opt/ros/*/setup.bash"))
    if not ros_setup_candidates:
        log("ERROR: no /opt/ros/*/setup.bash found")
        sys.exit(1)
    ros_setup = ros_setup_candidates[0]
    ws_setup = os.path.join(args.ws_dir, "install", "setup.bash")
    params_yaml = os.path.join(args.ws_dir, "src", "ommpc_ros2", "config", "params.yaml")
    if not os.path.exists(ws_setup):
        log(f"ERROR: {ws_setup} not found -- build the workspace first "
            f"(colcon build --packages-select ommpc_ros2)")
        sys.exit(1)
    if not os.path.exists(params_yaml):
        log(f"ERROR: {params_yaml} not found")
        sys.exit(1)

    px4_log_path = os.path.join(log_dir, "px4_boot.log")
    xrce_log_path = os.path.join(log_dir, "microxrceagent.log")
    ommpc_log_path = os.path.join(log_dir, "ommpc_node.log")
    collector_log_path = os.path.join(log_dir, "ecrl_data_collector_node.log")

    FSIZE_CAP = 400 * 1024 * 1024  # 400 MB, see _limit_fsize docstring

    procs = {}  # name -> (Popen, pgid)
    log_files = []
    gust_stop_event = threading.Event()
    gust_thread = None

    def start(name, argv, cwd, log_path, env=None, stdin_pipe=False):
        f = open(log_path, "wb", buffering=0)
        log_files.append(f)
        p = subprocess.Popen(
            argv, cwd=cwd, env=env,
            stdin=subprocess.PIPE if stdin_pipe else subprocess.DEVNULL,
            stdout=f, stderr=subprocess.STDOUT,
            start_new_session=True,
            preexec_fn=_limit_fsize(FSIZE_CAP),
            text=True,  # so px4_proc.stdin.write() below can take str, not bytes.
            # stdout/stderr are our own already-open binary file objects (f), not
            # PIPEs Popen creates itself, so text=True has no effect on them --
            # only stdin (when stdin_pipe=True) is actually wrapped as text.
        )
        procs[name] = p
        log(f"started {name}: pid/pgid={p.pid}, log={log_path}")
        return p

    def teardown():
        if args.no_teardown:
            log("[teardown] --no_teardown set, leaving all processes running")
            return
        log("=== tearing down ===")
        if gust_thread is not None:
            gust_stop_event.set()
            gust_thread.join(timeout=10.0)
            log("[teardown] gust thread stopped")
        # Reverse start order: collector, ommpc_node, px4/gz, microxrceagent.
        for name in ["collector", "ommpc_node", "px4", "microxrceagent"]:
            p = procs.get(name)
            if p is not None:
                kill_group(name, p)
        # Final pattern-based safety net in case anything escaped its pgid
        # (e.g. spawned via a further setsid of its own).
        leftover = find_pids_by_pattern(PATTERNS)
        if leftover:
            log(f"[teardown] safety-net sweep still finds {leftover}, killing directly")
            kill_pids("safety-net", leftover)
        else:
            log("[teardown] safety-net sweep: clean")
        for f in log_files:
            try:
                f.close()
            except Exception:
                pass

    def handle_signal(signum, frame):
        log(f"received signal {signum}, tearing down and exiting")
        teardown()
        sys.exit(1)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        log(f"=== collect_figure8_windy.py starting ===")
        log(f"csv_path={args.csv_path}")
        log(f"world={args.world} wind=({args.wind_x},{args.wind_y}) periods={args.periods} x {args.period_s}s")

        preflight_cleanup()

        # 1. MicroXRCEAgent
        start("microxrceagent", ["MicroXRCEAgent", "udp4", "-p", "8888"], cwd=None, log_path=xrce_log_path)
        time.sleep(2.0)
        if procs["microxrceagent"].poll() is not None:
            log("ERROR: MicroXRCEAgent exited immediately -- port 8888 conflict? check log.")
            raise RuntimeError("MicroXRCEAgent failed to start")

        # 2. PX4 SITL + Gazebo (windy eval world). stdin MUST be a pipe held
        # open for this process's entire lifetime -- see module docstring.
        px4_env = dict(os.environ)
        px4_env["HEADLESS"] = "1"
        px4_env["PX4_GZ_WORLD"] = args.world
        px4_proc = start("px4", ["make", "px4_sitl", "gz_x500"], cwd=args.px4_dir,
                          log_path=px4_log_path, env=px4_env, stdin_pipe=True)

        log(f"waiting up to {args.px4_ready_timeout:.0f}s for 'Ready for takeoff!' "
            f"(llvmpipe software rendering is slow, be patient)...")
        if not wait_for_pattern(px4_log_path, "Ready for takeoff", args.px4_ready_timeout, proc=px4_proc):
            raise RuntimeError(f"PX4 SITL did not report ready within {args.px4_ready_timeout:.0f}s -- see {px4_log_path}")
        log("PX4 SITL ready.")

        log("setting NAV_DLL_ACT=0 over the pxh> console (fixes no-GCS arming block)...")
        px4_proc.stdin.write("param set NAV_DLL_ACT 0\n")
        px4_proc.stdin.flush()
        time.sleep(2.0)
        px4_proc.stdin.write("param show NAV_DLL_ACT\n")
        px4_proc.stdin.flush()
        time.sleep(2.0)
        with open(px4_log_path, "r", errors="replace") as f:
            tail = f.read()[-2000:]
        if "NAV_DLL_ACT" in tail and ": 0" in tail.split("NAV_DLL_ACT")[-1][:40]:
            log("  confirmed: NAV_DLL_ACT = 0")
        else:
            log("  WARNING: could not confirm NAV_DLL_ACT=0 from log tail -- proceeding anyway "
                "(ommpc_node retries arm/offboard every ~1s regardless)")

        # 3. Live wind injection.
        wind_topic = f"/world/{args.world}/wind"
        wind_cmd = ["gz", "topic", "-t", wind_topic, "-m", "gz.msgs.Wind", "-p",
                    f"linear_velocity: {{x: {args.wind_x}, y: {args.wind_y}, z: 0.0}}, enable_wind: true"]
        log(f"enabling wind: {' '.join(wind_cmd)}")
        wind_res = subprocess.run(wind_cmd, capture_output=True, text=True, timeout=15)
        if wind_res.returncode != 0:
            log(f"  WARNING: gz topic wind command returned {wind_res.returncode}: "
                f"{wind_res.stdout} {wind_res.stderr}")
        else:
            log("  wind enabled.")

        if args.gusty:
            log(f"  gusty mode: re-publishing wind every {args.gust_interval_s:.1f}s with "
                f"N(0, {args.gust_std:.2f}) m/s perturbation (seed={args.gust_seed})")
            gust_thread = threading.Thread(
                target=gust_loop,
                args=(gust_stop_event, wind_topic, args.wind_x, args.wind_y,
                      args.gust_std, args.gust_interval_s, args.gust_seed),
                daemon=True)
            gust_thread.start()

        # 4. ommpc_node, trajectory=figure8 forced via CLI -p (read once at
        # startup, see ommpc_node.cpp -- config/params.yaml itself is NOT
        # modified, only overridden on this invocation).
        residual_overrides = ""
        if args.enable_residual_correction or args.enable_ambient_correction or args.enable_integral_trim:
            # residual_clip_norm is shared by all paths -- it caps the
            # combined NN+trim correction AND trim's own internal d_hat_
            # state (see Parameter_t's doc comment in ommpc_controller.hpp)
            # -- so it must be passed whenever any is enabled, not just
            # enable_residual_correction. Previously only forwarded in the
            # enable_residual_correction branch, so a trim-only run silently
            # kept the node's default 0.08 no matter what --residual_clip_norm
            # was passed here.
            residual_overrides += f" -p residual_clip_norm:={args.residual_clip_norm}"
            log(f"residual_clip_norm={args.residual_clip_norm}")
        if args.enable_residual_correction:
            residual_overrides += " -p enable_residual_correction:=true"
            log("enable_residual_correction=true")
        if args.enable_ambient_correction:
            residual_overrides += " -p enable_ambient_correction:=true"
            log("enable_ambient_correction=true")
        if args.enable_integral_trim:
            residual_overrides += (f" -p enable_integral_trim:=true "
                                    f"-p integral_trim_tau:={args.integral_trim_tau}")
            log(f"enable_integral_trim=true (integral_trim_tau={args.integral_trim_tau})")

        mpc_overrides = ""
        for kv in args.mpc_param:
            if "=" not in kv:
                raise ValueError(f"--mpc_param must be KEY=VALUE, got: {kv!r}")
            key, value = kv.split("=", 1)
            mpc_overrides += f" -p MPC_params.{key}:={value}"
            log(f"MPC_params.{key}={value}")

        period_override = f" -p figure8_period_s:={args.period_s}"
        log(f"figure8_period_s={args.period_s}")
        ros_bash = (f"source {ros_setup} && source {ws_setup} && "
                    f"exec ros2 run ommpc_ros2 ommpc_node --ros-args "
                    f"--params-file {params_yaml} -p trajectory:=figure8{period_override}"
                    f"{residual_overrides}{mpc_overrides}")
        start("ommpc_node", ["bash", "-lc", ros_bash], cwd=args.ws_dir, log_path=ommpc_log_path)

        # 5. ecrl_data_collector_node, pointed at the fresh CSV path.
        collector_bash = (f"source {ros_setup} && source {ws_setup} && "
                           f"exec ros2 run ommpc_ros2 ecrl_data_collector_node --ros-args "
                           f"-p csv_filepath:={args.csv_path}")
        start("collector", ["bash", "-lc", collector_bash], cwd=args.ws_dir, log_path=collector_log_path)

        # 6. Wait for HOVER -> TRACKING, then record for periods x period_s.
        log(f"waiting up to {args.tracking_timeout:.0f}s for ommpc_node to reach TRACKING "
            f"(arm/offboard/spool-up/takeoff/hover sequence)...")
        if not wait_for_pattern(ommpc_log_path, "Transitioning from HOVER to TRACKING",
                                 args.tracking_timeout, proc=procs["ommpc_node"]):
            raise RuntimeError(f"ommpc_node never reached TRACKING within {args.tracking_timeout:.0f}s -- "
                                f"see {ommpc_log_path} (and {px4_log_path} for arming issues)")
        log("TRACKING started.")

        record_s = args.periods * args.period_s
        log(f"recording for {args.periods} periods x {args.period_s:.0f}s = {record_s:.0f}s ...")
        t_end = time.time() + record_s
        while time.time() < t_end:
            time.sleep(5.0)
            remaining = t_end - time.time()
            if remaining > 0:
                log(f"  ... {remaining:.0f}s remaining")
            # Bail early and loudly if either node has died mid-flight.
            for name in ("ommpc_node", "collector", "px4"):
                p = procs[name]
                if p.poll() is not None:
                    raise RuntimeError(f"{name} exited unexpectedly (code {p.returncode}) mid-recording")

        log("recording window complete.")

    finally:
        teardown()

    if os.path.exists(args.csv_path):
        with open(args.csv_path, "r", errors="replace") as f:
            first_line = f.readline().strip()
            n_lines = 1 + sum(1 for _ in f)
        log(f"=== done. {args.csv_path}: {n_lines} lines, header='{first_line}' ===")
    else:
        log(f"=== done, but {args.csv_path} was never created -- check logs in {log_dir} ===")
        sys.exit(1)


if __name__ == "__main__":
    main()
