#!/usr/bin/env python3
"""
    Robot controller class.
    Owns the robot connection and executes move commands received via command_queue.
    Sends "ready", "done", or "error:<msg>" strings via result_queue.
    Shuts down cleanly when it receives None on command_queue.

    v6 changes:
      * On every startup we PURGE all known tool/TCP names before
        re-registering, so previous runs cannot leave stale entries
        that block add_tool/add_tcp.
      * Purge runs inside the Manual-mode block where del_tool / del_tcp
        are actually allowed.
"""

import doosan_drfl as drfl
import multiprocessing as mp
import logging
import time
import numpy as np

logger = logging.getLogger(__name__)

MOVE_TIMEOUT  = 120.0   # seconds
POLL_INTERVAL = 0.01    # seconds

# ── Tool configuration ─────────────────────────────────────────────────
TOOL_SHAPE_NAME = "Tool_Shape"

# ── Active tool: snoeks_suction ──
TOOL_NAME    = "snoeks_suction"
TOOL_WEIGHT  = 0.500  # kg
# COG unknown — placeholder, roughly halfway along the TCP Z offset.
TOOL_CENTER  = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # Cx, Cy, Cz (mm)
TOOL_INERTIA = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

# TCP offset for snoeks_suction — tip is 30 mm down the flange Z axis.
# IMPORTANT: TCP_NAME must differ from TOOL_NAME on some firmwares.
TCP_NAME   = "snoeks_suction_tcp"
TCP_OFFSET = np.array([-43.032, 1.759, 93.887, 0.0, 0.0, 0.0], dtype=np.float32)

MAX_FORCE_BOX = 5.5

COMPLIANCE_STIFFNESS = np.array(
    [2500.0, 2500.0, 1500.0, 200.0, 200.0, 200.0], dtype=np.float32
)

FORCE_PROBE_FD  = np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.0], dtype=np.float32)
FORCE_PROBE_DIR = np.array([0,   0,    1,   0,   0,   0  ], dtype=np.uint8)


class RobotController:
    def __init__(
        self,
        ip_address: str,
        port: int,
        speed: float = 20.0,
        acceleration: float = 10.0,
        lin_speed: float = 60.0,
        lin_acceleration: float = 35.0,
        force_speed: float = 5.0,
        force_acceleration: float = 5.0,
    ) -> None:
        self.ip_address = ip_address
        self.port = port
        self.speed = speed
        self.acceleration = acceleration
        self.lin_speed = lin_speed
        self.lin_acceleration = lin_acceleration
        self.force_speed = force_speed
        self.force_acceleration = force_acceleration

        self._max_force = MAX_FORCE_BOX

        self.robot = drfl.CDRFLEx()

        self.get_control_access: bool = False
        self.is_in_standby: bool = False
        self.is_in_safe_off: bool = False

        self.move_handlers = {
            "joint":  self._amovej,
            "linear": self._amovel,
            "movejx": self._ammovejx,
        }

        self.in_force_mode: bool = False

    # ── Callbacks ──────────────────────────────────────────────────────
    def _on_monitoring_access_control(self, access) -> None:
        logger.debug(f"[on_monitoring_access_control] "
                     f"{drfl.MONITORING_ACCESS_CONTROL(access).name} [{access}]")
        if access == drfl.MONITORING_ACCESS_CONTROL.Grant:
            logger.info("Access granted!")
            self.get_control_access = True
        elif access == drfl.MONITORING_ACCESS_CONTROL.Loss:
            logger.info("Access lost!")
            self.get_control_access = False

    def _on_monitoring_state(self, state) -> None:
        logger.debug(f"[on_monitoring_state] "
                     f"{drfl.ROBOT_STATE(state).name} [{state}]")
        self.is_in_standby  = (state == drfl.ROBOT_STATE.Standby)
        self.is_in_safe_off = (state == drfl.ROBOT_STATE.Safe_off)

    # ── Purge existing tools/TCPs (must be in Manual + access granted) ─
    def _purge_all_tools_and_tcps(self) -> None:
        """
        Delete every tool and TCP currently registered on the controller.
        Must be called while in Manual mode with access control granted.
        Safe to call even when nothing is registered — errors are logged
        and swallowed.
        """
        known_tools = [TOOL_NAME, "default", "Tool#20",
                       "snoeks1", "snoeks_suction", "snoeks_suction1"]
        known_tcps  = [TCP_NAME, "default", "snoeks1",
                       "snoeks_suction", "snoeks_suction_tcp"]

        # Deactivate first so nothing is "in use" when we delete
        try: self.robot.set_tool("")
        except Exception as e: logger.debug(f"[purge] set_tool('') raised: {e}")
        try: self.robot.set_tcp("")
        except Exception as e: logger.debug(f"[purge] set_tcp('') raised: {e}")

        for name in set(known_tcps):
            try:
                ok = self.robot.del_tcp(name)
                logger.info(f"[purge] del_tcp({name!r}) -> {ok}")
            except Exception as e:
                logger.debug(f"[purge] del_tcp({name!r}) raised: {e}")

        for name in set(known_tools):
            try:
                ok = self.robot.del_tool(name)
                logger.info(f"[purge] del_tool({name!r}) -> {ok}")
            except Exception as e:
                logger.debug(f"[purge] del_tool({name!r}) raised: {e}")

    # ── Move commands ──────────────────────────────────────────────────
    def _amovej(self, pose, speed=None, acc=None) -> bool:
        return self.robot.amovej(
            pos=pose,
            vel=speed if speed is not None else self.speed,
            acc=acc if acc is not None else self.acceleration,
            time=0.0,
            move_mode=drfl.MOVE_MODE.Absolute,
            blending_type=drfl.BLENDING_SPEED_TYPE.Duplicate,
        )

    def _amovel(self, pose, speed=None, acc=None) -> bool:
        velocity = np.array(
            [speed if speed is not None else self.lin_speed, 0], dtype=np.float32)
        acceleration = np.array(
            [acc if acc is not None else self.lin_acceleration, 0], dtype=np.float32)
        return self.robot.amovel(
            pos=pose,
            vel=velocity,
            acc=acceleration,
            time=0.0,
            move_mode=drfl.MOVE_MODE.Absolute,
            move_reference=drfl.MOVE_REFERENCE.Base,
            blending_type=drfl.BLENDING_SPEED_TYPE.Duplicate,
            app_type=drfl.DR_MV_APP.NoApp,
        )

    def _ammovejx(self, pose, solution_space, speed=None, acc=None) -> bool:
        if speed is None: speed = self.speed
        if acc   is None: acc   = self.acceleration
        return self.robot.amovejx(
            pos=pose,
            solution_space=solution_space,
            vel=speed,
            acc=acc,
            time=0.0,
            move_mode=drfl.MOVE_MODE.Absolute,
            move_reference=drfl.MOVE_REFERENCE.Base,
            blending_type=drfl.BLENDING_SPEED_TYPE.Duplicate,
        )

    def _enable_compliance(self) -> bool:
        return self.robot.task_compliance_ctrl(
            fTargetStiffness=COMPLIANCE_STIFFNESS,
            eForceReference=drfl.COORDINATE_SYSTEM.Base,
            time=0.0,
        )

    def _disable_compliance(self) -> bool:
        self.in_force_mode = False
        return self.robot.release_compliance_ctrl()

    def _amove_force(self, pose: np.ndarray) -> bool:
        time.sleep(0.5)
        self.in_force_mode = True
        return self._amovel(pose, speed=self.force_speed, acc=self.force_acceleration)

    def is_moving(self, timeout: float) -> bool:
        once = False
        time.sleep(0.5)
        now = time.time()
        while True:
            result = self.robot.check_motion()
            if result == 0:
                logger.info("Robot is done moving!")
                return True
            if time.time() - now > timeout:
                logger.info("Movement timed out")
                return False
            if not once:
                logger.info(f"Robot is still moving: {result}")
                once = True

            if self.in_force_mode:
                force = self.robot.get_tool_force(targetRef=drfl.COORDINATE_SYSTEM.Base)
                if force._fForce[2] > self._max_force:
                    self.robot.stop(stop_type=drfl.STOP_TYPE.Slow)
                    logger.warning(f"Robot exceeded too much force in Z axis: "
                                   f"{force._fForce[2]} N")
                    return True
            time.sleep(POLL_INTERVAL)

    # ── Mode / access helpers ──────────────────────────────────────────
    def _wait_for_mode(self, target_mode, timeout: float = 5.0) -> bool:
        start = time.time()
        while time.time() - start < timeout:
            if self.robot.get_robot_mode() == target_mode:
                return True
            time.sleep(0.1)
        return False

    def _ensure_access(self, attempts: int = 10) -> bool:
        for _ in range(attempts):
            if self.get_control_access:
                return True
            logger.info("Re-requesting access control…")
            self.robot.manage_access_control(drfl.MANAGE_ACCESS_CONTROL.Force_request)
            time.sleep(1.0)
        return self.get_control_access

    # ── Register + activate Tool/TCP in Manual mode ────────────────────
    def _register_and_activate_tool_tcp(self) -> bool:
        """
        Switch to Manual, PURGE existing tools/TCPs, then do
        add_tool / add_tcp / set_tool / set_tcp, then switch back to
        Autonomous + Servo_on.
        Returns True only if both set_tool and set_tcp succeeded.
        """
        logger.info(f"[tool/tcp] BEFORE: mode={self.robot.get_robot_mode()} "
                    f"access={self.get_control_access}")

        # 1) → Manual
        self.robot.set_robot_mode(drfl.ROBOT_MODE.Manual)
        if not self._wait_for_mode(drfl.ROBOT_MODE.Manual, timeout=5.0):
            logger.warning("[tool/tcp] failed to enter Manual mode")
        time.sleep(0.5)
        self._ensure_access()
        logger.info(f"[tool/tcp] in Manual: mode={self.robot.get_robot_mode()} "
                    f"access={self.get_control_access}")

        # 1b) Purge anything left over from previous runs
        logger.info("[tool/tcp] purging existing tools and TCPs…")
        self._purge_all_tools_and_tcps()
        time.sleep(0.5)

        # 2) Register tool (idempotent: log + continue if it already exists)
        try:
            ok = self.robot.add_tool(
                strSymbol=TOOL_NAME,
                fWeight=TOOL_WEIGHT,
                fCog=TOOL_CENTER,
                fInertia=TOOL_INERTIA,
            )
            logger.info(f"[tool/tcp] add_tool({TOOL_NAME!r}) -> {ok}")
        except Exception as e:
            logger.warning(f"[tool/tcp] add_tool raised: {e}")

        # 3) Register TCP (idempotent)
        try:
            ok = self.robot.add_tcp(strSymbol=TCP_NAME, fPosition=TCP_OFFSET)
            logger.info(f"[tool/tcp] add_tcp({TCP_NAME!r}) -> {ok}")
        except Exception as e:
            logger.warning(f"[tool/tcp] add_tcp raised: {e}")

        # 4) Activate tool (retry while making sure we're in Manual + have access)
        ok_tool = False
        for attempt in range(10):
            self._ensure_access()
            logger.debug(f"[tool/tcp] set_tool attempt {attempt}: "
                         f"mode={self.robot.get_robot_mode()} "
                         f"access={self.get_control_access}")
            if self.robot.set_tool(TOOL_NAME):
                ok_tool = True
                break
            time.sleep(0.5)
        logger.info(f"[tool/tcp] set_tool({TOOL_NAME!r}) -> {ok_tool}  "
                    f"active tool now: {self.robot.get_tool()!r}")

        # 5) Activate TCP (same retry pattern)
        ok_tcp = False
        for attempt in range(10):
            self._ensure_access()
            logger.debug(f"[tool/tcp] set_tcp attempt {attempt}: "
                         f"mode={self.robot.get_robot_mode()} "
                         f"access={self.get_control_access}")
            try:
                if self.robot.set_tcp(TCP_NAME):
                    ok_tcp = True
                    break
            except Exception as e:
                logger.warning(f"[tool/tcp] set_tcp raised: {e}")
            time.sleep(0.5)
        logger.info(f"[tool/tcp] set_tcp({TCP_NAME!r}) -> {ok_tcp}")

        # 6) Tool shape (also fine in Manual)
        if self.robot.set_tool_shape(TOOL_SHAPE_NAME):
            logger.info(f"[tool/tcp] set_tool_shape({TOOL_SHAPE_NAME!r}) -> OK")
        else:
            logger.warning(f"[tool/tcp] set_tool_shape({TOOL_SHAPE_NAME!r}) failed")

        # 7) → Autonomous
        self.robot.set_robot_mode(drfl.ROBOT_MODE.Autonomous)
        if not self._wait_for_mode(drfl.ROBOT_MODE.Autonomous, timeout=5.0):
            logger.warning("[tool/tcp] failed to return to Autonomous")
        time.sleep(0.5)
        self._ensure_access()

        # 8) Re-enable servo for subsequent moves
        if not self.is_in_standby:
            self.robot.set_robot_control(drfl.ROBOT_CONTROL.Servo_on)
            time.sleep(2.0)

        logger.info(f"[tool/tcp] AFTER: mode={self.robot.get_robot_mode()} "
                    f"access={self.get_control_access} standby={self.is_in_standby} "
                    f"tool={self.robot.get_tool()!r}")
        return ok_tool and ok_tcp

    # ── Setup / teardown ──────────────────────────────────────────────
    def connect(self) -> None:
        self.robot.set_on_monitoring_access_control(self._on_monitoring_access_control)
        self.robot.set_on_monitoring_state(self._on_monitoring_state)

        logger.debug(f"Connecting to robot at {self.ip_address}:{self.port}")
        if not self.robot.open_connection(self.ip_address, port=self.port):
            raise RuntimeError(
                f"Cannot open connection to robot at {self.ip_address}:{self.port}")

        # Wait for standby or safe off
        start = time.time()
        while not self.is_in_standby and not self.is_in_safe_off:
            if time.time() - start > MOVE_TIMEOUT:
                raise RuntimeError("Timed out waiting for robot ready state")
            time.sleep(POLL_INTERVAL)

        self.robot.setup_monitoring_version(1)

        version = drfl.SYSTEM_VERSION()
        self.robot.get_system_version(version)
        logger.info(f"Controller (DRCF) version: {version._szController}")
        logger.info(f"Library version: {self.robot.get_library_version()}")

        # ── Step 1: gain access control + servo on (still in default mode) ──
        for attempt in range(10):
            logger.debug(f"Attempt {attempt}: gaining access control and servo on")
            if not self.get_control_access:
                self.robot.manage_access_control(drfl.MANAGE_ACCESS_CONTROL.Force_request)
                time.sleep(1.0)
                continue
            if not self.is_in_standby:
                self.robot.set_robot_control(drfl.ROBOT_CONTROL.Servo_on)
                time.sleep(2.0)
                continue
            break

        if not (self.get_control_access and self.is_in_standby):
            raise RuntimeError(
                f"Failed to reach intended state — "
                f"access={self.get_control_access}, standby={self.is_in_standby}"
            )

        # ── Step 2: purge + register + activate tool/TCP via Manual round-trip ──
        if not self._register_and_activate_tool_tcp():
            logger.error("Tool/TCP activation failed — continuing without it")

        # ── Step 3: final mode + system ──
        if not self.robot.set_robot_mode(drfl.ROBOT_MODE.Autonomous):
            raise RuntimeError("Failed setting robot mode to Autonomous")
        if not self.robot.set_robot_system(drfl.ROBOT_SYSTEM.Real):
            raise RuntimeError("Failed setting robot system to Real")

        logger.info(f"Active tool: {self.robot.get_tool()!r}")

    def disconnect(self) -> None:
        """Switch to Manual before deleting tool/TCP, then close."""
        try:
            self.robot.set_robot_mode(drfl.ROBOT_MODE.Manual)
            self._wait_for_mode(drfl.ROBOT_MODE.Manual, timeout=5.0)
            time.sleep(0.5)
            self._ensure_access()
            try:
                ok = self.robot.del_tcp(TCP_NAME)
                logger.info(f"[disconnect] del_tcp({TCP_NAME!r}) -> {ok}")
            except Exception as e:
                logger.warning(f"[disconnect] del_tcp raised: {e}")
            try:
                ok = self.robot.del_tool(TOOL_NAME)
                logger.info(f"[disconnect] del_tool({TOOL_NAME!r}) -> {ok}")
            except Exception as e:
                logger.warning(f"[disconnect] del_tool raised: {e}")
        except Exception as e:
            logger.warning(f"[disconnect] cleanup failed: {e}")

        self.robot.close_connection()
        logger.info("Connection closed")

    # ── Command loop ───────────────────────────────────────────────────
    def run(self, command_queue: mp.Queue, result_queue: mp.Queue) -> None:
        try:
            self.connect()
        except RuntimeError as e:
            result_queue.put(f"error:{e}")
            return

        result_queue.put("ready")

        while True:
            command = command_queue.get()

            if command is None:
                logger.info("Shutdown command received")
                break

            if len(command) == 2:
                move_type, pose_array = command
                output = None
            elif len(command) == 3:
                move_type, pose_array, output = command
            else:
                continue

            if move_type == "toggle_air":
                if output is None:
                    if self.robot.get_digital_output(pose_array):
                        self.robot.set_digital_output(pose_array, False)
                    else:
                        self.robot.set_digital_output(pose_array, True)
                else:
                    self.robot.set_digital_output(pose_array, output)
                result_queue.put("done")
                continue

            if move_type == "force":
                if not self._enable_compliance():
                    result_queue.put("error:Failed to enable compliance control")
                    continue
                if isinstance(output, float):
                    self._max_force = output
                if not self._amove_force(pose_array):
                    self._disable_compliance()
                    result_queue.put("error:Force move command failed")
                    continue
                if not self.is_moving(MOVE_TIMEOUT):
                    self._disable_compliance()
                    result_queue.put("error:Force movement timed out")
                    continue
                contact_pose = self.robot.get_current_pose(drfl.ROBOT_SPACE.Task)
                self._disable_compliance()
                result_queue.put(("force_done", list(contact_pose._fPosition)))
                continue

            handler = self.move_handlers.get(move_type)
            if handler is None:
                result_queue.put(f"error:Unknown move type: {move_type}")
                continue

            if move_type == "movejx":
                if not handler(pose_array, output):
                    result_queue.put("error: Movement timed out")
                    continue
            elif move_type == "linear" and len(command) == 3 and output is not None:
                logger.info(f"Doing custom move with speed={output[0]}, acc={output[1]}")
                if not handler(pose_array, speed=output[0], acc=output[1]):
                    result_queue.put("error:Move command failed")
                    continue
            else:
                if not handler(pose_array):
                    result_queue.put("error:Move command failed")
                    continue

            if not self.is_moving(MOVE_TIMEOUT):
                result_queue.put("error: Movement timed out")
                continue

            result_queue.put("done")

        self.disconnect()


def robot_worker(
    command_queue: mp.Queue,
    result_queue: mp.Queue,
    ip_address: str,
    port: int,
    speed: float = 20,
    acceleration: float = 10,
    lin_speed: float = 60,
    lin_acceleration: float = 35,
) -> None:
    controller = RobotController(
        ip_address, port,
        speed=speed, acceleration=acceleration,
        lin_speed=lin_speed, lin_acceleration=lin_acceleration,
    )
    controller.run(command_queue, result_queue)
