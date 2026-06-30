#!/usr/bin/env python3
"""
Robot controller class.

Owns the robot connection and executes commands received via command_queue.
Sends "ready", "done", "error:<msg>" (or a tuple for force results) via
result_queue. Shuts down cleanly when it receives None on command_queue.

Supported command tuples (put on command_queue):

    ("joint",  pose_array)
    ("linear", pose_array)                       # default lin speed/acc
    ("linear", pose_array, (speed, acc))         # custom lin speed/acc
    ("movejx", pose_array, solution_space)
    ("force",  pose_array)                       # uses MAX_FORCE_BOX
    ("force",  pose_array, max_force_float)
    ("toggle_air", index)                        # toggle current state
    ("toggle_air", index, bool)                  # set explicit state
    ("set_tool", tool_name)                      # switch tool + matching TCP
    ("set_tcp",  tcp_name)                       # switch TCP only

Send None to shut down.
"""

import doosan_drfl as drfl
import multiprocessing as mp
import logging
import time
import numpy as np

logger = logging.getLogger(__name__)

MOVE_TIMEOUT  = 120.0   # seconds
POLL_INTERVAL = 0.01    # seconds

# ── Tool configuration ────────────────────────────────────────────────
TOOL_SHAPE_NAME = "Tool_Shape"

# Each tool: (weight_kg, cog_mm[3], inertia[6])
TOOLS = {
    "default": (
        0.5,
        np.array([21.670, -2.260, 51.880], dtype=np.float32),
        np.array([0, 0, 0, 0, 0, 0],       dtype=np.float32),
    ),
    "snoeks1": (
        0.5,
        np.array([21.670, -2.260, 51.880], dtype=np.float32),
        np.array([0, 0, 0, 0, 0, 0],       dtype=np.float32),
    ),
    "snoeks_paralell": (
        # TODO: replace with the real mass / COG / inertia of the
        # parallel-jaw gripper. For now mirrors the suction tool so
        # gravity comp doesn't go wildly wrong.
        0.5,
        np.array([21.670, -2.260, 51.880], dtype=np.float32),
        np.array([0, 0, 0, 0, 0, 0],       dtype=np.float32),
    ),
}
DEFAULT_TOOL = "snoeks1"

# Each TCP: [x_mm, y_mm, z_mm, a_deg, b_deg, c_deg] — geometric tip offset
TCPS = {
    "default":         np.array([   0.000,  0.000,   0.000, 0.0, 0.0, 0.0], dtype=np.float32),
    "snoeks1":         np.array([ -43.032,  1.759,  93.887, 0.0, 0.0, 0.0], dtype=np.float32),
    "snoeks_paralell": np.array([ 139.070, -0.574, 183.668, 0.0, -45.0, 0.0], dtype=np.float32),
}

MAX_FORCE_BOX = 5.5

# Compliance control — stiffness per axis [X, Y, Z, Rx, Ry, Rz]
COMPLIANCE_STIFFNESS = np.array(
    [2500.0, 2500.0, 1500.0, 200.0, 200.0, 200.0], dtype=np.float32
)

# Force probe — apply 5 N along +Z by default
FORCE_PROBE_FD  = np.array([0.0, 0.0, 5.0, 0.0, 0.0, 0.0], dtype=np.float32)
FORCE_PROBE_DIR = np.array([0,   0,    1,   0,   0,   0  ], dtype=np.uint8)


class RobotController:
    """Owns the robot connection and executes commands from a queue."""

    def __init__(
        self,
        ip_address: str,
        port: int,
        speed: float = 20.0,
        acceleration: float = 10.0,
        lin_speed: float = 20.0,
        lin_acceleration: float = 25.0,
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

        # State flags updated by callbacks
        self.get_control_access: bool = False
        self.is_in_standby:      bool = False
        self.is_in_safe_off:     bool = False

        self.move_handlers = {
            "joint":  self._amovej,
            "linear": self._amovel,
            "movejx": self._ammovejx,
        }

        self.in_force_mode: bool = False

    # ── Callbacks ─────────────────────────────────────────────────────
    def _on_monitoring_access_control(self, access: drfl.MONITORING_ACCESS_CONTROL) -> None:
        logger.debug(f"[on_monitoring_access_control] "
                    f"{drfl.MONITORING_ACCESS_CONTROL(access).name} [{access}]")
        if access == drfl.MONITORING_ACCESS_CONTROL.Grant:
            logger.info("Access granted!")
            self.get_control_access = True
        elif access == drfl.MONITORING_ACCESS_CONTROL.Loss:
            logger.info("Access lost!")
            self.get_control_access = False

    def _on_monitoring_state(self, state: drfl.ROBOT_STATE) -> None:
        logger.debug(f"[on_monitoring_state] "
                    f"{drfl.ROBOT_STATE(state).name} [{state}]")
        self.is_in_standby  = (state == drfl.ROBOT_STATE.Standby)
        self.is_in_safe_off = (state == drfl.ROBOT_STATE.Safe_off)

    # ── Move primitives ───────────────────────────────────────────────
    def _amovej(self, pose, speed=None, acc=None) -> bool:
        return self.robot.amovej(
            pos=pose,
            vel=speed if speed is not None else self.speed,
            acc=acc   if acc   is not None else self.acceleration,
            time=0.0,
            move_mode=drfl.MOVE_MODE.Absolute,
            blending_type=drfl.BLENDING_SPEED_TYPE.Duplicate,
        )

    def _amovel(self, pose, speed=None, acc=None) -> bool:
        velocity     = np.array([speed if speed is not None else self.lin_speed, 0],
                                dtype=np.float32)
        acceleration = np.array([acc   if acc   is not None else self.lin_acceleration, 0],
                                dtype=np.float32)
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

    # ── Compliance / force ────────────────────────────────────────────
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

    # ── Motion polling ────────────────────────────────────────────────
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
                    logger.warning(f"Exceeded Z force limit: {force._fForce[2]} N")
                    return True

            time.sleep(POLL_INTERVAL)

    # ── Mode / access helpers ─────────────────────────────────────────
    def _wait_for_mode(self, target: "drfl.ROBOT_MODE", timeout: float = 5.0) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.robot.get_robot_mode() == target:
                return True
            time.sleep(0.1)
        return False

    def _ensure_access(self, attempts: int = 10) -> bool:
        for _ in range(attempts):
            if self.get_control_access:
                return True
            self.robot.manage_access_control(drfl.MANAGE_ACCESS_CONTROL.Force_request)
            time.sleep(1.0)
        return self.get_control_access

    def _ensure_servo_on(self, timeout: float = 5.0) -> bool:
        if self.is_in_standby:
            return True
        self.robot.set_robot_control(drfl.ROBOT_CONTROL.Servo_on)
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_in_standby:
                return True
            time.sleep(0.1)
        return self.is_in_standby

    # ── Tool / TCP switching ──────────────────────────────────────────
    def _switch_tool_tcp(self,
                        tool_name: str | None,
                        tcp_name: str | None) -> tuple[bool, bool]:
        """
        Switch the active tool and/or TCP.

        Requires Manual mode + access control. Returns (ok_tool, ok_tcp).
        Pass None for either to skip switching that one (returns True for
        the skipped slot).
        """
        logger.info(f"[switch] requested tool={tool_name!r} tcp={tcp_name!r}")
        logger.info(f"[switch] BEFORE: mode={self.robot.get_robot_mode()} "
                    f"access={self.get_control_access} "
                    f"active_tool={self.robot.get_tool()!r}")

        # 1) Enter Manual
        self.robot.set_robot_mode(drfl.ROBOT_MODE.Manual)
        self._wait_for_mode(drfl.ROBOT_MODE.Manual)
        time.sleep(0.3)
        self._ensure_access()

        # 2) set_tool
        if tool_name is None:
            ok_tool = True
        else:
            try:
                ok_tool = bool(self.robot.set_tool(tool_name))
            except Exception as e:
                logger.warning(f"set_tool raised: {e}")
                ok_tool = False
            logger.info(f"[switch] set_tool({tool_name!r}) -> {ok_tool}  "
                        f"active tool: {self.robot.get_tool()!r}")

        # 3) set_tcp
        if tcp_name is None:
            ok_tcp = True
        else:
            try:
                ok_tcp = bool(self.robot.set_tcp(tcp_name))
            except Exception as e:
                logger.warning(f"set_tcp raised: {e}")
                ok_tcp = False
            logger.info(f"[switch] set_tcp({tcp_name!r}) -> {ok_tcp}")

        # 4) Back to Autonomous + access + servo on
        self.robot.set_robot_mode(drfl.ROBOT_MODE.Autonomous)
        self._wait_for_mode(drfl.ROBOT_MODE.Autonomous)
        time.sleep(0.3)
        self._ensure_access()
        self._ensure_servo_on()

        logger.info(f"[switch] AFTER: mode={self.robot.get_robot_mode()} "
                    f"access={self.get_control_access} "
                    f"standby={self.is_in_standby} "
                    f"active_tool={self.robot.get_tool()!r}")
        return ok_tool, ok_tcp

    # ── Setup / teardown ──────────────────────────────────────────────
    def connect(self) -> None:
        self.robot.set_on_monitoring_access_control(self._on_monitoring_access_control)
        self.robot.set_on_monitoring_state(self._on_monitoring_state)

        logger.debug(f"Connecting to robot at {self.ip_address}:{self.port}")
        if not self.robot.open_connection(self.ip_address, port=self.port):
            raise RuntimeError(
                f"Cannot open connection to robot at {self.ip_address}:{self.port}")

        # Wait for ready state
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

        # Access control + servo on
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
                f"access={self.get_control_access}, standby={self.is_in_standby}")

        # Register every tool and TCP (must be in Manual)
        self.robot.set_robot_mode(drfl.ROBOT_MODE.Manual)
        self._wait_for_mode(drfl.ROBOT_MODE.Manual)
        time.sleep(0.3)
        self._ensure_access()

        for tname, (w, cog, inertia) in TOOLS.items():
            try:
                ok = self.robot.add_tool(strSymbol=tname, fWeight=w,
                                        fCog=cog, fInertia=inertia)
                logger.info(f"add_tool({tname!r}) -> {ok}")
            except Exception as e:
                logger.warning(f"add_tool({tname!r}) raised: {e}")

        for tname, offset in TCPS.items():
            try:
                ok = self.robot.add_tcp(strSymbol=tname, fPosition=offset)
                logger.info(f"add_tcp({tname!r}) -> {ok}")
            except Exception as e:
                logger.warning(f"add_tcp({tname!r}) raised: {e}")

        if self.robot.set_tool_shape(TOOL_SHAPE_NAME):
            logger.info(f"Set tool shape: {TOOL_SHAPE_NAME}")
        else:
            logger.warning(f"Failed to set tool shape: {TOOL_SHAPE_NAME}")

        # Back to Autonomous for normal operation
        self.robot.set_robot_mode(drfl.ROBOT_MODE.Autonomous)
        self._wait_for_mode(drfl.ROBOT_MODE.Autonomous)
        time.sleep(0.3)
        self._ensure_access()
        self._ensure_servo_on()

        if not self.robot.set_robot_system(drfl.ROBOT_SYSTEM.Real):
            raise RuntimeError("Failed setting robot system to Real")

        # Activate the default tool + matching TCP
        self._switch_tool_tcp(DEFAULT_TOOL, DEFAULT_TOOL)
        logger.info(f"Active tool: {self.robot.get_tool()!r}")

    def disconnect(self) -> None:
        try:
            self.robot.set_robot_mode(drfl.ROBOT_MODE.Manual)
            self._wait_for_mode(drfl.ROBOT_MODE.Manual)
            logger.info(f"[disconnect] mode now: {self.robot.get_robot_mode()}")

            for tname in TOOLS:
                try:
                    ok = self.robot.del_tool(tname)
                    logger.info(f"[disconnect] del_tool({tname!r}) -> {ok}")
                except Exception as e:
                    logger.warning(f"[disconnect] del_tool({tname!r}) raised: {e}")

            for tname in TCPS:
                try:
                    ok = self.robot.del_tcp(tname)
                    logger.info(f"[disconnect] del_tcp({tname!r}) -> {ok}")
                except Exception as e:
                    logger.warning(f"[disconnect] del_tcp({tname!r}) raised: {e}")
        except Exception as e:
            logger.warning(f"[disconnect] cleanup failed: {e}")

        self.robot.close_connection()
        logger.info("Connection closed")

    # ── Command loop ──────────────────────────────────────────────────
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

            # ── DIGITAL OUTPUT ───────────────────────────────────────
            if move_type == "toggle_air":
                if output is None:
                    cur = self.robot.get_digital_output(pose_array)
                    self.robot.set_digital_output(pose_array, not cur)
                else:
                    self.robot.set_digital_output(pose_array, output)
                result_queue.put("done")
                continue

            # ── SET TOOL (+ matching TCP) ────────────────────────────
            if move_type == "set_tool":
                tool_name = pose_array
                logger.info(f"[set_tool] requested: {tool_name!r}")
                if tool_name not in TOOLS:
                    result_queue.put(f"error:unknown tool {tool_name}")
                    continue
                tcp_name = tool_name if tool_name in TCPS else None
                ok_tool, ok_tcp = self._switch_tool_tcp(tool_name, tcp_name)
                if not (ok_tool and ok_tcp):
                    result_queue.put(
                        f"error:set_tool/tcp failed (tool={ok_tool}, tcp={ok_tcp})")
                    continue
                result_queue.put("done")
                continue

            # ── SET TCP only ─────────────────────────────────────────
            if move_type == "set_tcp":
                tcp_name = pose_array
                logger.info(f"[set_tcp] requested: {tcp_name!r}")
                if tcp_name not in TCPS:
                    result_queue.put(f"error:unknown tcp {tcp_name}")
                    continue
                _, ok_tcp = self._switch_tool_tcp(None, tcp_name)
                if not ok_tcp:
                    result_queue.put(f"error:set_tcp failed for {tcp_name}")
                    continue
                result_queue.put("done")
                continue

            # ── FORCE MOVE ───────────────────────────────────────────
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

            # ── REGULAR MOVES ────────────────────────────────────────
            handler = self.move_handlers.get(move_type)
            if handler is None:
                result_queue.put(f"error:Unknown move type: {move_type}")
                continue

            if move_type == "movejx":
                if not handler(pose_array, output):
                    result_queue.put("error: Movement timed out")
                    continue
            elif move_type == "linear" and len(command) == 3 and output is not None:
                logger.info(f"Custom linear move: speed={output[0]}, acc={output[1]}")
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
