import math
import os
import configparser
from collections import deque

import carla
import pygame
from pygame.locals import (
    KMOD_CTRL,
    K_ESCAPE,
    K_q,
    K_SPACE,
    K_UP,
    K_DOWN,
    K_LEFT,
    K_RIGHT,
    K_w,
    K_a,
    K_s,
    K_d,
)


class DualControl(object):
    """
    Unified manual control for:
      1) keyboard
      2) generic joystick / gamepad
      3) Logitech-style steering wheel + pedals (e.g., G29)

    Design goals:
      - realistic steering buildup and return
      - realistic throttle/brake shaping
      - smooth acceleration/deceleration transitions
      - speed-sensitive steering
      - device abstraction so rest of CARLA loop stays unchanged

    Expected use:
        controller = DualControl(world, start_in_autopilot=False)
        ...
        while True:
            if controller.parse_events(world, clock):
                return
    """

    def __init__(self, world, start_in_autopilot, device_preference="auto",
                 wheel_config_path="wheel_config.ini"):
        self._autopilot_enabled = start_in_autopilot
        self._control = carla.VehicleControl()
        self._control.manual_gear_shift = False
        self._control.gear = 1

        self._steer_cache = 0.0
        self._throttle_cache = 0.0
        self._brake_cache = 0.0
        self._reverse_lock = False

        self._last_speed_mps = 0.0
        self._speed_samples = deque(maxlen=5)

        self._device_mode = "keyboard"   # keyboard / joystick / wheel
        self._active_joystick = None
        self._device_preference = device_preference

        # ---------------------------------------
        # Realism / shaping parameters
        # ---------------------------------------
        self._steer_deadzone = 0.04
        self._throttle_deadzone = 0.03
        self._brake_deadzone = 0.03

        # Wheel / joystick steering response
        self._steer_rate = 2.8              # per second, rise toward target
        self._steer_return_rate = 4.8       # per second, stronger centering
        self._steer_max_low_speed = 0.95
        self._steer_max_high_speed = 0.35
        self._steer_speed_sensitivity_kph = 70.0

        # Throttle realism
        self._throttle_rise_rate = 1.15     # slower pedal-up -> realistic accel
        self._throttle_fall_rate = 3.20     # fast lift-off
        self._throttle_gamma = 1.65         # non-linear pedal mapping

        # Brake realism
        self._brake_rise_rate = 3.80
        self._brake_fall_rate = 5.00
        self._brake_gamma = 1.35

        # Light engine drag / coasting feel when no throttle and no brake
        self._coast_brake_strength = 0.08
        self._coast_speed_threshold_kph = 5.0

        # Keyboard-only parameters
        self._kbd_steer_increment = 2.2
        self._kbd_steer_return = 5.2
        self._kbd_throttle_increment = 1.10
        self._kbd_brake_increment = 3.60

        # ---------------------------------------
        # Wheel config defaults
        # ---------------------------------------
        self._parser = configparser.ConfigParser()
        self._wheel_cfg_path = wheel_config_path
        self._wheel_cfg_loaded = False

        # Default axis/button mapping (can be overridden by wheel_config.ini)
        self._steer_idx = 0
        self._throttle_idx = 2
        self._brake_idx = 3
        self._reverse_idx = 5
        self._handbrake_idx = 4

        # Whether wheel pedals are inverted:
        # many Logitech pedals report 1.0 released -> -1.0 pressed or vice versa.
        self._invert_throttle = True
        self._invert_brake = True

        # Whether this joystick should be treated as a wheel
        self._force_wheel_names = ("g29", "g920", "g923", "driving force", "logitech")

        pygame.joystick.init()
        self._setup_input_device()
        self._load_wheel_config_if_available()

        if world.player is not None:
            world.player.set_autopilot(self._autopilot_enabled)

    # -------------------------------------------------------------------------
    # Public interface
    # -------------------------------------------------------------------------
    def parse_events(self, world, clock):
        """
        Returns True if quit shortcut is requested, else False.
        """
        if world.player is None:
            return False

        milliseconds = max(clock.get_time(), 1)
        dt = milliseconds / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return True

            elif event.type == pygame.JOYDEVICEADDED:
                self._setup_input_device()
                self._load_wheel_config_if_available()

            elif event.type == pygame.JOYDEVICEREMOVED:
                self._setup_input_device()
                self._load_wheel_config_if_available()

            elif event.type == pygame.KEYUP:
                if self._is_quit_shortcut(event.key):
                    return True

                # Toggle autopilot
                elif event.key == pygame.K_p:
                    self._autopilot_enabled = not self._autopilot_enabled
                    world.player.set_autopilot(self._autopilot_enabled)

                # Reverse toggle
                elif event.key == pygame.K_r:
                    self._reverse_lock = not self._reverse_lock
                    self._control.reverse = self._reverse_lock

                # Manual handbrake
                elif event.key == K_SPACE:
                    self._control.hand_brake = False

            elif event.type == pygame.KEYDOWN:
                if event.key == K_SPACE:
                    self._control.hand_brake = True

        if self._autopilot_enabled:
            return False

        keys = pygame.key.get_pressed()

        if isinstance(self._control, carla.VehicleControl):
            if self._device_mode == "wheel":
                self._parse_vehicle_wheel(world, dt, keys)
            elif self._device_mode == "joystick":
                self._parse_vehicle_joystick(world, dt, keys)
            else:
                self._parse_vehicle_keys(world, keys, dt)

            self._apply_vehicle_control(world)

        else:
            self._parse_walker_keys(keys, milliseconds)

        return False

    # -------------------------------------------------------------------------
    # Setup helpers
    # -------------------------------------------------------------------------
    def _setup_input_device(self):
        """
        Detect and initialize best available input device.

        Priority:
          - explicit preference
          - steering wheel if recognized
          - generic joystick/gamepad
          - keyboard fallback
        """
        pygame.joystick.quit()
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        self._active_joystick = None
        self._device_mode = "keyboard"

        if joystick_count == 0:
            return

        devices = []
        for i in range(joystick_count):
            js = pygame.joystick.Joystick(i)
            js.init()
            name = js.get_name().lower()
            devices.append((i, js, name))

        # Explicit preference
        if self._device_preference == "wheel":
            for _, js, name in devices:
                if any(tag in name for tag in self._force_wheel_names):
                    self._active_joystick = js
                    self._device_mode = "wheel"
                    return

        if self._device_preference == "joystick":
            self._active_joystick = devices[0][1]
            self._device_mode = "joystick"
            return

        if self._device_preference == "keyboard":
            self._device_mode = "keyboard"
            return

        # Auto-detect
        for _, js, name in devices:
            if any(tag in name for tag in self._force_wheel_names):
                self._active_joystick = js
                self._device_mode = "wheel"
                return

        self._active_joystick = devices[0][1]
        self._device_mode = "joystick"

    def _load_wheel_config_if_available(self):
        """
        Optional wheel config:
            [G29 Racing Wheel]
            steering_wheel = 0
            throttle = 2
            brake = 3
            reverse = 5
            handbrake = 4
            invert_throttle = true
            invert_brake = true
        """
        self._wheel_cfg_loaded = False

        if self._device_mode != "wheel":
            return

        if not os.path.exists(self._wheel_cfg_path):
            return

        self._parser.read(self._wheel_cfg_path)

        section_name = None
        possible_sections = [
            "G29 Racing Wheel",
            "Steering Wheel",
            "Wheel",
        ]

        for sec in possible_sections:
            if self._parser.has_section(sec):
                section_name = sec
                break

        if section_name is None:
            return

        cfg = self._parser[section_name]

        self._steer_idx = int(cfg.get("steering_wheel", self._steer_idx))
        self._throttle_idx = int(cfg.get("throttle", self._throttle_idx))
        self._brake_idx = int(cfg.get("brake", self._brake_idx))
        self._reverse_idx = int(cfg.get("reverse", self._reverse_idx))
        self._handbrake_idx = int(cfg.get("handbrake", self._handbrake_idx))
        self._invert_throttle = cfg.getboolean("invert_throttle", fallback=self._invert_throttle)
        self._invert_brake = cfg.getboolean("invert_brake", fallback=self._invert_brake)

        self._wheel_cfg_loaded = True

    # -------------------------------------------------------------------------
    # Keyboard control
    # -------------------------------------------------------------------------
    def _parse_vehicle_keys(self, world, keys, dt):
        """
        Keyboard control with gradual realism, not binary on/off.
        """
        speed_kph = self._get_speed_kph(world)

        # -------------------------
        # Steering
        # -------------------------
        steer_input = 0.0
        if keys[K_LEFT] or keys[K_a]:
            steer_input -= 1.0
        if keys[K_RIGHT] or keys[K_d]:
            steer_input += 1.0

        self._update_steering_from_target(steer_input, dt, speed_kph)

        # -------------------------
        # Throttle / brake
        # -------------------------
        throttle_pressed = keys[K_UP] or keys[K_w]
        brake_pressed = keys[K_DOWN] or keys[K_s]

        throttle_target = 1.0 if throttle_pressed else 0.0
        brake_target = 1.0 if brake_pressed else 0.0

        # If actively braking, kill throttle faster
        if brake_target > 0.0:
            throttle_target = 0.0

        self._throttle_cache = self._approach(
            self._throttle_cache,
            throttle_target,
            self._kbd_throttle_increment * dt,
            self._throttle_fall_rate * dt
        )

        self._brake_cache = self._approach(
            self._brake_cache,
            brake_target,
            self._kbd_brake_increment * dt,
            self._brake_fall_rate * dt
        )

        self._control.hand_brake = bool(keys[K_SPACE])

        # Reverse logic
        self._resolve_reverse_state(speed_kph, throttle_pressed, brake_pressed)

    # -------------------------------------------------------------------------
    # Generic joystick / gamepad control
    # -------------------------------------------------------------------------
    def _parse_vehicle_joystick(self, world, dt, keys):
        """
        Generic joystick/gamepad mapping.

        Assumptions:
          axis 0: steer
          axis 5 or 4: throttle trigger if present
          axis 2 or 3: brake trigger if present
        """
        if self._active_joystick is None:
            self._parse_vehicle_keys(world, keys, dt)
            return

        speed_kph = self._get_speed_kph(world)
        js = self._active_joystick

        axis_count = js.get_numaxes()
        button_count = js.get_numbuttons()

        # Steering
        raw_steer = js.get_axis(0) if axis_count > 0 else 0.0
        raw_steer = self._apply_deadzone(raw_steer, self._steer_deadzone)
        steer_target = self._shape_steering(raw_steer)
        self._update_steering_from_target(steer_target, dt, speed_kph)

        # Triggers / fallback axes
        raw_throttle = 0.0
        raw_brake = 0.0

        # Common layouts
        if axis_count >= 6:
            raw_throttle = self._trigger_to_unit(js.get_axis(5))
            raw_brake = self._trigger_to_unit(js.get_axis(4))
        elif axis_count >= 4:
            raw_throttle = self._trigger_to_unit(js.get_axis(2))
            raw_brake = self._trigger_to_unit(js.get_axis(3))
        else:
            # Fallback to buttons if analog unavailable
            raw_throttle = 1.0 if (button_count > 0 and js.get_button(0)) else 0.0
            raw_brake = 1.0 if (button_count > 1 and js.get_button(1)) else 0.0

        throttle_target = self._shape_pedal(raw_throttle, self._throttle_deadzone, self._throttle_gamma)
        brake_target = self._shape_pedal(raw_brake, self._brake_deadzone, self._brake_gamma)

        if brake_target > 0.02:
            throttle_target = 0.0

        self._throttle_cache = self._approach(
            self._throttle_cache,
            throttle_target,
            self._throttle_rise_rate * dt,
            self._throttle_fall_rate * dt
        )

        self._brake_cache = self._approach(
            self._brake_cache,
            brake_target,
            self._brake_rise_rate * dt,
            self._brake_fall_rate * dt
        )

        # Optional buttons
        handbrake = False
        reverse_button = False
        if button_count > 4:
            handbrake = bool(js.get_button(4))
        if button_count > 5:
            reverse_button = bool(js.get_button(5))

        self._control.hand_brake = handbrake

        if reverse_button:
            self._reverse_lock = True
        elif self._throttle_cache > 0.05:
            self._reverse_lock = False

        self._resolve_reverse_state(speed_kph,
                                    throttle_pressed=(self._throttle_cache > 0.02),
                                    brake_pressed=(self._brake_cache > 0.02))

    # -------------------------------------------------------------------------
    # Steering wheel + pedal control
    # -------------------------------------------------------------------------
    def _parse_vehicle_wheel(self, world, dt, keys):
        """
        Logitech-style wheel and pedal mapping.

        Uses wheel_config.ini if available, otherwise defaults.
        """
        if self._active_joystick is None:
            self._parse_vehicle_keys(world, keys, dt)
            return

        speed_kph = self._get_speed_kph(world)
        js = self._active_joystick

        axis_count = js.get_numaxes()
        button_count = js.get_numbuttons()

        # Safe axis reads
        def get_axis(idx, default=0.0):
            return js.get_axis(idx) if idx < axis_count else default

        def get_button(idx, default=0):
            return js.get_button(idx) if idx < button_count else default

        # -------------------------
        # Steering wheel
        # -------------------------
        raw_wheel = get_axis(self._steer_idx, 0.0)

        # Some wheels are already centered around 0; shape near center for precision
        raw_wheel = self._apply_deadzone(raw_wheel, self._steer_deadzone)
        steer_target = self._shape_steering(raw_wheel, expo=1.9)

        self._update_steering_from_target(steer_target, dt, speed_kph)

        # -------------------------
        # Pedals
        # -------------------------
        raw_throttle_axis = get_axis(self._throttle_idx, 1.0 if self._invert_throttle else 0.0)
        raw_brake_axis = get_axis(self._brake_idx, 1.0 if self._invert_brake else 0.0)

        raw_throttle = self._wheel_axis_to_unit(raw_throttle_axis, invert=self._invert_throttle)
        raw_brake = self._wheel_axis_to_unit(raw_brake_axis, invert=self._invert_brake)

        throttle_target = self._shape_pedal(raw_throttle, self._throttle_deadzone, self._throttle_gamma)
        brake_target = self._shape_pedal(raw_brake, self._brake_deadzone, self._brake_gamma)

        # Brake wins over throttle
        if brake_target > 0.03:
            throttle_target = 0.0

        self._throttle_cache = self._approach(
            self._throttle_cache,
            throttle_target,
            self._throttle_rise_rate * dt,
            self._throttle_fall_rate * dt
        )

        self._brake_cache = self._approach(
            self._brake_cache,
            brake_target,
            self._brake_rise_rate * dt,
            self._brake_fall_rate * dt
        )

        self._control.hand_brake = bool(get_button(self._handbrake_idx, 0))

        reverse_pressed = bool(get_button(self._reverse_idx, 0))
        if reverse_pressed:
            self._reverse_lock = True
        elif self._throttle_cache > 0.05:
            self._reverse_lock = False

        self._resolve_reverse_state(speed_kph,
                                    throttle_pressed=(self._throttle_cache > 0.02),
                                    brake_pressed=(self._brake_cache > 0.02))

    # -------------------------------------------------------------------------
    # Apply control
    # -------------------------------------------------------------------------
    def _apply_vehicle_control(self, world):
        """
        Final realism adjustments before applying to CARLA.
        """
        speed_kph = self._get_speed_kph(world)

        throttle = float(self._throttle_cache)
        brake = float(self._brake_cache)
        steer = float(self._steer_cache)

        # Small coasting drag feel
        if throttle < 0.01 and brake < 0.01 and speed_kph > self._coast_speed_threshold_kph:
            brake = max(brake, self._coast_brake_strength)

        # Prevent simultaneous high throttle + brake
        if brake > 0.05:
            throttle = 0.0

        self._control.throttle = self._clamp(throttle, 0.0, 1.0)
        self._control.brake = self._clamp(brake, 0.0, 1.0)
        self._control.steer = self._clamp(steer, -1.0, 1.0)

        world.player.apply_control(self._control)

    # -------------------------------------------------------------------------
    # Walker placeholder
    # -------------------------------------------------------------------------
    def _parse_walker_keys(self, keys, milliseconds):
        # Keep your walker logic here if needed
        pass

    # -------------------------------------------------------------------------
    # Steering / throttle / brake helpers
    # -------------------------------------------------------------------------
    def _update_steering_from_target(self, target, dt, speed_kph):
        """
        Smooth steering toward target with return-to-center and speed sensitivity.
        """
        target = self._clamp(target, -1.0, 1.0)

        max_steer = self._speed_sensitive_steer_limit(speed_kph)
        target *= max_steer

        if abs(target) < 1e-4:
            # return to center faster than buildup
            step = self._steer_return_rate * dt
            self._steer_cache = self._move_toward(self._steer_cache, 0.0, step)
        else:
            step = self._steer_rate * dt
            self._steer_cache = self._move_toward(self._steer_cache, target, step)

    def _shape_steering(self, x, expo=1.6):
        """
        Symmetric exponential response:
          small around center, stronger toward edges.
        """
        sign = -1.0 if x < 0 else 1.0
        x = abs(x)
        return sign * (x ** expo)

    def _shape_pedal(self, x, deadzone, gamma):
        x = self._apply_deadzone(x, deadzone)
        return self._clamp(x ** gamma, 0.0, 1.0)

    def _speed_sensitive_steer_limit(self, speed_kph):
        """
        Reduce available steering range as speed rises.
        """
        t = self._clamp(speed_kph / self._steer_speed_sensitivity_kph, 0.0, 1.0)
        return self._steer_max_low_speed + (self._steer_max_high_speed - self._steer_max_low_speed) * t

    def _resolve_reverse_state(self, speed_kph, throttle_pressed, brake_pressed):
        """
        Realistic reverse engagement logic:
          - if explicitly reverse-locked -> reverse
          - if nearly stopped and brake is held without throttle -> allow reverse
          - otherwise forward
        """
        if self._reverse_lock:
            self._control.reverse = True
            self._control.gear = -1
            return

        if speed_kph < 1.0 and brake_pressed and not throttle_pressed:
            self._control.reverse = True
            self._control.gear = -1
        else:
            self._control.reverse = False
            self._control.gear = 1

    # -------------------------------------------------------------------------
    # Utility helpers
    # -------------------------------------------------------------------------
    def _get_speed_kph(self, world):
        vel = world.player.get_velocity()
        speed_mps = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
        self._last_speed_mps = speed_mps
        self._speed_samples.append(speed_mps)
        speed_mps_smooth = sum(self._speed_samples) / max(len(self._speed_samples), 1)
        return speed_mps_smooth * 3.6

    @staticmethod
    def _clamp(x, lo, hi):
        return max(lo, min(hi, x))

    @staticmethod
    def _apply_deadzone(x, dz):
        if abs(x) <= dz:
            return 0.0
        # re-normalize outside deadzone
        sign = -1.0 if x < 0 else 1.0
        x = abs(x)
        x = (x - dz) / (1.0 - dz)
        return sign * min(max(x, 0.0), 1.0)

    @staticmethod
    def _move_toward(current, target, max_step):
        if current < target:
            return min(current + max_step, target)
        return max(current - max_step, target)

    @staticmethod
    def _approach(current, target, rise_step, fall_step):
        """
        Different rise/fall rates for realism.
        """
        if target > current:
            return min(current + rise_step, target)
        return max(current - fall_step, target)

    @staticmethod
    def _trigger_to_unit(v):
        """
        Converts a typical trigger axis in [-1, 1] to [0, 1].
        """
        return (v + 1.0) / 2.0

    @staticmethod
    def _wheel_axis_to_unit(v, invert=True):
        """
        Converts wheel pedal axis to [0, 1].
        Common wheel pedals may report:
          released = 1.0, pressed = -1.0
        """
        if invert:
            return (1.0 - v) / 2.0
        return (v + 1.0) / 2.0

    @staticmethod
    def _is_quit_shortcut(key):
        return (key == K_ESCAPE) or (key == K_q and pygame.key.get_mods() & KMOD_CTRL)
