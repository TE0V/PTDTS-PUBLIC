"""
Motor implementations for PTDTS
DRV8874 motor driver and simulated motor
"""

import time
import logging
from typing import Optional
from .interfaces import MotorInterface

logger = logging.getLogger(__name__)


class CurrentSensor:
    """
    Current sensor for DRV8874 IPROPI pin
    Reads current sense voltage via ADC and converts to motor current
    """

    def __init__(
        self,
        adc_channel: int = 1,
        ipropi_resistance: float = 10000.0,  # 10kΩ typical
        adc_address: int = 0x48
    ):
        """
        Initialize current sensor

        Args:
            adc_channel: ADS1115 channel for current sense (0-3)
            ipropi_resistance: IPROPI pullup resistance in ohms
            adc_address: I2C address of ADS1115
        """
        self.adc_channel = adc_channel
        self.ipropi_resistance = ipropi_resistance
        self.current_amps = 0.0

        try:
            import board
            import busio
            import adafruit_ads1x15.ads1115 as ADS
            from adafruit_ads1x15.analog_in import AnalogIn

            # Initialize I2C
            i2c = busio.I2C(board.SCL, board.SDA)

            # Initialize ADS1115
            self.ads = ADS.ADS1115(i2c, address=adc_address)
            self.ads.gain = 1  # ±4.096V range

            # Create analog input channel
            if adc_channel == 0:
                self.chan = AnalogIn(self.ads, ADS.P0)
            elif adc_channel == 1:
                self.chan = AnalogIn(self.ads, ADS.P1)
            elif adc_channel == 2:
                self.chan = AnalogIn(self.ads, ADS.P2)
            elif adc_channel == 3:
                self.chan = AnalogIn(self.ads, ADS.P3)

            logger.info(f"Current sensor initialized on ADS1115 channel {adc_channel}")

        except Exception as e:
            logger.warning(f"Current sensor not available: {e}")
            self.chan = None

    def read_current(self) -> float:
        """
        Read motor current from IPROPI pin

        DRV8874 IPROPI current: 377µA per amp of motor current (typical)
        Voltage = I_PROPI × R_PROPI

        Returns:
            Motor current in amps
        """
        if self.chan is None:
            return 0.0

        try:
            # Read voltage from IPROPI pin
            voltage = self.chan.voltage

            # Calculate IPROPI current
            i_propi = voltage / self.ipropi_resistance  # Amps

            # Convert to motor current (377µA/A typical sensitivity)
            motor_current = i_propi / 377e-6

            self.current_amps = abs(motor_current)  # Current magnitude
            return self.current_amps

        except Exception as e:
            logger.error(f"Error reading current sensor: {e}")
            return self.current_amps


class DRV8874Motor(MotorInterface):
    """
    DRV8874 DC motor driver
    PWM control via IN1 and IN2 pins

    HARDWARE-VERIFIED DIRECTION MAPPING (from test_pan_motor.py):
    - IN1=PWM, IN2=0   → CW rotation  → Encoder count increases → angle increases (via negation in encoder.py)
    - IN1=0, IN2=PWM   → CCW rotation → Encoder count decreases → angle decreases (via negation in encoder.py)

    Note: Encoder counts increase CW, but angle calculation negates to match expected direction
    """

    def __init__(
        self,
        gpio_in1: int = 27,
        gpio_in2: int = 22,
        gpio_fault: Optional[int] = None,
        pwm_frequency: int = 1000,
        max_pwm: float = 0.70,
        enable_current_sense: bool = False,
        current_sense_channel: int = 1
    ):
        """
        Initialize DRV8874 motor driver

        Args:
            gpio_in1: GPIO pin for IN1
            gpio_in2: GPIO pin for IN2
            gpio_fault: GPIO pin for FAULT (optional, active low)
            pwm_frequency: PWM frequency in Hz
            max_pwm: Maximum PWM duty cycle (0-1)
            enable_current_sense: Enable current sensing via IPROPI
            current_sense_channel: ADS1115 channel for current sense
        """
        self.gpio_in1 = gpio_in1
        self.gpio_in2 = gpio_in2
        self.gpio_fault = gpio_fault
        self.pwm_frequency = pwm_frequency
        self.max_pwm = max_pwm

        self.current_pwm = 0.0
        self.fault_pin = None
        self.current_sensor = None
        self.fault_detected = False

        try:
            from gpiozero import PWMOutputDevice, DigitalInputDevice

            # Initialize PWM outputs
            self.in1 = PWMOutputDevice(gpio_in1, frequency=pwm_frequency, initial_value=0)
            self.in2 = PWMOutputDevice(gpio_in2, frequency=pwm_frequency, initial_value=0)

            # Initialize fault pin if provided (active low)
            if gpio_fault is not None:
                self.fault_pin = DigitalInputDevice(gpio_fault, pull_up=True)
                logger.info(f"DRV8874 fault monitoring enabled on GPIO{gpio_fault}")

            # Initialize current sensor if enabled
            if enable_current_sense:
                try:
                    self.current_sensor = CurrentSensor(adc_channel=current_sense_channel)
                except Exception as e:
                    logger.warning(f"Current sensor initialization failed: {e}")

            logger.info(f"DRV8874 motor initialized: IN1=GPIO{gpio_in1}, IN2=GPIO{gpio_in2}, freq={pwm_frequency}Hz")

        except Exception as e:
            logger.error(f"Failed to initialize DRV8874: {e}")
            raise

    def set_pwm(self, pwm: float):
        """
        Set motor PWM duty cycle (hardware-verified from test_pan_motor.py)

        Args:
            pwm: PWM duty cycle (-1.0 to 1.0)
                 Positive = CW (IN1=PWM, IN2=0) → Count increases, angle increases
                 Negative = CCW (IN1=0, IN2=PWM) → Count decreases, angle decreases
        """
        # Clamp to limits
        pwm = max(-self.max_pwm, min(self.max_pwm, pwm))
        self.current_pwm = pwm

        if pwm > 0:
            # CW rotation: IN1=PWM, IN2=0
            self.in1.value = abs(pwm)
            self.in2.value = 0
        elif pwm < 0:
            # CCW rotation: IN1=0, IN2=PWM
            self.in1.value = 0
            self.in2.value = abs(pwm)
        else:
            # Stop: Both low
            self.in1.value = 0
            self.in2.value = 0

    def stop(self):
        """Stop motor (coast)"""
        self.set_pwm(0)
        logger.debug("Motor stopped (coast)")

    def brake(self):
        """
        Brake motor (active braking, both inputs high)
        More aggressive than stop()
        """
        self.in1.value = 1.0
        self.in2.value = 1.0
        self.current_pwm = 0.0
        logger.debug("Motor braking")

    def get_current_pwm(self) -> float:
        """
        Get current PWM setting

        Returns:
            Current PWM duty cycle
        """
        return self.current_pwm

    def get_current_draw(self) -> float:
        """
        Get motor current draw

        Returns:
            Motor current in amps (0 if sensor not available)
        """
        if self.current_sensor:
            return self.current_sensor.read_current()
        return 0.0

    def check_fault(self) -> bool:
        """
        Check if motor driver fault is detected

        Returns:
            True if fault detected (FAULT pin low)
        """
        if self.fault_pin:
            # FAULT pin is active low
            fault = not self.fault_pin.value
            if fault and not self.fault_detected:
                logger.error("DRV8874 FAULT detected!")
                self.fault_detected = True
            elif not fault and self.fault_detected:
                logger.info("DRV8874 fault cleared")
                self.fault_detected = False
            return fault
        return False

    def get_status(self) -> dict:
        """
        Get comprehensive motor status

        Returns:
            Dictionary with motor status information
        """
        return {
            'pwm': self.current_pwm,
            'current_amps': self.get_current_draw(),
            'fault': self.check_fault()
        }

    def close(self):
        """Clean up GPIO resources"""
        self.stop()
        if hasattr(self, 'in1'):
            self.in1.close()
        if hasattr(self, 'in2'):
            self.in2.close()
        if hasattr(self, 'fault_pin') and self.fault_pin:
            self.fault_pin.close()
        logger.info("DRV8874 motor closed")


class SimulatedMotor(MotorInterface):
    """
    Simulated motor for testing without hardware
    Uses accurate Pololu 37D motor model with realistic physics
    """

    def __init__(
        self,
        max_pwm: float = 0.70,
        voltage: float = 12.0,
        total_gear_ratio: float = 135.0,
        encoder: Optional[object] = None
    ):
        """
        Initialize simulated motor with accurate Pololu model

        Args:
            max_pwm: Maximum PWM duty cycle
            voltage: Motor voltage (12V nominal)
            total_gear_ratio: Total system gear ratio
            encoder: Reference to simulated encoder to update
        """
        self.max_pwm = max_pwm
        self.encoder = encoder

        # Import motor model
        from .motor_simulation import PololuMotorModel
        self.motor_model = PololuMotorModel(voltage=voltage, total_gear_ratio=total_gear_ratio)

        self.current_pwm = 0.0
        self.current_velocity_dps = 0.0  # degrees per second
        self.last_update_time = time.time()

        logger.info(f"Simulated motor initialized with accurate Pololu model: {voltage}V, {total_gear_ratio}:1")

    def _update_physics(self):
        """Update simulated motor physics using accurate model"""
        current_time = time.time()
        dt = current_time - self.last_update_time

        if dt > 0 and dt < 1.0:  # Ignore large time gaps
            # Compute motor torque
            torque_kgmm = self.motor_model.compute_motor_torque(
                self.current_pwm,
                self.current_velocity_dps
            )

            # Compute acceleration
            acceleration_dps2 = self.motor_model.compute_acceleration(torque_kgmm)

            # Update velocity (simple Euler integration)
            self.current_velocity_dps += acceleration_dps2 * dt

            # Update encoder if connected
            if self.encoder:
                self.encoder.set_velocity(self.current_velocity_dps)

            self.last_update_time = current_time

    def set_pwm(self, pwm: float):
        """
        Set motor PWM duty cycle

        Args:
            pwm: PWM duty cycle (-1.0 to 1.0)
        """
        # Update physics before changing PWM
        self._update_physics()

        # Clamp to limits
        pwm = max(-self.max_pwm, min(self.max_pwm, pwm))
        self.current_pwm = pwm

    def stop(self):
        """Stop motor"""
        self._update_physics()
        self.set_pwm(0)
        logger.debug("Simulated motor stopped")

    def brake(self):
        """Brake motor (instant stop in simulation)"""
        self._update_physics()
        self.current_pwm = 0.0
        self.current_velocity_dps = 0.0
        if self.encoder:
            self.encoder.set_velocity(0.0)
        logger.debug("Simulated motor braking")

    def get_current_pwm(self) -> float:
        """
        Get current PWM setting

        Returns:
            Current PWM duty cycle
        """
        self._update_physics()
        return self.current_pwm

    def get_velocity_dps(self) -> float:
        """
        Get current simulated velocity in degrees per second

        Returns:
            Velocity in deg/sec
        """
        self._update_physics()
        return self.current_velocity_dps

    def get_current_draw(self) -> float:
        """
        Get estimated current draw

        Returns:
            Current in Amps
        """
        self._update_physics()
        return self.motor_model.get_motor_current(self.current_pwm, self.current_velocity_dps)

    def close(self):
        """Clean up (no-op for simulation)"""
        self.stop()
        logger.info("Simulated motor closed")
