from commands2 import Subsystem
from phoenix6 import hardware, configs, controls, signals
from wpilib import SmartDashboard, Timer
from wpimath.filter import LinearFilter, SlewRateLimiter, Debouncer

from generated.tuner_constants import train_constants


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class shooter_subsystem(Subsystem):
    INDEXER_INVERTED = False       # AJUSTA
    SHOOTER_INVERTED = False       # AJUSTA

    MIN_SHOOTER_VOLTS = 0.0
    MAX_SHOOTER_VOLTS = 12.0
    SHOOTER_RAMP_VOLTS_PER_S = 24.0
    INDEXER_SETTLE_S = 0.3         # ignora el pico de arranque
    BALL_DEBOUNCE_S = 0.08

    SHOT_TABLE = [
        (1.0, 5.5),
        (2.0, 7.0),
        (3.0, 8.5),
        (4.0, 10.0),
    ]

    @classmethod
    def _table_volts(cls, distance):
        table = cls.SHOT_TABLE
        if distance <= table[0][0]:
            return table[0][1]
        if distance >= table[-1][0]:
            return table[-1][1]
        for (d0, v0), (d1, v1) in zip(table, table[1:]):
            if d0 <= distance <= d1:
                t = (distance - d0) / (d1 - d0)
                return v0 + (v1 - v0) * t
        return table[-1][1]

    def is_manual(self):
        return SmartDashboard.getBoolean("ManualShooting", False)

    def __init__(self):
        super().__init__()

        self.distance_to_hub = 1.0  # metroa

        self.indexer = hardware.TalonFX(train_constants._indexer_motor_id)
        self.shooter = hardware.TalonFX(train_constants._shooter_motor_id)
        self._configure(self.indexer, self.INDEXER_INVERTED,
                        signals.NeutralModeValue.BRAKE, stator_limit=60.0)
        self._configure(self.shooter, self.SHOOTER_INVERTED,
                        signals.NeutralModeValue.COAST, stator_limit=60.0)

        self._duty = controls.DutyCycleOut(0)
        self._volts = controls.VoltageOut(0)

        self._current_filter = LinearFilter.movingAverage(10)
        self._filtered_amps = 0.0
        self._load_amps = 0.0
        self._ball_debounce = Debouncer(self.BALL_DEBOUNCE_S, Debouncer.DebounceType.kRising)
        self._ball_detected = False
        self._shooter_slew = SlewRateLimiter(self.SHOOTER_RAMP_VOLTS_PER_S)

        self._indexer_running = False
        self._indexer_timer = Timer()

        SmartDashboard.setDefaultBoolean("ManualShooting", False)
        SmartDashboard.setDefaultNumber("Shooter/ManualVolts", 6.0)
        SmartDashboard.setDefaultNumber("Shooter/LoadGainVoltsPerAmp", 0.05)
        SmartDashboard.setDefaultNumber("Shooter/IndexerIdleAmps", 5.0)
        SmartDashboard.setDefaultNumber("Shooter/BallThresholdAmps", 10.0)
        SmartDashboard.setDefaultNumber("Shooter/TrimVolts", 0.0)
        SmartDashboard.setDefaultBoolean("Shooter/CalibrationMode", False)
        SmartDashboard.setDefaultNumber("Shooter/CalibrationVolts", 6.0)

    @staticmethod
    def _configure(motor, inverted, neutral, stator_limit):
        cfg = configs.TalonFXConfiguration()
        cfg.motor_output.neutral_mode = neutral
        cfg.motor_output.inverted = (
            signals.InvertedValue.CLOCKWISE_POSITIVE
            if inverted
            else signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        # El límite debe quedar por ENCIMA de la carga que quieres medir,
        # si no, la lectura de corriente se satura
        cfg.current_limits.stator_current_limit = stator_limit
        cfg.current_limits.stator_current_limit_enable = True
        motor.configurator.apply(cfg)

    # ---------- Distancia ----------
    def set_distance_to_hub(self, meters):
        self.distance_to_hub = meters

    # ---------- Indexer ----------
    def indexer_movement(self, percent):
        """Mueve el indexer y mide la carga. Llamar cada ciclo mientras esté activo."""
        if not self._indexer_running:
            self._indexer_running = True
            self._indexer_timer.restart()
        self.indexer.set_control(self._duty.with_output(percent))

    def stop_indexer(self):
        self._indexer_running = False
        self._indexer_timer.stop()
        self._indexer_timer.reset()
        self.indexer.set_control(self._duty.with_output(0.0))

    def indexer_load_amps(self):
        """Carga estimada del indexer (corriente filtrada menos la de vacío)."""
        return self._load_amps

    def ball_detected(self):
        return self._ball_detected

    # ---------- Shooter ----------
    def shooter_target_volts(self):
        if self.is_manual():
            # Manual: voltaje fijo desde Elastic
            volts = SmartDashboard.getNumber("Shooter/ManualVolts", 6.0)
        elif SmartDashboard.getBoolean("Shooter/CalibrationMode", False):
            # Calibración: voltaje manual, pero el auto-aim sigue midiendo distancia
            volts = SmartDashboard.getNumber("Shooter/CalibrationVolts", 6.0)
        else:
            # Automático: tabla por distancia + trim + corrección por carga
            base = self._table_volts(self.distance_to_hub)
            trim = SmartDashboard.getNumber("Shooter/TrimVolts", 0.0)
            load_gain = SmartDashboard.getNumber("Shooter/LoadGainVoltsPerAmp", 0.05)
            volts = base + trim + load_gain * self._load_amps
        return _clamp(volts, self.MIN_SHOOTER_VOLTS, self.MAX_SHOOTER_VOLTS)

    def shooter_movement(self):
        """Calcula el voltaje y lo aplica con rampa. Llamar cada ciclo mientras dispara."""
        applied = self._shooter_slew.calculate(self.shooter_target_volts())
        self.shooter.set_control(self._volts.with_output(applied))

    def stop_shooter(self):
        self._shooter_slew.reset(0.0)
        self.shooter.set_control(self._volts.with_output(0.0))

    def shooter_velocity_rps(self):
        return self.shooter.get_velocity().value_as_double

    # ---------- Ciclo ----------
    def periodic(self):
        raw = abs(self.indexer.get_stator_current().value_as_double)
        self._filtered_amps = self._current_filter.calculate(raw)

        settled = self._indexer_running and self._indexer_timer.get() > self.INDEXER_SETTLE_S
        if settled:
            idle = SmartDashboard.getNumber("Shooter/IndexerIdleAmps", 5.0)
            self._load_amps = max(0.0, self._filtered_amps - idle)
        else:
            self._load_amps = 0.0

        threshold = SmartDashboard.getNumber("Shooter/BallThresholdAmps", 10.0)
        self._ball_detected = self._ball_debounce.calculate(settled and self._load_amps > threshold)

        SmartDashboard.putNumber("Shooter/IndexerAmps", self._filtered_amps)
        SmartDashboard.putNumber("Shooter/IndexerLoadAmps", self._load_amps)
        SmartDashboard.putBoolean("Shooter/BallDetected", self._ball_detected)
        SmartDashboard.putNumber("Shooter/DistanceToHub", self.distance_to_hub)
        SmartDashboard.putNumber("Shooter/TargetVolts", self.shooter_target_volts())
        SmartDashboard.putNumber("Shooter/VelocityRPS", self.shooter_velocity_rps())

        d_min, d_max = self.SHOT_TABLE[0][0], self.SHOT_TABLE[-1][0]
        SmartDashboard.putBoolean(
            "Shooter/DistanceInRange", d_min <= self.distance_to_hub <= d_max
        )
        # Fila lista para copiar a la tabla
        SmartDashboard.putString(
            "Shooter/CalibrationRow",
            f"({self.distance_to_hub:.2f}, {self.shooter_target_volts():.1f}),",
        )