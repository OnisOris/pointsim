import matplotlib
import numpy as np
from pointsim.cython_pid import PIDController
from pointsim import StabilizationSimulator2D,  Point2D
matplotlib.use('Qt5Agg')

# Инициализируем PID-контроллер с коэффициентами
kp = [6.103582235784548, 6.103582235784548]
ki = [0, 0]
kd = [5.898832824054038, 5.898832824054038]
# pid_controller = PIDController(kp, ki, kd)
pid_controller = PIDController(np.array(kp, dtype=np.float64),
                                    np.array(ki, dtype=np.float64),
                                    np.array(kd, dtype=np.float64))

# Инициализируем точку
mass = 1.0
position = np.array([5.0, 5.0])  # Начальная позиция вдали от центра
speed = np.array([0.0, 0.0])  # Начальная скорость
point = Point2D(mass, position, speed)

# Создаем симулятор стабилизации с PID-регулятором
stabilization_simulator = StabilizationSimulator2D("PIDStabilizationSim", point, dt=0.1, pid_controller=pid_controller)

# Запускаем анимацию стабилизации с графиками
stabilization_simulator.animate()

