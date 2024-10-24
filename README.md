# pointsim
Проект создан для симуляции материальной точки для тестирования ПИД регуляторов и их коэффициентов.

# Пример
```python
from pointsim import PIDController, Point
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pointsim import StabilizationSimulator

# Инициализируем PID-контроллер с коэффициентами
kp = [6.103582235784548, 6.103582235784548]
ki = [0, 0]
kd = [5.898832824054038, 5.898832824054038]
pid_controller = PIDController(kp, ki, kd)

# Инициализируем точку
mass = 1.0
position = np.array([5.0, 5.0])  # Начальная позиция вдали от центра
speed = np.array([0.0, 0.0])  # Начальная скорость
point = Point(mass, position, speed)

# Создаем симулятор стабилизации с PID-регулятором
stabilization_simulator = StabilizationSimulator("PIDStabilizationSim", point, dt=0.1, pid_controller=pid_controller)

# Запускаем анимацию стабилизации с графиками
stabilization_simulator.animate()
```
