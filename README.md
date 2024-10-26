# pointsim
Проект создан для симуляции материальной точки для тестирования ПИД регуляторов и их коэффициентов.

# Установка
```
python setup.py build_ext --inplace
```
# Примеры

```python
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
```

```python
import matplotlib
from pointsim import StabilizationSimulator3D

matplotlib.use('Qt5Agg')

if __name__ == "__main__":
    simulator = StabilizationSimulator3D(
        name="PIDStabilizationSim",
        mass=1.0,
        position=[10.0, 10.0, 5.0],  # Начальное смещение
        speed=[0.0, 0.0, 0.0],
        kp=[1, 1, 1],  # Коэффициенты PID
        ki=[0.0, 0.0, 0.0],  # Интегральная часть отключена
        kd=[1, 1, 1],  # Дифференциальная часть
        dt=0.05,  # Шаг времени
        show_trajectory=True,  # Включаем отображение траектории
        max_acceleration=5
    )

    simulator.run_simulation(steps=1000)
```

```python
import numpy as np
import matplotlib
from pointsim import StabilizationSimulator3DRealTime

matplotlib.use('Qt5Agg')


if __name__ == "__main__":
    simulator = StabilizationSimulator3DRealTime(
        name="PIDRealTimeSim",
        mass=1.0,
        position=[10.0, 10.0, 5.0],
        speed=[0.0, 0.0, 0.0],
        kp=[1, 1, 1],
        ki=[0.0, 0.0, 0.0],
        kd=[1, 1, 1],
        dt=0.05,
        show_trajectory=True,
        max_acceleration=5

    )

    # Передаем внешний управляющий сигнал
    simulator.receive_external_signal(np.array([0.1, -0.2, 0.3]))

    # Запуск симуляции с анимацией
    simulator.animate_real_time()
```


