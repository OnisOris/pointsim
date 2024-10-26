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
