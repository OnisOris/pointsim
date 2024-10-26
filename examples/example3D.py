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