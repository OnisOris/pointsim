import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from pointsim.cython_pid import PIDController  # Cython-версия PIDController
import time


class Point3D:
    def __init__(self, mass, position, speed):
        self.mass = mass
        self.initial_position = np.array(position, dtype=np.float64)
        self.position = np.array(position, dtype=np.float64)
        self.speed = np.array(speed, dtype=np.float64)

    def rk4_step(self, acceleration, dt):
        k1v = acceleration * dt
        k1x = self.speed * dt

        k2v = (acceleration + 0.5 * k1v) * dt
        k2x = (self.speed + 0.5 * k1x) * dt

        k3v = (acceleration + 0.5 * k2v) * dt
        k3x = (self.speed + 0.5 * k2x) * dt

        k4v = (acceleration + k3v) * dt
        k4x = (self.speed + k3x) * dt

        self.speed += (k1v + 2 * k2v + 2 * k3v + k4v) / 6
        self.position += (k1x + 2 * k2x + 2 * k3x + k4x) / 6


class Simulator3D:
    def __init__(self, name, mass, position, speed, kp, ki, kd, dt, max_acceleration=1):
        self.name = name
        self.dt = dt  # Временной шаг для симуляции
        self.simulation_object = Point3D(mass, position, speed)
        self.pid_controller = PIDController(np.array(kp, dtype=np.float64),
                                            np.array(ki, dtype=np.float64),
                                            np.array(kd, dtype=np.float64))
        self.time_data, self.x_data, self.y_data, self.z_data = [], [], [], []
        self.max_acceleration = max_acceleration
        self.simulation_time = 0  # Инициализируем симулированное время
        self.external_control_signal = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    def receive_external_signal(self, control_signal):
        """
        Метод для получения внешнего управляющего сигнала.
        Например, этот метод можно использовать для получения команды из внешнего источника (сети, сенсора, и т.д.).
        control_signal: np.array([x, y, z]) — вектор управления.
        """
        self.external_control_signal = control_signal

    def step(self):
        # Получаем текущую позицию и вычисляем сигнал управления с PID
        target_position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        pid_control_signal = self.pid_controller.compute_control(
            target_position=target_position,
            current_position=self.simulation_object.position,
            dt=self.dt
        )

        # Добавляем внешнее управление
        total_control_signal = np.clip(pid_control_signal + self.external_control_signal,
                                       -self.max_acceleration, self.max_acceleration)

        # Вычисляем ускорение
        acceleration = total_control_signal / self.simulation_object.mass
        self.simulation_object.rk4_step(acceleration, self.dt)

        # Обновляем симулированное время и сохраняем данные для анализа
        self.simulation_time += self.dt
        self.time_data.append(self.simulation_time)
        self.x_data.append(self.simulation_object.position[0])
        self.y_data.append(self.simulation_object.position[1])
        self.z_data.append(self.simulation_object.position[2])

    def run_simulation(self, steps):
        """
        Запуск симуляции без визуализации. Просто выполняем шаги симуляции.
        """
        for _ in range(steps):
            self.step()


class Simulator3DRealTime(Simulator3D):
    def __init__(self, name, mass, position, speed, kp, ki, kd, dt, max_acceleration=1):
        # Инициализируем базовый класс
        super().__init__(name, mass, position, speed, kp, ki, kd, dt, max_acceleration)

    def run_real_time_simulation(self):
        """
        Запуск симуляции в реальном времени. Шаг симуляции синхронизируется с реальным временем.
        """
        last_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - last_time

            # Проверяем, прошло ли достаточно времени для очередного шага
            if elapsed_time >= self.dt:
                self.step()
                last_time = current_time

            time.sleep(0.01)  # Немного ждем, чтобы избежать слишком частых проверок
            print(f"Position: {self.simulation_object.position}, Time: {self.simulation_time}")


class StabilizationSimulator3D(Simulator3D):
    def __init__(self, name, mass, position, speed, kp, ki, kd, dt, max_acceleration=1, show_trajectory=False):
        super().__init__(name, mass, position, speed, kp, ki, kd, dt, max_acceleration)
        self.show_trajectory = show_trajectory  # Опция для отображения траектории

    def step(self):
        target_position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        control_signal = np.clip(self.pid_controller.compute_control(
            target_position=target_position,
            current_position=self.simulation_object.position,
            dt=self.dt
        ), -self.max_acceleration, self.max_acceleration)
        acceleration = control_signal / self.simulation_object.mass
        self.simulation_object.rk4_step(acceleration, self.dt)

        # Обновляем симулированное время и сохраняем данные для графика
        self.simulation_time += self.dt
        self.time_data.append(self.simulation_time)
        self.x_data.append(self.simulation_object.position[0])
        self.y_data.append(self.simulation_object.position[1])
        self.z_data.append(self.simulation_object.position[2])

    def animate(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(-10, 10)

        # Используем начальное значение позиции из Point3D
        initial_position = self.simulation_object.initial_position
        scatter = ax.scatter(*initial_position, s=100, color='red')

        # Если включено отображение траектории, инициализируем линию
        if self.show_trajectory:
            trajectory_line, = ax.plot([], [], [], color='blue', lw=2)

        # Создаем текст для отображения координат над точкой
        coord_text = ax.text(initial_position[0], initial_position[1], initial_position[2] + 1,
                             f"({initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f})",
                             color='blue')

        # Создаем текстовый объект для отображения времени симуляции
        time_text = ax.text2D(0.95, 0.95, f"Time: 0.00 s", transform=ax.transAxes, color='black',
                              ha='right', va='top')

        def update(frame):
            self.step()
            x, y, z = self.simulation_object.position
            scatter._offsets3d = (np.array([x]), np.array([y]), np.array([z]))

            # Обновляем текстовый объект с текущими координатами точки
            coord_text.set_position((x, y))
            coord_text.set_3d_properties(z + 1)
            coord_text.set_text(f"({x:.2f}, {y:.2f}, {z:.2f})")

            # Обновляем текст времени симуляции
            time_text.set_text(f"Time: {self.simulation_time:.2f} s")

            # Если включено отображение траектории, обновляем линию траектории
            if self.show_trajectory:
                trajectory_line.set_data(self.x_data, self.y_data)
                trajectory_line.set_3d_properties(self.z_data)

            return scatter, coord_text, time_text

        anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
        plt.show()

    def run_simulation(self, steps):
        for _ in range(steps):
            self.step()
        self.animate()


class StabilizationSimulator3DRealTime(StabilizationSimulator3D):
    def init(self, name, mass, position, speed, kp, ki, kd, dt, max_acceleration=1, show_trajectory=False):
        super().init(name, mass, position, speed, kp, ki, kd, dt, max_acceleration, show_trajectory)
        self.external_control_signal = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    def receive_external_signal(self, control_signal):
        """
        Метод для получения внешнего управляющего сигнала.
        Например, этот метод можно использовать для получения команды из внешнего источника (сети, сенсора, и т.д.).
        control_signal: np.array([x, y, z]) — вектор управления.
        """
        self.external_control_signal = np.clip(control_signal, -self.max_acceleration, self.max_acceleration)

    def step(self):
        # Получаем текущую позицию и вычисляем сигнал управления с PID
        target_position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        pid_control_signal = self.pid_controller.compute_control(
            target_position=target_position,
            current_position=self.simulation_object.position,
            dt=self.dt
        )

        # Добавляем внешнее управление
        total_control_signal = np.clip(pid_control_signal + self.external_control_signal,
                                       -self.max_acceleration, self.max_acceleration)

        # Вычисляем ускорение
        acceleration = total_control_signal / self.simulation_object.mass
        self.simulation_object.rk4_step(acceleration, self.dt)

        # Обновляем симулированное время и сохраняем данные для графика
        self.simulation_time += self.dt
        self.time_data.append(self.simulation_time)
        self.x_data.append(self.simulation_object.position[0])
        self.y_data.append(self.simulation_object.position[1])
        self.z_data.append(self.simulation_object.position[2])

    def run_real_time_simulation(self):
        """
        Запуск симуляции в реальном времени. Шаг симуляции синхронизируется с реальным временем.
        """
        last_time = time.time()
        while True:
            current_time = time.time()
            elapsed_time = current_time - last_time
            print(self.simulation_object.position)
            # Проверяем, прошло ли достаточно времени для очередного шага
            if elapsed_time >= self.dt:
                self.step()
                last_time = current_time

            time.sleep(0.01)  # Немного ждем, чтобы избежать слишком частых проверок

    def animate_real_time(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(-10, 10)

        # Используем начальное значение позиции из Point3D
        initial_position = self.simulation_object.initial_position
        scatter = ax.scatter(*initial_position, s=100, color='red')

        # Если включено отображение траектории, инициализируем линию
        if self.show_trajectory:
            trajectory_line, = ax.plot([], [], [], color='blue', lw=2)
        # Создаем текст для отображения координат над точкой
        coord_text = ax.text(initial_position[0], initial_position[1], initial_position[2] + 1,
                             f"({initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f})",
                             color='blue')

        # Создаем текст для отображения времени симуляции
        time_text = ax.text2D(0.95, 0.95, f"Time: 0.00 s", transform=ax.transAxes, color='black',
                              ha='right', va='top')

        def update(frame):
            x, y, z = self.simulation_object.position
            scatter._offsets3d = (np.array([x]), np.array([y]), np.array([z]))

            # Обновляем текст с текущими координатами
            coord_text.set_position((x, y))
            coord_text.set_3d_properties(z + 1)
            coord_text.set_text(f"({x:.2f}, {y:.2f}, {z:.2f})")

            # Обновляем текст времени симуляции
            time_text.set_text(f"Time: {self.simulation_time:.2f} s")

            # Обновляем линию траектории
            if self.show_trajectory:
                trajectory_line.set_data(self.x_data, self.y_data)
                trajectory_line.set_3d_properties(self.z_data)

            return scatter, coord_text, time_text

        anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
        plt.show()

class StabilizationSimulator3DRealTime(StabilizationSimulator3D):
    def __init__(self, name, mass, position, speed, kp, ki, kd, dt, max_acceleration=1, show_trajectory=False):
        super().__init__(name, mass, position, speed, kp, ki, kd, dt, max_acceleration, show_trajectory)
        self.external_control_signal = np.array([0.0, 0.0, 0.0], dtype=np.float64)

    def receive_external_signal(self, control_signal):
        """
        Метод для получения внешнего управляющего сигнала.
        Например, этот метод можно использовать для получения команды из внешнего источника (сети, сенсора, и т.д.).
        control_signal: np.array([x, y, z]) — вектор управления.
        """
        self.external_control_signal = control_signal

    def step(self):
        # Получаем текущую позицию и вычисляем сигнал управления с PID
        target_position = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        pid_control_signal = self.pid_controller.compute_control(
            target_position=target_position,
            current_position=self.simulation_object.position,
            dt=self.dt
        )

        # Добавляем внешнее управление
        total_control_signal = np.clip(pid_control_signal + self.external_control_signal,
                                       -self.max_acceleration, self.max_acceleration)

        # Вычисляем ускорение
        acceleration = total_control_signal / self.simulation_object.mass
        self.simulation_object.rk4_step(acceleration, self.dt)

        # Обновляем симулированное время и сохраняем данные для графика
        self.simulation_time += self.dt
        self.time_data.append(self.simulation_time)
        self.x_data.append(self.simulation_object.position[0])
        self.y_data.append(self.simulation_object.position[1])
        self.z_data.append(self.simulation_object.position[2])

    def animate_real_time(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(-10, 10)

        # Используем начальное значение позиции из Point3D
        initial_position = self.simulation_object.initial_position
        scatter = ax.scatter(*initial_position, s=100, color='red')

        # Если включено отображение траектории, инициализируем линию
        if self.show_trajectory:
            trajectory_line, = ax.plot([], [], [], color='blue', lw=2)

        # Создаем текст для отображения координат над точкой
        coord_text = ax.text(initial_position[0], initial_position[1], initial_position[2] + 1,
                             f"({initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f})",
                             color='blue')

        # Создаем текст для отображения времени симуляции
        time_text = ax.text2D(0.95, 0.95, f"Time: 0.00 s", transform=ax.transAxes, color='black',
                              ha='right', va='top')

        def update(frame):
            self.step()  # Выполняем шаг симуляции в каждом кадре
            x, y, z = self.simulation_object.position
            scatter._offsets3d = (np.array([x]), np.array([y]), np.array([z]))

            # Обновляем текст с текущими координатами
            coord_text.set_position((x, y))
            coord_text.set_3d_properties(z + 1)
            coord_text.set_text(f"({x:.2f}, {y:.2f}, {z:.2f})")

            # Обновляем текст времени симуляции
            time_text.set_text(f"Time: {self.simulation_time:.2f} s")

            # Обновляем линию траектории
            if self.show_trajectory:
                trajectory_line.set_data(self.x_data, self.y_data)
                trajectory_line.set_3d_properties(self.z_data)

            return scatter, coord_text, time_text

        anim = FuncAnimation(fig, update, interval=self.dt * 1000, cache_frame_data=False)  # Интегрируем реальное время
        plt.show()
