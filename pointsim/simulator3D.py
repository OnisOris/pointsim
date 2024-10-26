import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from .controller import PIDController
import time

class Simulator3D:
    def __init__(self, name: str, simulation_object: 'Point3D', dt: float, force: np.ndarray):
        """
        Инициализация 3D симулятора.

        :param name: Имя симулятора.
        :param simulation_object: Объект, который участвует в симуляции (экземпляр класса Point3D).
        :param dt: Шаг времени симуляции.
        :param force: Сила, действующая на объект симуляции.
        """
        self.name = name
        self.t0 = time.time()
        self.simulation_object = simulation_object
        self.simulator_flag = True
        self.dt = dt
        self.force = force  # Сила, действующая на объект

    def step(self):
        """
        Один шаг симуляции для обновления состояния объекта.
        """
        acceleration = self.force / self.simulation_object.mass
        self.simulation_object.move(acceleration, self.dt)

    def animate(self):
        """
        Метод для создания и отображения анимации.
        """
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(-10, 10)
        scatter = ax.scatter(self.simulation_object.position[0],
                             self.simulation_object.position[1],
                             self.simulation_object.position[2],
                             s=100, color='red')

        def update(frame):
            self.step()
            x, y, z = self.simulation_object.position
            scatter._offsets3d = (np.array([x]), np.array([y]), np.array([z]))  # отдельные массивы для 3D координат
            return scatter,

        # cache_frame_data=False предотвращает проблему кэширования
        anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
        plt.show()

class StabilizationSimulator3D(Simulator3D):
    def __init__(self, name: str, simulation_object: 'Point3D', dt: float, pid_controller: PIDController):
        """
        Инициализация симулятора стабилизации с ПИД-регулятором.
        """
        super().__init__(name, simulation_object, dt, np.zeros(3))
        self.pid_controller = pid_controller
        self.time_data, self.x_data, self.y_data, self.z_data = [], [], [], []

        self.initial_position = simulation_object.position.copy()
        self.initial_speed = simulation_object.speed.copy()

    def step(self):
        """
        Один шаг симуляции для стабилизации в центре.
        """
        control_signal = self.pid_controller.compute_control(
            target_position=np.array([0.0, 0.0, 0.0]),
            current_position=self.simulation_object.position,
            dt=self.dt
        )
        acceleration = control_signal / self.simulation_object.mass
        self.simulation_object.move(acceleration, self.dt)

        current_time = time.time() - self.t0
        self.time_data.append(current_time)
        self.x_data.append(self.simulation_object.position[0])
        self.y_data.append(self.simulation_object.position[1])
        self.z_data.append(self.simulation_object.position[2])

    def reset(self):
        """
        Сброс симуляции к начальному состоянию.
        """
        self.simulation_object.position = self.initial_position.copy()
        self.simulation_object.speed = self.initial_speed.copy()

class Point3D:
    def __init__(self, mass: float, position: np.ndarray, speed: np.ndarray):
        """
        Инициализация точки в симуляции.
        """
        self.mass = mass
        self.position = position
        self.speed = speed

    def move(self, acceleration: np.ndarray, dt: float) -> np.ndarray:
        """
        Обновление положения и скорости объекта.
        """
        self.speed += acceleration * dt
        self.position += self.speed * dt + 0.5 * acceleration * dt ** 2
        return self.position
