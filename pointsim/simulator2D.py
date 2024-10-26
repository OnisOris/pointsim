import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from .controller import PIDController
import time
from typing import Union


class Simulator2D:
    def __init__(self,
                 name: str,
                 simulation_object: 'Point2D',
                 dt: float,
                 force: np.ndarray):
        """
        Инициализация симулятора.

        :param name: Имя симулятора.
        :type name: str
        :param simulation_object: Объект, который участвует в симуляции (экземпляр класса Point).
        :type simulation_object: Point
        :param dt: Шаг времени симуляции.
        :type dt: float
        :param force: Сила, действующая на объект симуляции.
        :type force: np.ndarray
        """
        self.name = name
        self.t0 = time.time()
        self.simulation_object = simulation_object
        self.simulator_flag = True
        self.dt = dt
        self.force = force  # Сила, действующая на объект

    def step(self) -> None:
        """
        Один шаг симуляции для обновления состояния объекта.

        :return: None
        """
        acceleration = self.force / self.simulation_object.mass  # Вычисляем ускорение через силу и массу
        self.simulation_object.move(acceleration, self.dt)

    def update(self, frame: int, scatter: plt.scatter) -> tuple:
        """
        Обновление положения объекта на графике для каждого кадра анимации.

        :param frame: Номер текущего кадра.
        :type frame: int
        :param scatter: Текущая точка на графике.
        :type scatter: plt.scatter
        :return: Обновленный scatter на графике.
        :rtype: tuple
        """
        self.step()  # Выполняем один шаг симуляции
        scatter.set_offsets(self.simulation_object.position)  # Обновляем положение точки на графике
        return scatter,

    def animate(self) -> None:
        """
        Метод для создания и отображения анимации.

        :return: None
        """
        # Настраиваем график
        fig, ax = plt.subplots()
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        scatter = ax.scatter(self.simulation_object.position[0], self.simulation_object.position[1], s=100, color='red')

        # Создаем анимацию
        anim = FuncAnimation(fig, self.update, fargs=(scatter,), interval=100)

        # Показываем анимацию
        plt.show()


class StabilizationSimulator2D(Simulator2D):
    def __init__(self,
                 name: str,
                 simulation_object: 'Point',
                 dt: float,
                 pid_controller: PIDController):
        """
        Инициализация симулятора стабилизации с ПИД-регулятором.

        :param name: Имя симулятора.
        :type name: str
        :param simulation_object: Объект симуляции (экземпляр класса Point).
        :type simulation_object: Point
        :param dt: Шаг времени симуляции.
        :type dt: float
        :param pid_controller: PID-регулятор для управления положением.
        :type pid_controller: PIDController
        """
        super().__init__(name, simulation_object, dt, np.zeros(2))  # Инициализируем симулятор без внешней силы
        self.pid_controller = pid_controller  # PID-регулятор для управления точкой
        self.time_data = []  # Время
        self.x_data = []  # Данные по x(t)
        self.y_data = []  # Данные по y(t)

        # Сохраняем начальные условия для сброса
        self.initial_position = simulation_object.position.copy()
        self.initial_speed = simulation_object.speed.copy()

    def step(self) -> None:
        """
        Один шаг симуляции для стабилизации в центре.

        :return: None
        """
        # Рассчитываем управляющее воздействие с помощью PID-регулятора
        control_signal = self.pid_controller.compute_control(
            target_position=np.array([0.0, 0.0]),  # Центр как целевая позиция
            current_position=self.simulation_object.position,
            dt=self.dt
        )

        # Применяем корректирующее воздействие как ускорение
        acceleration = control_signal / self.simulation_object.mass
        self.simulation_object.move(acceleration, self.dt)

        # Сохраняем данные для построения графиков
        current_time = time.time() - self.t0
        self.time_data.append(current_time)
        self.x_data.append(self.simulation_object.position[0])
        self.y_data.append(self.simulation_object.position[1])

    def reset(self) -> None:
        """
        Сброс симуляции к начальному состоянию.

        :return: None
        """
        self.simulation_object.position = self.initial_position.copy()
        self.simulation_object.speed = self.initial_speed.copy()

    def update(self,
               frame: int,
               scatter: plt.scatter,
               x_line: plt.Line2D,
               y_line: plt.Line2D,
               time_text: plt.Text,
               ax_x: plt.Axes,
               ax_y: plt.Axes) -> tuple:
        """
        Обновление графики для каждой итерации симуляции.

        :param frame: Текущий номер кадра.
        :type frame: int
        :param scatter: Точка на основном графике.
        :type scatter: plt.scatter
        :param x_line: Линия для графика x(t).
        :type x_line: plt.Line2D
        :param y_line: Линия для графика y(t).
        :type y_line: plt.Line2D
        :param time_text: Текст для отображения времени на основном графике.
        :type time_text: plt.Text
        :param ax_x: Ось для графика x(t).
        :type ax_x: plt.Axes
        :param ax_y: Ось для графика y(t).
        :type ax_y: plt.Axes
        :return: Обновленные объекты графики.
        :rtype: tuple
        """
        self.step()  # Выполняем шаг симуляции

        # Обновляем положение точки на графике
        scatter.set_offsets(self.simulation_object.position)

        # Обновляем линии графиков x(t) и y(t)
        x_line.set_data(self.time_data, self.x_data)
        y_line.set_data(self.time_data, self.y_data)

        # Адаптивное масштабирование осей
        ax_x.set_xlim(0, max(self.time_data))  # по оси времени
        ax_x.set_ylim(min(self.x_data) - 1, max(self.x_data) + 1)  # по оси x(t)

        ax_y.set_xlim(0, max(self.time_data))  # по оси времени
        ax_y.set_ylim(min(self.y_data) - 1, max(self.y_data) + 1)  # по оси y(t)

        # Обновляем текст времени
        time_text.set_text(f'Time: {self.time_data[-1]:.2f}s')

        return scatter, x_line, y_line, time_text

    def animate(self) -> None:
        """
        Метод для создания и отображения анимации.

        :return: None
        """
        fig, (ax_main, ax_x, ax_y) = plt.subplots(3, 1, figsize=(8, 12))

        # Настраиваем основной график для отображения позиции
        ax_main.set_xlim(-10, 10)
        ax_main.set_ylim(-10, 10)
        scatter = ax_main.scatter(self.simulation_object.position[0], self.simulation_object.position[1], s=100,
                                  color='red')
        ax_main.set_title("Position of the Point")
        ax_main.axhline(0, color='grey', lw=1)  # Линия целевой позиции по оси y
        ax_main.axvline(0, color='grey', lw=1)  # Линия целевой позиции по оси x

        # Настраиваем график для x(t)
        ax_x.set_xlim(0, 20)  # Задаём ограничение по времени
        ax_x.set_ylim(-10, 10)  # Диапазон изменения координат
        x_line, = ax_x.plot([], [], lw=2, label='x(t)', color='blue')
        ax_x.set_title("x(t) over Time")
        ax_x.set_xlabel("Time [s]")
        ax_x.set_ylabel("x [m]")
        ax_x.axhline(0, color='grey', lw=1)  # Линия целевой позиции (target) по x
        ax_x.legend()

        # Настраиваем график для y(t)
        ax_y.set_xlim(0, 20)
        ax_y.set_ylim(-10, 10)
        y_line, = ax_y.plot([], [], lw=2, label='y(t)', color='green')
        ax_y.set_title("y(t) over Time")
        ax_y.set_xlabel("Time [s]")
        ax_y.set_ylabel("y [m]")
        ax_y.axhline(0, color='grey', lw=1)  # Линия целевой позиции (target) по y
        ax_y.legend()

        # Текст для отображения времени
        time_text = ax_main.text(0.02, 0.95, '', transform=ax_main.transAxes)

        # Создаем анимацию
        anim = FuncAnimation(fig, self.update, fargs=(scatter, x_line, y_line, time_text, ax_x, ax_y), interval=100)

        # Показываем анимацию
        plt.tight_layout()
        plt.show()


class Point2D:
    def __init__(self,
                 mass: Union[float, int],
                 position: np.ndarray,
                 speed: np.ndarray):
        """
        Инициализация точки в симуляции.

        :param mass: Масса объекта.
        :type mass: float | int
        :param position: Положение объекта.
        :type position: np.ndarray
        :param speed: Скорость объекта.
        :type speed: np.ndarray
        """
        self.mass = mass
        self.position = position
        self.speed = speed

    def move(self, acceleration: np.ndarray, dt: float) -> np.ndarray:
        """
        Обновление положения и скорости объекта.

        :param acceleration: Ускорение, действующее на объект.
        :type acceleration: np.ndarray
        :param dt: Шаг времени для обновления состояния.
        :type dt: float
        :return: Обновленное положение объекта.
        :rtype: np.ndarray
        """
        # Обновление скорости
        self.speed = self.speed + acceleration * dt
        # Обновление позиции
        self.position = self.position + self.speed * dt + 0.5 * acceleration * dt ** 2
        return self.position
