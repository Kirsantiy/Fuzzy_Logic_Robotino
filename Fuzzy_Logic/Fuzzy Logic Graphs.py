import numpy as np
import matplotlib.pyplot as plt

def plot_membership_functions():
    # Входная переменная S (Расстояние до целевого объекта)
    S = np.linspace(0, 200, 500)
    small_distance = np.where(S <= 70, 1, np.maximum(1 - (S - 70) / 50, 0))
    large_distance = np.where(S >= 120, 1, np.maximum((S - 70) / 50, 0))

    # Входная переменная d (Расстояние до динамического объекта)
    d = np.linspace(0, 50, 500)
    small_d = np.where(d <= 20, 1, np.maximum(1 - (d - 20) / 15, 0))
    large_d = np.where(d >= 35, 1, np.maximum((d - 20) / 15, 0))

    # Входная переменная νдо (Скорость движения динамического объекта)
    v_dynamic = np.linspace(0, 30, 500)
    small_v_dynamic = np.where(v_dynamic <= 10, 1, np.maximum(1 - (v_dynamic - 10) / 10, 0))
    large_v_dynamic = np.where(v_dynamic >= 20, 1, np.maximum((v_dynamic - 10) / 10, 0))

    # Выходная переменная νр (Скорость робота)
    v_robot = np.linspace(0, 40, 500)
    small_v_robot = np.where(v_robot <= 10, 1, np.maximum(1 - (v_robot - 10) / 15, 0))
    large_v_robot = np.where(v_robot >= 25, 1, np.maximum((v_robot - 10) / 15, 0))

    # Выходная переменная α (Направление движения)
    alpha = np.linspace(-90, 90, 500)
    left = np.where(alpha <= -40, 1, np.maximum((0 - alpha) / 40, 0))
    straight = np.where(np.abs(alpha) <= 40, 1 - np.abs(alpha) / 40, 0)
    right = np.where(alpha >= 40, 1, np.maximum((alpha - 0) / 40, 0))

    plt.figure(figsize=(15, 10))

    # График для S
    plt.subplot(3, 2, 1)
    plt.plot(S, small_distance, 'r', label='"Малое расстояние"')
    plt.plot(S, large_distance, 'b', label='"Большое расстояние"')
    plt.title('Расстояние до целевого объекта (S)')
    plt.xlabel('S, см')
    plt.ylabel('μ(S)')
    plt.legend(loc='upper right')
    plt.xticks(np.arange(0, 201, 20))
    plt.grid(True)

    # График для d
    plt.subplot(3, 2, 2)
    plt.plot(d, small_d, 'r', label='"Малое расстояние"')
    plt.plot(d, large_d, 'b', label='"Большое расстояние"')
    plt.title('Расстояние до динамического объекта (d)')
    plt.xlabel('d, см')
    plt.ylabel('μ(d)')
    plt.legend(loc='upper right')
    plt.xticks(np.arange(0, 51, 5))
    plt.grid(True)

    # График для νдо
    plt.subplot(3, 2, 3)
    plt.plot(v_dynamic, small_v_dynamic, 'r', label='"Малая скорость"')
    plt.plot(v_dynamic, large_v_dynamic, 'b', label='"Большая скорость"')
    plt.title('Скорость движения динамического объекта (νдо)')
    plt.xlabel('ν, см/с')
    plt.ylabel('μ(ν)')
    plt.legend(loc='upper right')
    plt.grid(True)

    # График для νр
    plt.subplot(3, 2, 4)
    plt.plot(v_robot, small_v_robot, 'r', label='"Малая скорость"')
    plt.plot(v_robot, large_v_robot, 'b', label='"Большая скорость"')
    plt.title('Скорость робота (νр)')
    plt.xlabel('νр, см/с')
    plt.ylabel('μ(νр)')
    plt.legend(loc='upper right')
    plt.grid(True)

    # График для α
    plt.subplot(3, 2, 5)
    plt.plot(alpha, left, 'r', label='"Влево"')
    plt.plot(alpha, straight, 'g', label='"Прямо"')
    plt.plot(alpha, right, 'b', label='"Вправо"')
    plt.title('Направление движения (α)')
    plt.xlabel('α, градусы')
    plt.ylabel('μ(α)')
    plt.legend(loc='upper right')
    plt.xticks(np.arange(-90, 91, 10))
    plt.grid(True)

    plt.tight_layout()
    plt.show()

plot_membership_functions()
