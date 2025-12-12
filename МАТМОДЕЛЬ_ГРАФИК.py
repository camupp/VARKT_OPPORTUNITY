import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

mu_kerbin = 3.53e12          # Гравитационный параметр Кербина 
R_kerbin = 600000.0          # Радиус Кербина 
g0 = 9.81                    # Ускорение свободного падения 

# Атмосфера
rho0 = 1.225                 # Плотность на уровне моря 
H = 5600.0                   # Шкала высоты 

M_start = 52814.0            # Стартовая масса 
M_after_SRB = 32700.0        # Масса после сброса ускорителей 
M_dry_main = 6199.0          # Сухая масса центрального блока 
M_payload = 2760.0           # Полезная нагрузка 
M_prop_main = 25741.0        # Топливо центрального блока 
M_prop_SRB = 18215.0         # Топливо ускорителей 
M_shell_SRB = 1899.0         # Масса корпусов ускорителей 

Isp_main = 270.0             # Удельный импульс центрального двигателя 
Isp_SRB = 245.0              # Удельный импульс ускорителей 

t_SRB = 60.0                 # Время работы ускорителей 
t_main = 300.0               # Время работы центрального двигателя 

Cd = 0.3                     # Коэффициент сопротивления
S = 3.1416 * (1.5/2)**2      # Площадь поперечного сечения 

# Тяга SRB: суммарная тяга 9 ускорителей
T_SRB_total = M_prop_SRB * g0 * Isp_SRB / t_SRB  
# Тяга центрального двигателя
T_main = M_prop_main * g0 * Isp_main / t_main


def gravity(h):
    #Ускорение свободного падения на высоте h
    return mu_kerbin / (R_kerbin + h)**2

def density(h):
    #Плотность атмосферы на высоте h
    return rho0 * np.exp(-h / H)

def mass(t):
    #Масса ракеты в зависимости от времени
    if t < t_SRB:
        # Работают только SRB
        return M_start - M_prop_SRB * (t / t_SRB)
    elif t < t_SRB + t_main:
        # Работает только центральный двигатель
        prop_used = M_prop_main * ((t - t_SRB) / t_main)
        return M_after_SRB - prop_used
    else:
        # Двигатели выключены
        return M_after_SRB - M_prop_main

def thrust(t):
    #Тяга в зависимости от времени
    if t < t_SRB:
        return T_SRB_total 
    elif t < t_SRB + t_main:
        return T_main    
    else:
        return 0.0

def angle_of_attack(h):
    #Угол наклона ракеты к горизонту в зависимости от высоты (градусы)
    if h < 15000:
        return 90.0
    elif h < 30000:
        return 85.0  
    elif h < 40000:
        return 70.0
    elif h < 50000:
        return 60.0
    elif h < 60000:
        return 45.0
    elif h < 70000:
        return 35.0
    elif h < 80000:
        return 20.0
    else:
        return 0.0

# СИСТЕМА ДИФФЕРЕНЦИАЛЬНЫХ УРАВНЕНИЙ

def rocket_equations(t, y):
    vx, vy, x, h = y
    v = np.sqrt(vx**2 + vy**2)
    
    # Текущие параметры
    m = mass(t)
    T = thrust(t)
    g = gravity(h)
    rho = density(h)
    theta = np.radians(angle_of_attack(h))  # угол в радианах
    
    # Аэродинамическое сопротивление (направлено против скорости)
    if v > 0:
        Fd = 0.5 * rho * v**2 * Cd * S
        Fdx = -Fd * (vx / v)  # Знак минус - против движения
        Fdy = -Fd * (vy / v)
    else:
        Fdx, Fdy = 0.0, 0.0
    
    # Проекции тяги
    Tx = T * np.cos(theta)
    Ty = T * np.sin(theta)
    
    # Уравнения движения
    dvx_dt = (Tx + Fdx) / m
    dvy_dt = (Ty + Fdy) / m - g
    dx_dt = vx
    dh_dt = vy
    
    return [dvx_dt, dvy_dt, dx_dt, dh_dt]

# РЕШЕНИЕ СИСТЕМЫ УРАВНЕНИЙ

# Начальные условия: ракета на стартовом столе
y0 = [0.0, 0.0, 0.0, 0.0]  # [vx, vy, x, h]

# Время интегрирования 
t_max = t_SRB + t_main -25  #уменьшаем время, так как остается топливо 
t_eval = np.linspace(0, t_max, 1000)

# Решение системы уравнений
solution = solve_ivp(
    rocket_equations,
    [0, t_max],
    y0,
    t_eval=t_eval,
    method='RK45',
    rtol=1e-6,
    atol=1e-9
)

# Извлечение результатов
t = solution.t
vx = solution.y[0]
vy = solution.y[1]
x = solution.y[2]
h = solution.y[3]
v = np.sqrt(vx**2 + vy**2)


# данные для графиков из KSP
f = open('flight_data.txt', 'r', encoding='utf-8')

times, speeds, altitudes, os_x, masses = [], [], [], [], []
for line in f:
    line = line.strip()  # Убираем пробелы и символы новой строки
    if line:  # Проверяем, что строка не пустая
        t_ksp, s, a, x_ksp, m = line.split(',')  # Разделяем строку на 3 части
        times.append(float(t_ksp))
        speeds.append(float(s))
        altitudes.append(float(a))
        os_x.append(float(x_ksp))
        masses.append(float(m))

f.close()


# ПОСТРОЕНИЕ ГРАФИКОВ

# 1. Скорость от времени
plt.figure(figsize=(10, 6))
plt.plot(t, v, 'b--', linewidth=2, label='Полная скорость')
plt.plot(times, speeds, 'g-', linewidth=1.5, label='KSP данные')
plt.xlabel('Время (с)')
plt.ylabel('Скорость (м/c)')
plt.title('График зависимости скорости от времени')
plt.grid(True)
plt.show()
plt.savefig('flight_speed.png', dpi=300, bbox_inches='tight')

# 2. Высота от времени
plt.figure(figsize=(10, 6))
plt.plot(times, altitudes, 'g-', linewidth=1.5, label='KSP данные')
plt.plot(t, h, 'b--', linewidth=2)
plt.xlabel('Время (с)')
plt.ylabel('Высота (м)')
plt.title('График зависимости высоты от времени')
plt.grid(True)
plt.show()
plt.savefig('flight_altitude.png', dpi=300, bbox_inches='tight')

# 3. Масса от времени
mass_vals = np.array([mass(ti) for ti in t])
plt.figure(figsize=(10, 6))
plt.plot(times, masses, 'g-', linewidth=1.5, label='KSP данные')
plt.plot(t, mass_vals, 'b--', linewidth=2)
plt.xlabel('Время (с)')
plt.ylabel('масса (кг)')
plt.title('График зависимости массы от времени')
plt.grid(True)
plt.show()
plt.savefig('flight_altitude.png', dpi=300, bbox_inches='tight')

plt.tight_layout()
plt.show()
