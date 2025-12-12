import krpc, time
import matplotlib.pyplot as plt

conn = krpc.connect()
v = conn.space_center.active_vessel
space_center = conn.space_center

v.control.sas = False
v.control.rcs = False 
v.control.throttle = 1.0
body = v.orbit.body
launchpad_frame = body.reference_frame

def v_position():
    pos = v.position(launchpad_frame)
    return (pos[0]**2 + pos[2]**2)**0.5  # Горизонтальное расстояние

#график начало 
times, speeds, altitudes, os_x, masses = [], [], [], [], []
start_time = time.time()

# Старт
v.control.activate_next_stage()
time.sleep(2)

# Потоки
alt = conn.add_stream(getattr, v.flight(), 'mean_altitude')
spd = conn.add_stream(getattr, v.flight(v.orbit.body.reference_frame), 'speed')
solid = conn.add_stream(v.resources.amount, 'SolidFuel')
alt_p = conn.add_stream(getattr, v.flight(), 'surface_altitude')




# УЧАСТОК С SRB 
while solid() > 1.0:
    current_alt = alt()
    current_speed = spd()
    # РУЧНОЕ УПРАВЛЕНИЕ рысканием через control.yaw
    if current_alt < 3000:
        v.control.yaw = 0.0  # летим вертикально
    else:
        v.control.yaw = 1.0  # на максимальном рыскании
    
    times.append(time.time() - start_time)
    speeds.append(spd())
    altitudes.append(alt_p())
    os_x.append(v_position())
    masses.append(v.mass)


# Сбрасываем ручное управление
v.control.yaw = 0.0
time.sleep(1)
print("\nОтделение SRB \n Активация центрального двигателя")
v.control.activate_next_stage()
time.sleep(1)

# Потоки орбиты
orbit = v.orbit
apo = conn.add_stream(getattr, orbit, 'apoapsis_altitude')
peri = conn.add_stream(getattr, orbit, 'periapsis_altitude')
time_to_apo = conn.add_stream(getattr, orbit, 'time_to_apoapsis')
flight_surf = v.flight(v.surface_reference_frame)

def liquid():
    # общее количество LiquidFuel (для простоты)
    return v.resources.amount('LiquidFuel')

# Включаем автопилот
ap = v.auto_pilot
ap.reference_frame = v.surface_reference_frame
ap.engage()
pro = flight_surf.prograde  
ap.target_direction = pro
ap.target_pitch_and_heading(75, 90)

time.sleep(2)
v.control.throttle = 1.0
print("Разворот и набор скорости центральным двигателем")
target_apo = 90_000

while apo() < 100_000:
    h = alt()
    pause = 0
    apoc = apo()
    t = time_to_apo()
    base_pitch = 75
    if h < 30_000:
        base_pitch = 70
        base_th = 1.0   
    elif h < 40_000:
        base_pitch = 60
        base_th = 0.9
    elif h < 50_000:
        base_pitch = 45
        base_th = 0.7
    elif h < 76_000:
        base_pitch = 35
        base_th = 0.7   
    else:
        base_pitch = 5
        base_th = 0.3

    if t < 20 and apoc < 90_000:
        pitch = base_pitch + 10
    elif t < 40 and apoc > 90_000:
        pitch = base_pitch - 10
    else:
        pitch = base_pitch

    if t < 20 and spd() < 1800:
        th = 1.0
        pause = 3
    elif( t > 60 and h > 30_000 ) or ( spd() > 1800 and t > 40):
        th = 0.1
    else:
        th = base_th
    
    pitch = max(0, min(90, pitch))
    ap.target_pitch_and_heading(pitch, 90)
    v.control.throttle = th
    time.sleep(2+pause)

    times.append(time.time() - start_time)
    speeds.append(spd())
    altitudes.append(alt_p())
    os_x.append(v_position())
    masses.append(v.mass)

v.control.throttle = 0.0
time.sleep(2)
v.control.activate_next_stage()
time.sleep(2)

#график конец
# 1. Скорость от времени
plt.figure(figsize=(10, 6))
plt.plot(times, speeds)
plt.xlabel('Время (с)')
plt.ylabel('Скорость (м/c)')
plt.title('График зависимости скорости от времени')
plt.grid(True)
plt.show()
plt.savefig('flight_speed.png', dpi=300, bbox_inches='tight')

# 2. Высота от времени
plt.figure(figsize=(10, 6))
plt.plot(times, altitudes)
plt.xlabel('Время (с)')
plt.ylabel('Высота (м)')
plt.title('График зависимости высоты от времени')
plt.grid(True)
plt.show()
plt.savefig('flight_altitude.png', dpi=300, bbox_inches='tight')

# 3. Масса от времени
plt.figure(figsize=(10, 6))
plt.plot(times, masses)
plt.xlabel('Время (с)')
plt.ylabel('масса (кг)')
plt.title('График зависимости массы от времени')
plt.grid(True)
plt.show()
plt.savefig('flight_altitude.png', dpi=300, bbox_inches='tight')

plt.tight_layout()
plt.show()
with open('flight_data.txt', 'w', encoding='utf-8') as f:
    for t, s, a, x, m in zip(times, speeds, altitudes, os_x, masses):
        f.write(f"{t:.2f},{s:.2f},{a:.2f},{x:.2f},{m:.2f}\n")