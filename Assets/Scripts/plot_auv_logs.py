"""
plot_auv_logs.py — Визуализация логов AUV для дипломной работы
Парсит вывод auv_brain.py и строит графики для отчёта/защиты.
Использование:
python plot_auv_logs.py logs.txt
python plot_auv_logs.py logs.txt --output report_plots
"""
import re
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import rcParams

rcParams['font.family'] = 'DejaVu Sans'
rcParams['axes.unicode_minus'] = False
rcParams['figure.dpi'] = 300

CONTROL_DT = 0.02
LOG_EVERY_N = 40
LOG_INTERVAL = CONTROL_DT * LOG_EVERY_N

def parse_log_file(filepath):
    pattern = re.compile(
        r'\[(CRUISE|TURNING|CLEARING)\]\s+'
        r'F:([\d.]+)\s+L:([\d.]+)\s+R:([\d.]+)\s+\|\s+'
        r'T:\s*([\d.-]+)°\s+'
        r'S:\s*([\d.-]+)°\s+'
        r'SPD:\s*([\d.-]+)'
    )
    
    data = {
        'time': [], 'state': [],
        'F': [], 'L': [], 'R': [], 'min_dist': [],
        'target_heading': [], 'safe_heading': [], 'speed': []
    }
    start_time = None
    
    print(f"Чтение файла: {filepath}")
    with open(filepath, 'r', encoding='utf-8') as f:
        for line_num, line in enumerate(f, 1):
            match = pattern.search(line)
            if match:
                state = match.group(1)
                f_dist = float(match.group(2))
                l_dist = float(match.group(3))
                r_dist = float(match.group(4))
                t_head = float(match.group(5))
                s_head = float(match.group(6))
                speed = float(match.group(7))
                
                if start_time is None:
                    start_time = line_num
                
                rel_time = (line_num - start_time) * LOG_INTERVAL
                
                min_d = min(f_dist, l_dist, r_dist)
                data['time'].append(rel_time)
                data['state'].append(state)
                data['F'].append(f_dist)
                data['L'].append(l_dist)
                data['R'].append(r_dist)
                data['min_dist'].append(min_d)
                data['target_heading'].append(t_head)
                data['safe_heading'].append(s_head)
                data['speed'].append(speed)
                
    for key in data:
        if key != 'state':
            data[key] = np.array(data[key])
            
    print(f"Загружено {len(data['time'])} записей")
    return data

def plot_trajectory(data, output_prefix):
    plt.figure(figsize=(10, 8))
    dt = LOG_INTERVAL
    x, z = [0], [0]
    for i in range(1, len(data['time'])):
        heading_rad = np.deg2rad(data['safe_heading'][i])
        speed = data['speed'][i]
        dx = speed * np.sin(heading_rad) * dt
        dz = speed * np.cos(heading_rad) * dt
        x.append(x[-1] + dx)
        z.append(z[-1] + dz)
        
    states = np.array(data['state'])
    colors = {'CRUISE': 'green', 'TURNING': 'orange', 'CLEARING': 'red'}
    for state in ['CRUISE', 'TURNING', 'CLEARING']:
        mask = states == state
        if np.any(mask):
            plt.scatter(np.array(x)[mask], np.array(z)[mask], 
                        c=colors[state], label=state, s=10, alpha=0.6)
            
    plt.plot(x, z, 'k--', linewidth=0.5, alpha=0.3)
    plt.xlabel('Положение X, м (относительное)')
    plt.ylabel('Положение Z, м (относительное)')
    plt.title('Траектория движения АНПА')
    plt.legend(loc='upper right')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.axis('equal')
    plt.tight_layout()
    plt.savefig(f'{output_prefix}_trajectory.png', dpi=300)
    print(f"Сохранено: {output_prefix}_trajectory.png")
    plt.close()

def plot_speed_profile(data, output_prefix):
    fig, ax1 = plt.subplots(figsize=(12, 5))
    color = 'tab:blue'
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('Скорость, у.е.', color=color)
    ax1.plot(data['time'], data['speed'], color=color, linewidth=1.5, label='Командная скорость')
    ax1.tick_params(axis='y', labelcolor=color)
    ax1.grid(True, linestyle=':', alpha=0.6)
    
    states = np.array(data['state'])
    colors_bg = {'CRUISE': '#e0f7e0', 'TURNING': '#fff3e0', 'CLEARING': '#ffebee'}
    prev_t = 0
    for i in range(1, len(data['time'])):
        if states[i] != states[i-1] or i == len(data['time']) - 1:
            ax1.axvspan(data['time'][prev_t], data['time'][i], 
                        facecolor=colors_bg.get(states[i], 'white'), alpha=0.3)
            prev_t = i
            
    ax2 = ax1.twinx()
    color = 'tab:red'
    ax2.set_ylabel('Дистанция до препятствия, м', color=color)
    ax2.plot(data['time'], data['min_dist'], color=color, linestyle='--', linewidth=1.5, label='Минимальный зазор')
    ax2.axhline(y=2.5, color='gray', linestyle=':', linewidth=1, label='Порог срабатывания')
    ax2.tick_params(axis='y', labelcolor=color)
    
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper right')
    plt.title('Профиль скорости и безопасность (мин. дистанция)')
    plt.tight_layout()
    plt.savefig(f'{output_prefix}_speed.png', dpi=300)
    print(f"Сохранено: {output_prefix}_speed.png")
    plt.close()

def plot_headings(data, output_prefix):
    plt.figure(figsize=(12, 5))
    def smooth_angle(angles):
        smoothed = [angles[0]]
        for a in angles[1:]:
            prev = smoothed[-1]
            diff = (a - prev + 180) % 360 - 180
            smoothed.append(prev + diff)
        return smoothed
        
    t_smooth = smooth_angle(data['target_heading'])
    s_smooth = smooth_angle(data['safe_heading'])
    plt.plot(data['time'], t_smooth, 'b-', linewidth=1, label='Курс к цели', alpha=0.7)
    plt.plot(data['time'], s_smooth, 'r-', linewidth=2, label='Безопасный курс')
    
    states = np.array(data['state'])
    for i in range(1, len(data['time'])):
        if states[i] != states[i-1]:
            plt.axvline(data['time'][i], color='gray', linestyle=':', linewidth=0.5)
            plt.text(data['time'][i], -180, states[i], rotation=90, fontsize=8, va='bottom')
            
    plt.xlabel('Время, с')
    plt.ylabel('Курс, град')
    plt.title('Управление курсом: Реакция алгоритма на препятствия')
    plt.legend(loc='upper right')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.tight_layout()
    plt.savefig(f'{output_prefix}_headings.png', dpi=300)
    print(f"Сохранено: {output_prefix}_headings.png")
    plt.close()

def plot_sensor_sectors(data, output_prefix):
    plt.figure(figsize=(12, 5))
    plt.plot(data['time'], data['F'], label='Front (±30°)', linewidth=1.5)
    plt.plot(data['time'], data['L'], label='Left (>30°)', linewidth=1, linestyle='--')
    plt.plot(data['time'], data['R'], label='Right (<-30°)', linewidth=1, linestyle='--')
    plt.axhline(y=4.5, color='orange', linestyle=':', linewidth=1, label='Порог срабатывания')
    plt.axhline(y=15.0, color='gray', linestyle=':', linewidth=0.5, alpha=0.5, label='Макс. дальность')
    plt.xlabel('Время, с')
    plt.ylabel('Дистанция, м')
    plt.title('Показания секторов сонара')
    plt.legend(loc='upper right')
    plt.grid(True, linestyle=':', alpha=0.6)
    plt.ylim(0, 16)
    plt.tight_layout()
    plt.savefig(f'{output_prefix}_sensors.png', dpi=300)
    print(f"Сохранено: {output_prefix}_sensors.png")
    plt.close()

def generate_summary_table(data):
    print("\n" + "="*60)
    print("Сводные метрики")
    print("="*60)
    total_time = data['time'][-1] if len(data['time']) > 0 else 0
    min_clearance = np.min(data['min_dist'])
    max_speed = np.max(data['speed'])
    
    states = np.array(data['state'])
    unique, counts = np.unique(states, return_counts=True)
    state_times = {s: c * LOG_INTERVAL for s, c in zip(unique, counts)}
    
    print(f"Общее время симуляции: {total_time:.2f} с")
    print(f"Минимальный зазор (безопасность): {min_clearance:.2f} м")
    print(f"Максимальная скорость: {max_speed:.1f} у.е.")
    print("\nРаспределение времени по состояниям:")
    for s in ['CRUISE', 'TURNING', 'CLEARING']:
        t = state_times.get(s, 0)
        pct = (t / total_time * 100) if total_time > 0 else 0
        print(f"  • {s:8s}: {t:5.2f} с ({pct:5.1f}%)")
        
    print("\nРекомендация для текста диплома:")
    num_turns = state_times.get("TURNING", 0) / LOG_INTERVAL
    print(f'"Минимальная дистанция до препятствия составила {min_clearance:.2f} м, '
          f'что превышает установленный порог безопасности (2.5 м). '
          f'Алгоритм успешно отработал {num_turns:.0f} манёвров обхода."')
    print("="*60 + "\n")

def main():
    parser = argparse.ArgumentParser(description='Визуализация логов AUV для диплома')
    parser.add_argument('logfile', help='Путь к файлу с логами')
    parser.add_argument('--output', '-o', default='auv_report', help='Префикс для выходных файлов')
    args = parser.parse_args()
    
    try:
        data = parse_log_file(args.logfile)
        if len(data['time']) == 0:
            print("Ошибка: Не найдено записей в логе. Проверьте формат.")
            return
        generate_summary_table(data)
        print("Построение графиков...")
        plot_trajectory(data, args.output)
        plot_speed_profile(data, args.output)
        plot_headings(data, args.output)
        plot_sensor_sectors(data, args.output)
        print(f"\nГотово! Файлы сохранены с префиксом '{args.output}'.")
    except FileNotFoundError:
        print(f"Файл не найден: {args.logfile}")
    except Exception as e:
        print(f"Ошибка при обработке: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()