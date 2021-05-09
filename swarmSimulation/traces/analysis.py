import pandas as pd
import matplotlib.pyplot as plt

trace_200 = pd.read_csv('traces_200.csv')
trace_150 = pd.read_csv('traces_150.csv')
trace_100 = pd.read_csv('traces_100.csv')
trace_80 = pd.read_csv('traces_80.csv')
trace_60 = pd.read_csv('traces_60.csv')
trace_40 = pd.read_csv('traces_40.csv')
trace_20 = pd.read_csv('traces_20.csv')


# Plot with time to process 1 simulation second
# def simulation_second_processing_time(trace: pd.DataFrame, title: str):
#     avg_simulation_sec = trace['avg_simulation_second']
#     avg_simulation_sec = avg_simulation_sec.apply(lambda x: float(x[5:]))
#     return avg_simulation_sec.plot(
#         kind='line',
#         title=title,
#         xlabel='Simulation seconds',
#         ylabel='Process time (seconds)'
#     )


# # Parse time from avg_simulation_second to float
# def get_avg_time_float(trace: pd.DataFrame):
#     return trace['avg_simulation_second'].apply(lambda x: float(x[5:]))


# def get_avg_of_average_time(trace: pd.DataFrame):
#     avg_times = trace['avg_simulation_second'].apply(lambda x: float(x[5:]))
#     return avg_times.mean()

def parse_row(trace: pd.DataFrame):
    avg_times = trace['avg_simulation_second'].apply(lambda x: float(x[5:]))
    total_times = trace['total_time'].apply(lambda x: float(x[5:]) + (float(x[2:4]) * 60))
    for i in range(len(total_times)-1, 0, -1):
        total_times[i] = total_times[i] - total_times[i-1]
    return {
        'Tempo da Simulação': trace['env_time'].iloc[-1],
        'Min. tempo 1s': min(total_times),
        'Max. tempo 1s': max(total_times),
        'Delta': max(total_times) - min(total_times),
        'Média': avg_times.mean()
    }

# simulation_second_processing_time(trace_80, 'Time to process 1 simulation second, 80 drones')
# plt.show()
traces = [trace_20, trace_40, trace_60, trace_80, trace_100, trace_150, trace_200]
names = [20, 40, 60, 80, 100, 150, 200]
new_frame = pd.DataFrame(columns=['No de Drones', 'Tempo da Simulação', 'Min. tempo 1s', 'Max. tempo 1s', 'Delta', 'Média'])
for i in range(len(traces)):
    data = parse_row(traces[i])
    data['No de Drones'] = names[i]
    new_frame = new_frame.append(data, ignore_index=True)

print(new_frame.to_latex())


# avg_per_size = [(n, get_avg_of_average_time(t)) for t, n in zip(traces, names)]

# plt.plot([x[0] for x in avg_per_size], [x[1] for x in avg_per_size], marker='.')
# plt.xlabel('Drone count')
# plt.ylabel('Processing time (second)')
# plt.title('Average processing time of 1 simulation second')
# plt.show()
