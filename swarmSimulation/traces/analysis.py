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
def simulation_second_processing_time(trace: pd.DataFrame, title: str):
    avg_simulation_sec = trace['avg_simulation_second']
    avg_simulation_sec = avg_simulation_sec.apply(lambda x: float(x[5:]))
    return avg_simulation_sec.plot(
        kind='line',
        title=title,
        xlabel='Simulation seconds',
        ylabel='Process time (seconds)'
    )


# Parse time from avg_simulation_second to float
def get_avg_time_float(trace: pd.DataFrame):
    return trace['avg_simulation_second'].apply(lambda x: float(x[5:]))


def get_avg_of_average_time(trace: pd.DataFrame):
    avg_times = trace['avg_simulation_second'].apply(lambda x: float(x[5:]))
    return avg_times.mean()


traces = [trace_20, trace_40, trace_60, trace_80, trace_100, trace_150, trace_200]
names = [20, 40, 60, 80, 100, 150, 200]

avg_per_size = [(n, get_avg_of_average_time(t)) for t, n in zip(traces, names)]

# plt.plot([x[0] for x in avg_per_size], [x[1] for x in avg_per_size], marker='.')
# plt.xlabel('Drone count')
# plt.ylabel('Processing time (second)')
# plt.title('Average processing time of 1 simulation second')
# plt.show()
