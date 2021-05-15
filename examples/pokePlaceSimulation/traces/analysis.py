import pandas as pd
import matplotlib.pyplot as plt

sizes = ["SMALL", "SMALL2", "MEDIUM", "MEDIUM2", "MEDIUM3", "LARGE"]
personnel = [10, 20, 30, 50, 70, 100]
pandas = [pd.read_csv(f'./traces_{size}.csv') for size in sizes]


# Plot with time to process 1 simulation second
def simulation_second_processing_time(trace: pd.DataFrame, title: str):
    avg_simulation_sec = trace['avg_simulation_second']
    avg_simulation_sec = avg_simulation_sec.apply(lambda x: float(x[5:]))
    simulation_time = trace['env_time'].apply(int)
    plt.plot(simulation_time, avg_simulation_sec, label=title)


def plot_avg_time_and_event_size(trace: pd.DataFrame, title: str):
    avg_simulation_sec = trace['avg_simulation_second']
    avg_simulation_sec = avg_simulation_sec.apply(lambda x: float(x[5:]))
    simulation_time = trace['env_time'].apply(int)
    event_count = trace['event_store_size'].apply(int)
    fig, ax = plt.subplots()
    ax.plot(simulation_time, avg_simulation_sec, label='Processing time')
    ax.set_ylabel('Processing time (second)', fontsize=14)
    ax.set_xlabel('Simulation time', fontsize=14)
    ax2 = ax.twinx()
    ax2.plot(simulation_time, event_count, color='red', label='Event store')
    ax2.set_ylabel('Event store size', fontsize=14)
    ax.set_title(title)
    plt.legend()
    plt.show()


def get_avg_of_average_time(trace: pd.DataFrame):
    avg_times = trace['avg_simulation_second'].apply(lambda x: float(x[5:]))
    return avg_times.mean()


# All series, one at a time, avg time vs event size
# for i in range(len(sizes)):
#     plot_avg_time_and_event_size(pandas[i], f'Time to process 1 simulation second, {personnel[i]} cooks')

# All series average time
# for i in range(len(sizes)):
#     simulation_second_processing_time(pandas[i], f'{personnel[i]} cooks')
#     plt.ylabel('Processing time (second)', fontsize=14)
#     plt.xlabel('Simulation time', fontsize=14)
#     plt.title('Average time to compute 1 simulation second', fontsize=16)
#     plt.legend(title='Simulation size')
#     plt.show()
# # Parse time from avg_simulation_second to float
# def get_avg_time_float(trace: pd.DataFrame):
#     return trace['avg_simulation_second'].apply(lambda x: float(x[5:]))


