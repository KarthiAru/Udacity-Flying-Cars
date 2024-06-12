import pandas as pd
import matplotlib.pyplot as plt
import os

# Create the plot directory if it doesn't exist
plot_directory = 'plots'
if not os.path.exists(plot_directory):
    os.makedirs(plot_directory)

# Load data from the log file
df = pd.read_csv('Logs/flight_data_log.txt', sep=',')
df['timestamp'] = pd.to_datetime(df['timestamp'], unit='s')
df.to_csv("Logs/data.csv", index=False)
print(df.shape)

# Create the plot directory if it doesn't exist
plot_directory = 'plots'
if not os.path.exists(plot_directory):
    os.makedirs(plot_directory)

# Plotting function that dynamically adjusts y-axis
def plot_data(filtered_df, x, y, title, ylabel, filename):
    if filtered_df.empty or filtered_df[y].dropna().empty:
        print(f"No valid data to plot for {title}. Skipping plot.")
        return
    
    plt.figure(figsize=(10, 6))
    for col in y:
        if filtered_df[col].notna().any():  # Check if there are any non-NaN values
            plt.plot(filtered_df[x], filtered_df[col], label=col)
    plt.xlabel('Time')
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid(True)
    
    # Calculate limits if possible
    min_val = filtered_df[y].min().min()
    max_val = filtered_df[y].max().max()
    if pd.notna(min_val) and pd.notna(max_val):
        plt.ylim([min_val * 1.1, max_val * 1.1])
    
    plt.savefig(f'{plot_directory}/{filename}')
    plt.close()

# Function to filter data and plot each dataset
def filter_and_plot(columns, title, ylabel, filename):
    plot_df = df[columns].dropna(subset=columns[1:])  # Drop rows where the needed columns have NaN values
    print(plot_df.shape)
    plot_data(plot_df, 'timestamp', columns[1:], title, ylabel, filename)

# Execute plotting
filter_and_plot(['timestamp', 'x', 'y', 'z'], 'Local Position over Time', 'Local Position Coordinates', 'local_position.png')
filter_and_plot(['timestamp', 'vx', 'vy', 'vz'], 'Local Velocity over Time', 'Velocity Components', 'local_velocity.png')