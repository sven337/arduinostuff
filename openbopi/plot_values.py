#!/usr/bin/python

import pandas as pd
import matplotlib.pyplot as plt
from datetime import datetime

# Read the file with custom parsing
def parse_line(line):
    parts = line.strip().split()
    timestamp = datetime.strptime(' '.join(parts[:3]), '%b %d %H:%M:%S.%f')
    sensor = parts[3].split('/')[1]
    value = float(parts[4])
    return timestamp, sensor, value

# Read the file and create lists for DataFrame
timestamps = []
ph_values = []
orp_values = []

with open('sensor_data.txt', 'r') as file:
    current_time = None
    current_ph = None
    current_orp = None
    
    for line in file:
        timestamp, sensor, value = parse_line(line)
        
        if sensor == 'pH':
            current_ph = value
        else:  # ORP
            current_orp = value
            
        if current_ph is not None and current_orp is not None:
            timestamps.append(pd.to_datetime(timestamp))
            ph_values.append(current_ph)
            orp_values.append(current_orp)
            current_ph = None
            current_orp = None

# Create DataFrame
df = pd.DataFrame({
    'timestamp': timestamps,
    'pH': ph_values,
    'ORP': orp_values
})

# Set timestamp as index
df.set_index('timestamp', inplace=True)

# Create figure and axis objects with a single subplot
fig, ax1 = plt.subplots(figsize=(10, 6))

# Plot pH on primary y-axis
color1 = 'tab:blue'
ax1.set_xlabel('Time')
ax1.set_ylabel('pH', color=color1)
ax1.plot(df.index, df['pH'], color=color1, label='pH')
ax1.tick_params(axis='y', labelcolor=color1)

# Create second y-axis that shares x-axis
ax2 = ax1.twinx()

# Plot ORP on secondary y-axis
color2 = 'tab:red'
ax2.set_ylabel('ORP (mV)', color=color2)
ax2.plot(df.index, df['ORP'], color=color2, label='ORP')
ax2.tick_params(axis='y', labelcolor=color2)

# Add legend
lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right')

# Rotate x-axis labels for better readability
plt.xticks(rotation=45)

# Adjust layout to prevent label cutoff
plt.tight_layout()

# Show the plot
plt.show()

