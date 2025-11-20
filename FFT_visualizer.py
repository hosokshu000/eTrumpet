import serial
import matplotlib.pyplot as plt
import numpy as np

SERIAL_PORT = "/dev/cu.usbmodem14201" # Arduino port
BAUD_RATE = 115200 # Arduino baud rate

# Number of FFT bins
NUM_SAMPLES = 128
SAMPLING_FREQUENCY = 5000 # The same as in the Arduino code
FREQ_MAX = 1500 # Maximum frequency to display (Hz)
TICK_INTERVAL = 100 # Frequency interval between ticks (Hz)

def parse_fft_data(data):
    #Parse FFT data wrapped in <FFT>...</FFT>.
    if "<FFT>" in data and "</FFT>" in data:
        start = data.index("<FFT>") + len("<FFT>")
        end = data.index("</FFT>")
        fft_string = data[start:end]
        try:
            fft_values = list(map(float, fft_string.split(",")))
            return fft_values
        except ValueError:
            print("Error parsing FFT values.")
            return None
    return None

def parse_peak_data(data):
    #Parse peak frequency data wrapped in <PEAK>...</PEAK>.
    if "<PEAK>" in data and "</PEAK>" in data:
        start = data.index("<PEAK>") + len("<PEAK>")
        end = data.index("</PEAK>")
        try:
            peak_value = float(data[start:end])
            return peak_value
        except ValueError:
            print("Error parsing peak value.")
            return None
    return None

def get_frequency_for_bin(bin_index, sampling_frequency, num_samples):
    #Convert a frequency bin index to its corresponding frequency.
    return (bin_index * sampling_frequency) / num_samples

def main():
    # Connect to the serial port
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

    # Initialize FFT graph
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 6))

    # Calculate frequency values for each bin
    freq_values = [get_frequency_for_bin(i, SAMPLING_FREQUENCY, NUM_SAMPLES) for i in range(NUM_SAMPLES//2)]
    
    # Create initial empty bars using frequency values for x-coordinates
    bars = ax.bar(freq_values[:NUM_SAMPLES//2], [0] * (NUM_SAMPLES//2), width=freq_values[1]-freq_values[0], color='green')

    # Set up the axis labels and title
    ax.set_xlabel("Frequency (Hz)")
    ax.set_ylabel("Magnitude")
    ax.set_title("Real-Time FFT Visualization")

    # Set x-axis limits and ticks
    ax.set_xlim(0, FREQ_MAX)
    ax.set_xticks(np.arange(0, FREQ_MAX + TICK_INTERVAL, TICK_INTERVAL))
    
    max_magnitude = 1 # Start with a minimum Y-axis scaling value
    peak_annotation = None # For annotating the peak frequency
    current_peak_frequency = None # To store the most recent peak frequency

    try:
        buffer = ""
        while True:
            # Read from serial
            if ser.in_waiting > 0:
                buffer += ser.read(ser.in_waiting).decode("utf-8", errors="ignore")

                # Parse FFT data
                if "</FFT>" in buffer:
                    fft_data = parse_fft_data(buffer)
                    buffer = buffer[buffer.index("</FFT>") + len("</FFT>"):] # Remove processed data

                    if fft_data:
                        # Update the graph
                        current_max = max(fft_data)
                        if current_max > max_magnitude:
                            max_magnitude = current_max

                        for i, bar in enumerate(bars):
                            if i < len(fft_data):
                                bar.set_height(fft_data[i])

                        ax.set_ylim(0, max_magnitude * 1.1) # Add a 10% margin above the highest value

                # Parse peak data
                if "<PEAK>" in buffer and "</PEAK>" in buffer:
                    current_peak_frequency = parse_peak_data(buffer)
                    buffer = buffer[buffer.index("</PEAK>") + len("</PEAK>"):] # Remove processed data

                # Annotate the peak frequency if available
                if current_peak_frequency is not None and fft_data is not None:
                    closest_bin = int((current_peak_frequency / SAMPLING_FREQUENCY) * NUM_SAMPLES)
                    if closest_bin < len(bars):
                        if peak_annotation:
                            peak_annotation.remove()
                        peak_annotation = ax.annotate(
                            f"{current_peak_frequency:.1f} Hz",
                            xy=(current_peak_frequency, fft_data[closest_bin]),
                            xytext=(current_peak_frequency, fft_data[closest_bin] + max_magnitude * 0.05),
                            arrowprops=dict(facecolor='red', shrink=0.05),
                            fontsize=10,
                            color="red"
                        )

                        plt.pause(0.01)

    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()