import usb.core
import usb.util
import platform
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
from math import sin
import numpy as np

def list_two_usb_devices(vid, pid):
    """
    Find and list two USB devices with the specified Vendor ID (VID) and Product ID (PID), 
    including their port numbers for differentiation.

    Args:
        vid (int): The Vendor ID of the USB devices.
        pid (int): The Product ID of the USB devices.

    Returns:
        list[dict]: A list of dictionaries, each containing details about a matching USB device:
            - device: The USB device object.
            - port_number: The port number the device is connected to.
    """
    devices = usb.core.find(find_all=True)
    mice = []
    for device in devices:
        if device.idVendor == vid and device.idProduct == pid:
            try:
                port_number = device.port_number if hasattr(device, 'port_number') else 'N/A'
                
                mice.append({
                    'device': device,
                    'port_number': port_number,
                })
                
                if len(mice) == 2:
                    break

            except usb.core.USBError as e:
                print(f"Could not retrieve information for device with Vendor ID: {vid}, Product ID: {pid}. Error: {e}")
    
    return mice

class MouseTracker:
    def __init__(self, device, dpi, name):
        self.device = device
        self.dpi = dpi
        self.name = name
        self.counts_to_meters = 1 / dpi * 0.0254 # Change the unit to meters
        self.dev = self.setup_device()
        self.start_time = time.perf_counter()
        self.y_position = 0.0
        self.time_data = []
        self.y_position_data = []
        self.lock = threading.Lock()
        self.running = True

    def setup_device(self):
        """
        Set up the USB device for data communication.

        Returns:
            usb.core.Device: The configured USB device object.
        """
        if platform.system() == 'Windows':
            self.device.set_configuration()
        else:
            if self.device.is_kernel_driver_active(0):
                self.device.detach_kernel_driver(0)
            usb.util.claim_interface(self.device, 0)

        return self.device

    def read_data(self):
        """
        Read data from the USB device's endpoint.

        Returns:
            bytes or None: The data read from the device's endpoint as a byte array, or `None` if a timeout occurs.
        """
        try:
            endpoint = self.get_endpoint()
            data = self.device.read(endpoint.bEndpointAddress, endpoint.wMaxPacketSize, timeout=100)
            return data
        except usb.core.USBError as e:
            if e.errno in (110, 10060):  # Ignore ALL Timeout error
                return None
            else:
                raise

    def get_endpoint(self):
        """
        Retrieve the input endpoint of the USB device.

        Returns:
            usb.core.Endpoint: The IN endpoint of the USB device for data reception.

        Raises:
            ValueError: If no IN endpoint is found on the device.
        """
        cfg = self.device.get_active_configuration()
        interface_number = cfg[(0, 0)].bInterfaceNumber
        interface = usb.util.find_descriptor(cfg, bInterfaceNumber=interface_number)
        endpoint = usb.util.find_descriptor(
            interface,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )
        if endpoint is None:
            raise ValueError('Endpoint not found')
        return endpoint

    def process_data(self):
        """
        Continuously read and process data from the USB device.

        Description:
        This method runs in a loop while `self.running` is True. It reads data from the mouse
        using the `read_data` method, calculates the elapsed time and y-axis movement, and updates 
        the y-position and time data. A lock was used to ensure thread safety when updating shared data.

        1. If data is successfully read (mouse is moving), it extracts the y-movement from the data, converts it to meters, 
          and updates the y-position, time data, and y-position data.
        2. If no data is read (mouse is stationary), it updates the time data and y-position data 
          with the current elapsed time and the last y-position.

        Additionally, this method ensures that the shared data structures (`time_data` and `y_position_data`) are 
        safely updated in a multi-threaded environment using a threading lock.
        """
        while self.running:
            data = self.read_data()
            if data:
                elapsed_time = time.perf_counter() - self.start_time
                y_movement = int.from_bytes(data[2:3], byteorder='little', signed=True) # The second and third bytes are x and y movement

                with self.lock:
                    self.y_position += y_movement * self.counts_to_meters # Mouse only output relative movement
                    self.time_data.append(elapsed_time)
                    self.y_position_data.append(self.y_position)
            else:
                elapsed_time = time.perf_counter() - self.start_time
                with self.lock:
                    self.time_data.append(elapsed_time)
                    self.y_position_data.append(self.y_position)


    def stop(self):
        self.running = False

class RealTimePlot:
    def __init__(self, tracker1, tracker2, x1, x2, x_ea):
        """
        Initialize the RealTimePlot object with two mouse trackers and important x-coordinates.

        Args:
            tracker1 (MouseTracker): The first mouse tracker object.
            tracker2 (MouseTracker): The second mouse tracker object.
            x1 (float): The x-coordinate of the first mouse.
            x2 (float): The x-coordinate of the second mouse.
            x_ea (float): The x-coordinate of the effective axis.
        """
        self.tracker1 = tracker1
        self.tracker2 = tracker2
        self.x1 = x1
        self.x2 = x2
        self.x_ea = x_ea
        self.fig, self.ax = plt.subplots(4, 1, figsize=(10, 16))

        # Time domain plots
        self.line_h, = self.ax[0].plot([], [], lw=2, label='h (Vertical Displacement)')
        self.line_alpha, = self.ax[1].plot([], [], lw=2, label='alpha (Pitch Angle)')
        self.ax[0].set_ylabel('h (m)')
        self.ax[1].set_ylabel('alpha (radians)')
        self.ax[0].legend()
        self.ax[1].legend()

        # Frequency domain plots
        self.line_h_freq, = self.ax[2].plot([], [], lw=2, label='h (Frequency Domain)')
        self.line_alpha_freq, = self.ax[3].plot([], [], lw=2, label='alpha (Frequency Domain)')
        self.ax[2].set_ylabel('h (magnitude)')
        self.ax[3].set_ylabel('alpha (magnitude)')
        self.ax[2].legend()
        self.ax[3].legend()

        # Fix the frequency range for frequency domain plots
        self.freq_range = (0, 10)
        
        # Add grid lines to all subplots
        for axis in self.ax:
            axis.grid(True) 

    def init_plot(self):
        """
        Initialize the plot lines.

        Returns:
            tuple: The initialized line objects for the vertical displacement and pitch angle plots.
        """
        self.line_h.set_data([], [])
        self.line_alpha.set_data([], [])
        self.line_h_freq.set_data([], [])
        self.line_alpha_freq.set_data([], [])
        return self.line_h, self.line_alpha, self.line_h_freq, self.line_alpha_freq

    def update_plot(self, frame):
        """
        Update the plot with the latest vertical displacement and pitch angle data.

        Args:
            frame: The current frame number (not used in this method).

        Returns:
            tuple: The updated line objects for the vertical displacement and pitch angle plots.
        """
        with self.tracker1.lock, self.tracker2.lock:
            time_data = self.tracker1.time_data
            y1_data = self.tracker1.y_position_data
            y2_data = self.tracker2.y_position_data

            # Ensure both trackers have the same length of data
            min_length = min(len(time_data), len(y1_data), len(y2_data))
            time_data = time_data[:min_length]
            y1_data = y1_data[:min_length]
            y2_data = y2_data[:min_length]

            # Calculate alpha and h
            alpha_data = [(y1 - y2) / (self.x2 - self.x1) for y1, y2 in zip(y1_data, y2_data)]
            h_data = [y1 + alpha * sin(alpha) * abs(self.x1 - self.x_ea) for y1, alpha in zip(y1_data, alpha_data)]

            # Check if no new data was received
            if len(self.tracker1.time_data) > 0:
                last_time = self.tracker1.time_data[-1]
                if len(time_data) == 0 or time_data[-1] != last_time:
                    time_data.append(last_time)
                    h_data.append(h_data[-1])
                    alpha_data.append(alpha_data[-1])

            # Compute FFT for frequency domain
            if len(h_data) > 1:
                h_fft = np.fft.fft(h_data)
                alpha_fft = np.fft.fft(alpha_data)
                freqs = np.fft.fftfreq(len(h_data), np.mean(np.diff(time_data)))

                # Select positive frequencies within the desired range
                pos_mask = (freqs > 0) & (freqs <= self.freq_range[1])
                freqs = freqs[pos_mask]
                h_fft = np.abs(h_fft[pos_mask])
                alpha_fft = np.abs(alpha_fft[pos_mask])
            else:
                freqs = np.array([])
                h_fft = np.array([])
                alpha_fft = np.array([])

        # Update plots
        self.line_h.set_data(time_data, h_data)
        self.line_alpha.set_data(time_data, alpha_data)
        self.line_h_freq.set_data(freqs, h_fft)
        self.line_alpha_freq.set_data(freqs, alpha_fft)

        # Adjust x-axis limits for time domain plots
        if time_data:
            max_time = time_data[-1]
            if max_time > 10:
                self.ax[0].set_xlim(max_time - 10, max_time)
                self.ax[1].set_xlim(max_time - 10, max_time)
            else:
                self.ax[0].set_xlim(0, max_time)
                self.ax[1].set_xlim(0, max_time)

        self.ax[0].set_xlabel('Time (s)')
        self.ax[1].set_xlabel('Time (s)')

        # Adjust x-axis limits for frequency domain plots
        self.ax[2].set_xlim(self.freq_range)
        self.ax[3].set_xlim(self.freq_range)

        self.ax[2].set_xlabel('Frequency (Hz)')
        self.ax[3].set_xlabel('Frequency (Hz)')

        # Autoscale y-axis for each subplot
        for axis in self.ax:
            axis.relim()
            axis.autoscale_view()

        return self.line_h, self.line_alpha, self.line_h_freq, self.line_alpha_freq

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=1, save_count=100)
        plt.show()

if __name__ == "__main__":
    # Switch !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    Plan = False  # Change it if you want to switch the assignment

    # Mouse details
    vid = 0x1532
    pid = 0x0098
    dpi = 6400

    # Airfoil parameters
    c = 0.1  # Chord length of the airfoil
    x1 = -0.11 # rear sensor location
    x2 = 0.09 # front sensor location
    x_ea = 0
    
    # Identify the devices
    mice = list_two_usb_devices(vid, pid)
    if len(mice) != 2:
        raise ValueError("Could not find exactly two mice with the given VID and PID")

    # Switch assignment based on Plan
    if Plan:
        mouse1, mouse2 = mice[0], mice[1]
    else:
        mouse1, mouse2 = mice[1], mice[0]

    # Create tracker instances
    tracker1 = MouseTracker(mouse1['device'], dpi, 'Mouse1')
    tracker2 = MouseTracker(mouse2['device'], dpi, 'Mouse2')

    # Start the data processing threads
    t1 = threading.Thread(target=tracker1.process_data)
    t2 = threading.Thread(target=tracker2.process_data)
    t1.daemon = True
    t2.daemon = True
    t1.start()
    t2.start()

    plotter = RealTimePlot(tracker1, tracker2, x1, x2, x_ea)
    plotter.run()

    # Stop the data processing threads when the plotting is done
    tracker1.stop()
    tracker2.stop()
    t1.join()
    t2.join()
