import usb.core
import usb.util
import platform
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading

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
    def __init__(self, tracker1, tracker2):
        """
        Initialize the RealTimePlot object with two mouse trackers.

        Args:
            tracker1 (MouseTracker): The first mouse tracker object.
            tracker2 (MouseTracker): The second mouse tracker object.
        """
        self.tracker1 = tracker1
        self.tracker2 = tracker2
        self.fig, self.ax = plt.subplots(2, 1, figsize=(10, 8))

        # Time domain plots for y-positions
        self.line_y1_time, = self.ax[0].plot([], [], lw=2, label=f'{self.tracker1.name} Y-Position')
        self.line_y2_time, = self.ax[1].plot([], [], lw=2, label=f'{self.tracker2.name} Y-Position')
        self.ax[0].set_ylabel(f'{self.tracker1.name} Y-Position (m)')
        self.ax[1].set_ylabel(f'{self.tracker2.name} Y-Position (m)')
        self.ax[0].legend()
        self.ax[1].legend()

        # Add grid lines
        self.ax[0].grid(True)
        self.ax[1].grid(True)

    def init_plot(self):
        """
        Initialize the plot lines.

        Returns:
            tuple: The initialized line objects for the y-position plots of tracker1 and tracker2.
        """
        self.line_y1_time.set_data([], [])
        self.line_y2_time.set_data([], [])
        return self.line_y1_time, self.line_y2_time

    def update_plot(self, frame):
        """
        Update the plot with the latest y-position data.

        Args:
            frame: The current frame number (not used in this method).

        Returns:
            tuple: The updated line objects for the y-position plots of tracker1 and tracker2.
        """
        with self.tracker1.lock:
            self.line_y1_time.set_data(self.tracker1.time_data, self.tracker1.y_position_data)
        with self.tracker2.lock:
            self.line_y2_time.set_data(self.tracker2.time_data, self.tracker2.y_position_data)

        # Adjust x-axis limits for time domain plots
        max_time = max(self.tracker1.time_data[-1], self.tracker2.time_data[-1])
        if max_time > 10:
            self.ax[0].set_xlim(max_time - 10, max_time)
            self.ax[1].set_xlim(max_time - 10, max_time)
        else:
            self.ax[0].set_xlim(0, 10)
            self.ax[1].set_xlim(0, 10)

        self.ax[0].set_xlabel('Time (s)')
        self.ax[1].set_xlabel('Time (s)')

        # Autoscale y-axis for each subplot
        for axis in self.ax:
            axis.relim()
            axis.autoscale_view()

        return self.line_y1_time, self.line_y2_time

    def run(self):
        ani = animation.FuncAnimation(self.fig, self.update_plot, init_func=self.init_plot, interval=1, save_count=100)
        plt.show()

if __name__ == "__main__":
    # Mouse details
    vid = 0x1532
    pid = 0x0098
    dpi = 6400

    # Identify the devices
    mice = list_two_usb_devices(vid, pid)
    if len(mice) != 2:
        raise ValueError("Could not find exactly two mice with the given VID and PID")

    # Assign mice based on port numbers for differentiation
    mouse1, mouse2 = mice

    # Create tracker instances
    tracker1 = MouseTracker(mouse1['device'], dpi, f'Mouse1 Port {mouse1["port_number"]}')
    tracker2 = MouseTracker(mouse2['device'], dpi, f'Mouse2 Port {mouse2["port_number"]}')

    # Start the data processing threads
    t1 = threading.Thread(target=tracker1.process_data)
    t2 = threading.Thread(target=tracker2.process_data)
    t1.daemon = True
    t2.daemon = True
    t1.start()
    t2.start()

    plotter = RealTimePlot(tracker1, tracker2)
    plotter.run()

    # Stop the data processing threads when the plotting is done
    tracker1.stop()
    tracker2.stop()
    t1.join()
    t2.join()