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
    def __init__(self, device, dpi, name, initial_position):
        self.device = device
        self.dpi = dpi
        self.name = name
        self.counts_to_meters = 1 / dpi * 0.0254 # Change the unit to meters
        self.dev = self.setup_device()
        self.start_time = time.perf_counter()
        self.x_position, self.y_position = initial_position
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
        self.tracker1 = tracker1
        self.tracker2 = tracker2
        self.fig, self.ax = plt.subplots(figsize=(10, 8))

        # Initial positions
        self.line1, = self.ax.plot([tracker1.x_position], [tracker1.y_position], 'o', label=f'{self.tracker1.name} Position')
        self.line2, = self.ax.plot([tracker2.x_position], [tracker2.y_position], 'o', label=f'{self.tracker2.name} Position')
        self.line_connection, = self.ax.plot([tracker1.x_position, tracker2.x_position], [tracker1.y_position, tracker2.y_position], '-', label='Connection Line')

        self.ax.set_xlim(-0.1, 0.1)
        self.ax.set_ylim(-0.01, 0.01)
        self.ax.set_xlabel('X Position (m)')
        self.ax.set_ylabel('Y Position (m)')
        self.ax.legend()

    def init_plot(self):
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        self.line_connection.set_data([], [])
        return self.line1, self.line2, self.line_connection

    def update_plot(self, frame):
        with self.tracker1.lock:
            x1 = self.tracker1.x_position
            y1 = self.tracker1.y_position
        with self.tracker2.lock:
            x2 = self.tracker2.x_position
            y2 = self.tracker2.y_position

        self.line1.set_data([x1], [y1])
        self.line2.set_data([x2], [y2])
        self.line_connection.set_data([x1, x2], [y1, y2])

        # Autoscale x and y axes
        self.ax.relim()
        self.ax.autoscale_view()

        return self.line1, self.line2, self.line_connection

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

    # Identify the devices
    mice = list_two_usb_devices(vid, pid)
    if len(mice) != 2:
        raise ValueError("Could not find exactly two mice with the given VID and PID")

    # Initial positions
    initial_positions = [(-0.05, 0), (0.05, 0)]

    # Switch assignment based on Plan
    if Plan:
        mouse1, mouse2 = mice[0], mice[1]
        pos1, pos2 = initial_positions[0], initial_positions[1]
    else:
        mouse1, mouse2 = mice[1], mice[0]
        pos1, pos2 = initial_positions[1], initial_positions[0]

    # Create tracker instances
    tracker1 = MouseTracker(mouse1['device'], dpi, 'Mouse1', pos1)
    tracker2 = MouseTracker(mouse2['device'], dpi, 'Mouse2', pos2)

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