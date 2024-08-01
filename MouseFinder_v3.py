import usb.core
import usb.util

def list_usb_devices():
    devices = usb.core.find(find_all=True)
    for device in devices:
        try:
            # Read the vendor and product strings
            vendor_id = device.idVendor
            product_id = device.idProduct

            # Try to get the product string
            try:
                product_string = usb.util.get_string(device, device.iProduct)
            except usb.core.USBError:
                product_string = 'Unknown'
            
            # Try to get the serial number string
            try:
                serial_number = usb.util.get_string(device, device.iSerialNumber)
            except usb.core.USBError:
                serial_number = 'N/A'
            
            # Try to get the manufacturer string
            try:
                manufacturer_string = usb.util.get_string(device, device.iManufacturer)
            except usb.core.USBError:
                manufacturer_string = 'Unknown'
            
            # Fetch bus and address as unique identifiers
            bus = device.bus
            address = device.address

            # Fetch port number
            port_number = device.port_number if hasattr(device, 'port_number') else 'N/A'

            # Fetch device class and subclass
            device_class = device.bDeviceClass
            device_subclass = device.bDeviceSubClass

            # Fetch interface number
            try:
                interface_number = device[0].interfaces()[0].bInterfaceNumber
            except IndexError:
                interface_number = 'N/A'
            
            print(f"Device: {product_string} (Vendor ID: {vendor_id}, Product ID: {product_id}, Serial Number: {serial_number}, Manufacturer: {manufacturer_string}, Bus: {bus}, Address: {address}, Port: {port_number}, Class: {device_class}, Subclass: {device_subclass}, Interface: {interface_number})")
        except Exception as e:
            print(f"Could not retrieve information for device with Vendor ID: {device.idVendor}, Product ID: {device.idProduct}. Error: {e}")

def main():
    list_usb_devices()

if __name__ == "__main__":
    main()
