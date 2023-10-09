import dynamixel_sdk as dxl
import math

# Dynamixel control parameters
PORT_NAME = 'COM3'  # Adjust this to match your setup
BAUD_RATE = 1000000
PROTOCOL_VERSION = 1  # Use 1.0 or 2.0 depending on your Dynamixel model
DXL_ID1 = 2  # ID of the first servo
DXL_ID2 = 3  # ID of the second servo


# Physical parameters of your robot
LINK1_LENGTH = 0.192  # Length of the first link
LINK2_LENGTH = 0.156  # Length of the second link
GRAVITY = 9.81  # Acceleration due to gravity (m/s^2)
MASS1 = 0.153  # Mass of the first link (kg)
MASS2 = 0.245  # Mass of the second link (kg)
COM1 = 0.131  # Center of mass of the first link (m)
COM2 = 0.0789  # Center of mass of the second link (m)

# Initialize the Dynamixel SDK
port_handler = dxl.PortHandler(PORT_NAME)
packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

def initialize_dynamixel():
    if port_handler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        return False

    if port_handler.setBaudRate(BAUD_RATE):
        print(f"Succeeded to set the baud rate to {BAUD_RATE}")
    else:
        print("Failed to set the baud rate")
        return False

    return True

def compute_gravity_compensation(theta1, theta2):
    # Compute forward kinematics to get the end-effector position
    x = LINK1_LENGTH * math.cos(theta1) + LINK2_LENGTH * math.cos(theta1 + theta2)
    y = LINK1_LENGTH * math.sin(theta1) + LINK2_LENGTH * math.sin(theta1 + theta2)

    # Compute torques for gravity compensation
    torque1 = GRAVITY * (MASS1 * COM1 + MASS2 * (LINK1_LENGTH + COM2)) * math.sin(theta1)
    torque2 = GRAVITY * MASS2 * COM2 * math.sin(theta1 + theta2)

    return torque1, torque2

if __name__ == '__main__':
    if not initialize_dynamixel():
        exit(1)

    while True:
        # Read current joint angles from the Dynamixel servos
        dxl1_present_position, _, _ = packet_handler.read4ByteTxRx(port_handler, DXL_ID1, 132)
        dxl2_present_position, _, _ = packet_handler.read4ByteTxRx(port_handler, DXL_ID2, 132)

        # Convert joint angles to radians
        theta1 = math.radians(dxl1_present_position)
        theta2 = math.radians(dxl2_present_position)

        # Compute gravity compensation torques
        torque1, torque2 = compute_gravity_compensation(theta1, theta2)

        # Send the computed torques to the Dynamixel servos (you'll need to implement this)

        # Print the torques for debugging
        print(f"Torque 1: {torque1} Nm")
        print(f"Torque 2: {torque2} Nm")
