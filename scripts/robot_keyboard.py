
import os
import sys
import time
import select
import termios
import tty

from dynamixel_sdk import *

# ---------------- CONFIGURATION ----------------

PORT = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL = 2.0

BASE_ID = 11

ADDR_TORQUE = 64
ADDR_MODE = 11
ADDR_DRIVE = 10
ADDR_ERROR = 70
ADDR_PRESENT = 132
ADDR_GOAL = 116
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_VOLTAGE = 144
ADDR_TEMP = 146

STEP = 5                 # Approximately 0.44 degrees
MIN_POS = 0
MAX_POS = 4095

port = PortHandler(PORT)
packet = PacketHandler(PROTOCOL)

old_terminal = None


def signed32(value):
    return value if value < 2**31 else value - 2**32


def read1(address):
    value, result, error = packet.read1ByteTxRx(
        port, BASE_ID, address
    )
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError("Motor communication/read error")
    return value


def read2(address):
    value, result, error = packet.read2ByteTxRx(
        port, BASE_ID, address
    )
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError("Motor communication/read error")
    return value


def read4(address):
    value, result, error = packet.read4ByteTxRx(
        port, BASE_ID, address
    )
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError("Motor communication/read error")
    return signed32(value)


def write1(address, value):
    result, error = packet.write1ByteTxRx(
        port, BASE_ID, address, value
    )
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError("Motor write error")


def write4(address, value):
    result, error = packet.write4ByteTxRx(
        port, BASE_ID, address, value
    )
    if result != COMM_SUCCESS or error != 0:
        raise RuntimeError("Motor write error")


def shortest_difference(a, b):
    difference = abs((a % 4096) - (b % 4096))
    return min(difference, 4096 - difference)


def get_key():
    ready, _, _ = select.select([sys.stdin], [], [], 0.1)
    if ready:
        return sys.stdin.read(1).lower()
    return None


def stop_motor():
    try:
        write1(ADDR_TORQUE, 0)
        print("\nBASE TORQUE DISABLED.")
    except Exception as error:
        print("\nCould not disable torque:", error)


try:
    if not port.openPort():
        raise RuntimeError("Cannot open U2D2")

    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError("Cannot set baud rate")

    print("\nOPENMANIPULATOR-X BASE TEST")
    print("=" * 45)

    # Check before enabling torque.
    if read1(ADDR_TORQUE) != 0:
        raise RuntimeError("Base torque is already enabled.")

    if read1(ADDR_MODE) != 3:
        raise RuntimeError("Unexpected operating mode.")

    if read1(ADDR_DRIVE) != 4:
        raise RuntimeError("Unexpected drive mode.")

    if read1(ADDR_ERROR) != 0:
        raise RuntimeError("Hardware error detected.")

    if read4(ADDR_PROFILE_ACCEL) != 100:
        raise RuntimeError("Profile acceleration is not 100 ms.")

    if read4(ADDR_PROFILE_VELOCITY) != 500:
        raise RuntimeError("Profile time is not 500 ms.")

    present = read4(ADDR_PRESENT) % 4096
    goal = read4(ADDR_GOAL) % 4096

    print("Present position:", present)
    print("Goal position:", goal)
    print("Voltage:", read2(ADDR_VOLTAGE) / 10, "V")
    print("Temperature:", read1(ADDR_TEMP), "C")

    if shortest_difference(present, goal) > 2:
        raise RuntimeError(
            "Present and goal positions differ. Aborting."
        )

    print("\nPre-check passed.")
    print("Enabling BASE motor only...")

    write1(ADDR_TORQUE, 1)
    time.sleep(0.3)

    if read1(ADDR_TORQUE) != 1:
        raise RuntimeError("Base torque did not enable.")

    new_present = read4(ADDR_PRESENT) % 4096

    if shortest_difference(new_present, present) > 5:
        raise RuntimeError(
            "Position changed unexpectedly after torque enable."
        )

    print("Base torque enabled.")
    print("Starting position:", new_present)

    print("\nKEYBOARD CONTROLS")
    print("A = Base decrease")
    print("D = Base increase")
    print("Q = Stop and disable base torque")
    print("\nEach key press moves only 5 encoder counts.")
    print("Keep your hand away from moving joints.")

    old_terminal = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    while True:
        key = get_key()

        if key == "q":
            break

        if key not in ("a", "d"):
            continue

        current = read4(ADDR_PRESENT) % 4096

        if key == "a":
            target = current - STEP
        else:
            target = current + STEP

        target = max(MIN_POS, min(MAX_POS, target))

        if target == current:
            print("Position limit reached.")
            continue

        write4(ADDR_GOAL, target)

        print(
            f"Key: {key.upper()} | "
            f"Current: {current} | "
            f"Commanded target: {target}"
        )

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nKeyboard interrupt.")

except Exception as error:
    print("\nSTOP:", error)

finally:
    if old_terminal is not None:
        termios.tcsetattr(
            sys.stdin, termios.TCSADRAIN, old_terminal
        )

    if port.is_open:
        stop_motor()
        port.closePort()

    print("Communication closed.")
