import asyncio, struct
from bleak import BleakScanner, BleakClient

CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"

def on_data(sender, data):
    t, ax, ay, az, gx, gy, gz = struct.unpack("<Ihhhhhh", data)
    print(f"t={t} ms  acc=({ax/1000:.2f}, {ay/1000:.2f}, {az/1000:.2f}) g  "
          f"gyro=({gx/10:.1f}, {gy/10:.1f}, {gz/10:.1f}) deg/s")

async def main():
    print("Scanning...")
    dev = await BleakScanner.find_device_by_name("XIAO-IMU", timeout=15)
    if dev is None:
        print("Board not found")
        return
    async with BleakClient(dev) as client:
        print("Connected")
        await client.start_notify(CHAR_UUID, on_data)
        await asyncio.sleep(60)      # run for 60 seconds

asyncio.run(main())
