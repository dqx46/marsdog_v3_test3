import serial
import struct

# ==========================
# 配置
# ==========================
PORT = "/dev/ttyUSB0"
BAUD = 115200

# 打开串口
ser = serial.Serial(
    port=PORT,
    baudrate=BAUD,
    bytesize=8,
    parity=serial.PARITY_NONE,
    stopbits=1,
    timeout=1
)


def read_motor_status(motor_id):
    # --------------------------
    # 组帧
    # --------------------------
    cmd = bytearray([
        0x3E,
        0x9A,
        motor_id,
        0x00
    ])

    cmd.append(sum(cmd) & 0xFF)

    print(f"\n========== 电机 ID {motor_id} ==========")
    print("发送:", cmd.hex(' '))

    # 清空接收缓存
    ser.reset_input_buffer()

    # 发送
    ser.write(cmd)
    ser.flush()

    # 接收
    reply = ser.read(13)

    print("收到:", reply.hex(' '))

    if len(reply) != 13:
        print("没有收到完整回复！")
        return

    # --------------------------
    # 校验CMD
    # --------------------------
    cmd_sum = (reply[0] + reply[1] + reply[2] + reply[3]) & 0xFF

    if cmd_sum != reply[4]:
        print("CMD校验失败！")
        return

    # --------------------------
    # 校验DATA
    # --------------------------
    data_sum = sum(reply[5:12]) & 0xFF

    if data_sum != reply[12]:
        print("DATA校验失败！")
        return

    # --------------------------
    # 解析数据
    # --------------------------
    temperature = struct.unpack("b", reply[5:6])[0]

    voltage = struct.unpack("<H", reply[6:8])[0] / 100

    current = struct.unpack("<h", reply[8:10])[0] / 100

    motor_state = reply[10]

    error_state = reply[11]

    print("温度      :", temperature, "℃")
    print("母线电压  :", voltage, "V")
    print("母线电流  :", current, "A")

    if motor_state == 0x00:
        print("电机状态  : 开启")
    elif motor_state == 0x10:
        print("电机状态  : 关闭")
    else:
        print(f"电机状态  : 0x{motor_state:02X}")

    if error_state == 0x00:
        print("错误状态  : 无故障")
    else:
        print(f"错误状态  : 0x{error_state:02X}")


# ==========================
# 读取两个电机
# ==========================
read_motor_status(0x01)
read_motor_status(0x02)

ser.close()