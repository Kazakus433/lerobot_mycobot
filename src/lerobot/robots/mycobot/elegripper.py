import serial
import time
import threading


class Command():
    """Communication message:
        Header(int):Frame Header
        Len(int):length
        ID(int):Gripper ID
        Code(int):Function code(6-write,3-read)
        Number_High(int):Instruction number high byte
        Number_LOW(int):Instruction number low byte
        Value_High(int):Parameter high byte
        Value_LOW(int):Parameter low byte

    """
    Header = 254
    Len = 8
    ID = 14
    Code = 0
    Zero = 0
    Number_High = 0
    Number_LOW = 0
    Value_High = 0
    Value_LOW = 0
    cmd_list = [Header, Header, Len, ID, Code, Number_High, Number_LOW, Value_High, Value_LOW]


class Gripper(Command):

    def __init__(self, port, baudrate=115200, id=14):
        """
        Args:
            port : Serial port number
            baudrate : Baud rate,Defaults to 115200.
            id: Gripper ID, default is 1
        """

        self.lock = threading.Lock()
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate,
                                 timeout=5)  # timeout is the timeout period, the default value is 5 seconds
        self.cmd_list[3] = id

    def check_value(self, value, lower, upper, index=1):
        valid_values = list(range(lower, upper + 1))  # 生成有效值范围

        # 判断输入是单个值还是列表
        if isinstance(value, list):
            # 遍历列表中的每个值
            for i, val in enumerate(value):
                if val not in valid_values:
                    raise ValueError(
                        f"The {index} input value at position {i + 1} is invalid. Valid values between [{lower},{upper}]")
                    return False
            return True
        else:
            # 单个值的处理逻辑
            if value in valid_values:
                return True
            else:
                if index == 1:
                    raise ValueError(f"The first input value can be selected as: [{lower},{upper}]")
                else:
                    raise ValueError(f"The second input value can be selected as: [{lower},{upper}]")
                return False

    def __byte_deal(self, value1, value2):
        """Convert decimal to hexadecimal

        Args:
            value1 (int): Instruction number
            value2 (int): parameter

        Returns:
            Return hexadecimal data
        """
        high_byte1 = (value1 >> 8) & 0xFF  # High Byte
        low_byte1 = value1 & 0xFF  # Low Byte
        high_byte2 = (value2 >> 8) & 0xFF  # High Byte
        low_byte2 = value2 & 0xFF  # Low Byte
        return [high_byte1, low_byte1, high_byte2, low_byte2]

    def __crc16_modbus(self, data: bytes) -> bytes:
        """CRC16 modbus check bit generation

        Args:
            data (bytes): Message

        Returns:
            bytes: Check digit
        """
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 0x0001) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='big')

    def __send_cmd(self, cmd, is_special_interface=False):
        """Processing Messages

        Args:
            cmd(byte) : Message

        Raises:
            TimeoutError: Reply timeout

        Returns:
            Response Reply,If the data length is incorrect, -1 is returned; if the CRC check is incorrect, -2 is returned.
        """
        with self.lock:
            send_data = cmd + self.__crc16_modbus(cmd)
            # print(send_data.hex())
            self.ser.write(send_data)
            self.ser.flush()
            time.sleep(0.04)
            recv_data = self.ser.read(11)
            if not recv_data:
                raise TimeoutError("Reading data timeout")
            # print(recv_data.hex())
            if len(recv_data) == 11:
                data = recv_data[0:9]
                crc_data = recv_data[9:]
                if self.__crc16_modbus(data) == crc_data:
                    response = data + crc_data
                    if is_special_interface:
                        result = int(response.hex()[14:16], 16)
                    else:
                        result = int(response.hex()[14:18], 16)
                    return result
                else:
                    return -2

            else:
                return -1

    def set_gripper_value(self, value, speed=100):
        """Setting the gripper position

        Args:
            value (int): Angle range 0-100
            speed (int): Speed ​​range 0-100

        Returns:
           Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 100):
            if self.check_value(speed, 1, 100, index=2):
                self.set_gripper_speed(speed)
                self.cmd_list[4] = 6
                tmp = self.__byte_deal(11, value)
                for i in range(5, 9):
                    self.cmd_list[i] = tmp[i - 5]
                cmd = bytes(self.cmd_list)
                return self.__send_cmd(cmd)

    def set_gripper_speed(self, value):
        """Setting the gripper speed

        Args:
            value (int): Speed ​​range 1-100

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 1, 100):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(32, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def set_gripper_enable(self, value):
        """Set the gripper to enable

        Args:
            value (int): 0 means disabled, 1 means enabled

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 1):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(10, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def set_gripper_calibration(self):
        """Setting the gripper Calibration

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(13, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_gripper_value(self):
        """Get the gripper position

        Returns:
            Response results:angle value: 0-100
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(12, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_gripper_status(self):
        """Get the gripper status

        Returns:
           Response results:
            0 - Moving
            1 - Stopped moving, no clamping detected
            2 - Stopped moving, clamping detected
            3 - After clamping detected, the object fell
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(14, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_firmware_version(self):
        """Get the firmware major version number

        Returns:
            Response results
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(1, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_modified_version(self):
        """Get the firmware minor version number

        Returns:
            Response results
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(2, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_Id(self, value):
        """Set the gripper ID

        Args:
            value (int):ID range:1 - 254

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 1, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(3, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            self.cmd_list[3] = value
            return self.__send_cmd(cmd)

    def get_gripper_Id(self):
        """Get the gripper ID

        Returns:
            Response results
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(4, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_baud(self, value=0):
        """Setting the baud rate

        Args:
            value (int): baud rate selection range:
                0 - 115200 (default),
                1 - 1000000,
                2 - 57600,
                3 - 19200,
                4 - 9600,
                5 - 4800,

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 5):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(5, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_baud(self):
        """Get the baud rate

        Returns:
            Response results:
                0 - 115200 (default),
                1 - 1000000,
                2 - 57600,
                3 - 19200,
                4 - 9600,
                5 - 4800,

        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(6, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_P(self, value):
        """Set the gripper P value

        Args:
            value (int):The value range is 0-254

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(15, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_P(self):
        """Get the gripper P value

        Returns:
            Response results:0-254
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(16, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_D(self, value):
        """Set the gripper D value

        Args:
            value (int):The value range is 0-254

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(17, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_D(self):
        """Get the gripper P value

        Returns:
            Response results:0-254
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(18, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_I(self, value):
        """Set the gripper I value

        Args:
            value (int):The value range is 0-254

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(19, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_I(self):
        """Get the gripper I value

        Returns:
            Response results:0-254
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(20, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_cw(self, value):
        """Set the clockwise runnable error of the gripper

        Args:
            value (int): The value range is 0-16

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 16):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(21, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_cw(self):
        """Get the clockwise runnable error of the gripper

        Returns:
            Response results:0-16
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(22, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_cww(self, value):
        """Set the anti-clockwise runnable error of the gripper

        Args:
            value (int): The value range is 0-16

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 16):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(23, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_cww(self):
        """Get the anti-clockwise runnable error of the gripper

        Returns:
            Response results:0-16
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(24, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_mini_pressure(self, value):
        """Setting the minimum actuation force of the gripper

        Args:
            value (int): The value range is 0-254

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(25, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_mini_pressure(self):
        """Get the minimum starting force of the gripper

        Returns:
            Response results:0-254
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(26, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_torque(self, value):
        """Setting the gripper jaw torque

        Args:
            value (int): The value range is 0-100

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 100):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(27, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_torque(self):
        """Get the gripper jaw torque

        Returns:
            Response results:0-100
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(28, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_output(self, value=0):
        """Set the gripper OUT output

        Args:
            value (int): The value range is 0-3
                0-out1 off,out2 off;
                1-out1 on ,out2 off;
                2-out1 off,out2 on;
                3-out1 on ,out2 on;

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 3):
            if value == 2:
                value = 16
            elif value == 3:
                value = 17
        else:
            return None
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(29, value)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_io_open_value(self, value):
        """Set the io opening angle

        Args:
            value (int): The value range is 0-100

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 100):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(30, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def set_gripper_io_close_value(self, value):
        """Set io closing angle

        Args:
            value (int):The value range is 0-100

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 100):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(31, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_speed(self):
        """Get the gripper speed

        Returns:
            Response results:1-100
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(33, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_gripper_io_open_value(self):
        """Get the io opening angle

        Returns:
            Response results:0-100
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(34, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_gripper_io_close_value(self):
        """Get io closing angle

        Returns:
            Response results:0-100
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(35, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_abs_gripper_value(self, value, speed=100):
        """Set the absolute angle that the gripper reaches


        Args:
            value (int): The value range is 0-100
            speed (int): The speed range is 1-100

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 100):
            if self.check_value(speed, 1, 100):
                self.set_gripper_speed(speed)
                self.cmd_list[4] = 6
                tmp = self.__byte_deal(36, value)
                for i in range(5, 9):
                    self.cmd_list[i] = tmp[i - 5]
                cmd = bytes(self.cmd_list)
                return self.__send_cmd(cmd)

    def set_gripper_pause(self):
        """Set the gripper to pause motion

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(37, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_resume(self):
        """Set the gripper recovery motion

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(38, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_stop(self):
        """Set the command to stop the gripper and clear the cache

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(39, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_gripper_queue_count(self):
        """Get the amount of data in the current queue of the gripper

        Returns:
            Response results
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(40, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_vir_pos(self, value):
        """Set the virtual position value of the servo

        Args:
            value (int): The value range is 0-100

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 100):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(41, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_vir_pos(self):
        """Get the virtual position value of the servo

        Returns:
            Response results:0-100
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(42, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_protection_current(self, value):
        """Set the gripper clamping current

        Args:
            value (int): The value range is 100-300

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 100, 300):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(43, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_protection_current(self):
        """Get the gripper clamping current

        Returns:
            Response results:0-100
        """
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(44, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_state(self, value, speed=100):
        """Set the gripper position to fully open or fully closed

        Args:
            value (int): The value range is 0-1
                0-close;
                1-open

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 1):
            if self.check_value(speed, 1, 100):
                self.set_gripper_speed(speed)
                if value == 1:
                    return self.set_gripper_value(100)
                elif value == 0:
                    return self.set_gripper_value(0)

    def set_gripper_value_return_angle(self, value):
        """Set the gripper position to a specific angle

        Args:
            value (int): The value range is 0-100

        Returns:
            Response results:current angle 0-100
        """
        if self.check_value(value, 0, 100):
            self.cmd_list[4] = 6
        tmp = self.__byte_deal(46, value)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd,is_special_interface=True)

    def set_modbus(self, value):
        """Set the gripper modbus

        Args:
            value (int): The value range is 100-300

        Returns:
            Response results:0 represents failure, 1 represents success
        """
        if self.check_value(value, 0, 1):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(47, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def close(self):
        self.ser.close()