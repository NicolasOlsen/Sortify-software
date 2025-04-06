# crc16.py

class CRC16:
    def __init__(self):
        self.polynomial = 0x8001
        self.init_value = 0x0000

    def compute(self, data: bytes) -> int:
        crc = self.init_value
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ self.polynomial) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc
