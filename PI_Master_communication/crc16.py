# crc16.py

"""
This file were fully generated with support from ChatGPT, based on the GPT-4o model by OpenAI.

The generation prompt was based on implementing the same CRC-16 calculation as 
the CRC-16 class from Rob Tillaarts library "CRC".

All generated content has been reviewed, manually adjusted, and tested by the responsible developer to ensure correctness, 
maintainability, and adherence to the projects overall design principles.
"""

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
