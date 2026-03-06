def calculate_nmea_crc(sentence):
    """
    计算NMEA0183协议的CRC校验码（8位）
    :param sentence: 要计算的字符串（不含末尾的*XX）
    :return: 两位十六进制的校验码字符串（大写）
    """
    # 初始化CRC值为0
    crc = 0
    # 遍历字符串中的每个字符（从$后开始，到最后一个字符）
    for char in sentence:
        # 跳过$符号（NMEA校验不包含$）
        if char == '$':
            continue
        # 按位异或计算CRC
        crc ^= ord(char)
    # 返回两位十六进制格式（02x），并转为大写
    return f"{crc:02X}"

print(calculate_nmea_crc("$NAVICTL,1,154,127,0,0,0"))