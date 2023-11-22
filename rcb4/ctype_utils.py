def c_type_to_size(c_type):
    return {'uint8': 1,
            'uint16': 2,
            'float': 4,
            'double': 8,
            'int': 4,
            'int16': 2,
            'uint32': 4}[c_type]
