"""Consistent Overhead Byte Stuffing. encode() output contains no 0x00."""


def encode(data: bytes) -> bytes:
    out = bytearray(1)
    code_pos, code = 0, 1
    for b in data:
        if b:
            out.append(b)
            code += 1
            if code < 0xFF:
                continue
        out[code_pos] = code
        code_pos, code = len(out), 1
        out.append(0)
    out[code_pos] = code
    return bytes(out)


def decode(data: bytes) -> bytes:
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        end = i + code
        if code == 0 or end > n:
            raise ValueError("invalid COBS block")
        out += data[i + 1:end]
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)
