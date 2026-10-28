"""Check C results against Python's arbitrary-precision integer arithmetic."""

import ctypes
import random
import sys


MAX_U32 = (1 << 32) - 1
SENTINEL = 0xDEADBEEF


def main():
    library = ctypes.CDLL(sys.argv[1])
    distance = library.encoder_math_distance_um
    distance.argtypes = [ctypes.c_uint32] * 3 + [ctypes.POINTER(ctypes.c_uint32)]
    distance.restype = ctypes.c_bool
    speed = library.encoder_math_speed_um_s
    speed.argtypes = [ctypes.c_uint32] * 4 + [ctypes.POINTER(ctypes.c_uint32)]
    speed.restype = ctypes.c_bool
    delta = library.encoder_math_count_delta
    delta.argtypes = [ctypes.c_uint32, ctypes.c_uint32]
    delta.restype = ctypes.c_uint32

    rng = random.Random(2004)
    boundary = [0, 1, 2, 3, 999999, 1000000, MAX_U32 - 1, MAX_U32]
    for index in range(5000):
        values = [rng.getrandbits(32) for _ in range(4)]
        if index < len(boundary) ** 2:
            values[0] = boundary[index % len(boundary)]
            values[1] = boundary[index // len(boundary)]
        count, travel, edges, elapsed = values
        if index % 3 == 0:
            elapsed = rng.choice(boundary)
        if index % 5 == 0:
            edges = rng.choice(boundary)

        for function, arguments, expected in [
            (distance, (count, travel, edges),
             count * travel // edges if travel and edges else None),
            (speed, (count, elapsed, travel, edges),
             count * travel * 1000000 // (edges * elapsed)
             if travel and edges and elapsed else None),
        ]:
            output = ctypes.c_uint32(SENTINEL)
            valid = function(*arguments, ctypes.byref(output))
            expected_valid = expected is not None and expected <= MAX_U32
            assert valid == expected_valid, (function.__name__, arguments, expected)
            assert output.value == (expected if expected_valid else SENTINEL), (
                function.__name__, arguments, expected, output.value
            )

        previous = rng.getrandbits(32)
        current = rng.getrandbits(32)
        assert delta(previous, current) == (current - previous) % (1 << 32)

    print("5000 reference cases passed (distance, speed and counter difference)")


if __name__ == "__main__":
    main()
