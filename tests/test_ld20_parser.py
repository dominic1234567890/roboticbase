from tcr_minibot.sensors.lidar_ld20 import FRAME_LEN, LD20Parser, crc8

EXAMPLE_PACKET = bytes.fromhex(
    "54 2C 68 08 AB 7E E0 00 E4 DC 00 E2 D9 00 E5 D5 00 "
    "E3 D3 00 E4 D0 00 E9 CD 00 E4 CA 00 E2 C7 00 E9 C5 "
    "00 E5 C2 00 E5 C0 00 E5 BE 82 3A 1A 50"
)


def test_example_packet_length():
    assert len(EXAMPLE_PACKET) == FRAME_LEN


def test_crc_example_packet():
    assert crc8(EXAMPLE_PACKET[:-1]) == EXAMPLE_PACKET[-1]


def test_parse_example_packet():
    parser = LD20Parser(check_crc=True)
    frame = parser.parse_frame(EXAMPLE_PACKET)
    assert frame is not None
    assert frame.crc_ok is True
    assert len(frame.points) == 12
    assert abs(frame.start_angle_deg_cw - 324.27) < 0.01
    assert abs(frame.end_angle_deg_cw - 334.70) < 0.01
    assert abs(frame.points[0].distance_m - 0.224) < 0.001
