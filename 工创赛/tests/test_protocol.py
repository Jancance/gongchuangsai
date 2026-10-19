import unittest

from smart_carrier.protocol import (
    Command,
    IdempotencyCache,
    Packet,
    StreamParser,
    crc16_ccitt_false,
    decode_json_payload,
    encode_json_payload,
    encode_packet,
)


class ProtocolTests(unittest.TestCase):
    def test_json_payload_is_deterministic_and_round_trips(self):
        payload = encode_json_payload({"status": "DONE", "seq": 9})
        self.assertEqual(payload, b'{"seq":9,"status":"DONE"}')
        self.assertEqual(decode_json_payload(payload), {"seq": 9, "status": "DONE"})

    def test_crc_known_vector(self):
        self.assertEqual(crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_parser_handles_noise_partial_and_concatenated_frames(self):
        first = Packet(7, Command.HEARTBEAT, b"abc")
        second = Packet(8, Command.STATUS, b"xyz")
        data = b"noise" + encode_packet(first) + encode_packet(second)
        parser = StreamParser()
        result = []
        for index in range(0, len(data), 3):
            result.extend(parser.feed(data[index : index + 3]))
        self.assertEqual(result, [first, second])

    def test_bad_crc_is_rejected_and_next_packet_recovers(self):
        corrupt = bytearray(encode_packet(Packet(1, Command.STATUS, b"bad")))
        corrupt[-4] ^= 0x40
        good = Packet(2, Command.STATUS, b"good")
        parser = StreamParser()
        self.assertEqual(parser.feed(bytes(corrupt) + encode_packet(good)), [good])
        self.assertEqual(parser.crc_errors, 1)

    def test_idempotency_cache_returns_final_response(self):
        cache = IdempotencyCache(capacity=2)
        response = Packet(10, Command.DONE, b"ok")
        cache.remember(4, response)
        self.assertEqual(cache.lookup(4), response)


if __name__ == "__main__":
    unittest.main()
