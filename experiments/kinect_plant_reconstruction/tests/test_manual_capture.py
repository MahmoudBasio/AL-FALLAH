import tempfile
import threading
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import plant_capture_manual as m


def message(stamp=1, endian=False):
    xyz = np.arange(18, dtype=np.float32).reshape(2, 3, 3) + 1
    dtype = '>f4' if endian else '<f4'
    data = b''.join(row.astype(dtype).tobytes() + b'PAD!' for row in xyz)
    return NS(height=2, width=3, point_step=12, row_step=40,
              is_bigendian=endian, data=data,
              fields=[NS(name=n, datatype=7, count=1, offset=i*4)
                      for i, n in enumerate('xyz')],
              header=NS(frame_id='camera', stamp=NS(sec=stamp, nanosec=0)))


class Tests(unittest.TestCase):
    def test_preview_geometry_and_frame(self):
        cloud = lambda: NS(header=NS())
        xyz = np.array([[1, 2, 3], [np.nan, 0, 1], [4, 5, 6]])
        msg = m.preview_message(xyz, 'kinect_depth', 123, cloud, NS)
        self.assertEqual(msg.header.frame_id, 'kinect_depth')
        self.assertEqual(msg.header.stamp, 123)
        self.assertEqual((msg.height, msg.width, msg.row_step), (1, 2, 24))
        self.assertEqual([f.offset for f in msg.fields], [0, 4, 8])
        np.testing.assert_equal(np.frombuffer(msg.data, dtype='<f4').reshape(-1, 3),
                                [[1, 2, 3], [4, 5, 6]])

    def test_decoder_padding_and_endian(self):
        expected = np.arange(18).reshape(2, 3, 3) + 1
        for endian in (False, True):
            np.testing.assert_equal(m.decode_xyz(message(endian=endian)), expected)

    def test_fresh_frames_and_duplicate_rejection(self):
        inbox = m.Inbox()
        inbox.push(message(1))
        def feed():
            for stamp in [2, 2, 3]:
                time.sleep(.04)
                inbox.push(message(stamp))
        worker = threading.Thread(target=feed)
        worker.start()
        frames, stamps, frame = m.capture(inbox, 2, 0, 1)
        worker.join()
        self.assertEqual(stamps.tolist(), [2000000000, 3000000000])
        self.assertEqual(frames.shape, (2, 2, 3, 3))
        self.assertEqual(frame, 'camera')

    def test_stale_cloud_times_out(self):
        inbox = m.Inbox()
        inbox.push(message())
        with self.assertRaises(TimeoutError):
            m.capture(inbox, 1, 0, .02)

    def test_save_and_invalid_pixels(self):
        frames = np.stack([m.decode_xyz(message())] * 3)
        frames[:2, 0, 0] = np.nan
        frames[0, 1, 1] = 1000
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)
            m.save_view(path, 1, frames, np.arange(3), 'camera')
            with np.load(path / 'view_000001.npz') as saved:
                self.assertFalse(saved['pose_known'])
                self.assertEqual(saved['xyz'].shape, (5, 3))
                np.testing.assert_equal(saved['xyz_frames'], frames)
                self.assertTrue(np.isnan(saved['xyz_organized'][0, 0]).all())
                np.testing.assert_equal(saved['xyz_organized'][1, 1], frames[1, 1, 1])
            data = (path / 'view_000001.ply').read_bytes().split(b'end_header\n')[1]
            self.assertEqual(len(data), 5 * 12)
            with self.assertRaises(FileExistsError):
                m.save_view(path, 1, frames, np.arange(3), 'camera')


if __name__ == '__main__':
    unittest.main()
