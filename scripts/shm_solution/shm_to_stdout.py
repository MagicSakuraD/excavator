#!/usr/bin/env python3
"""
Shared Memory -> H.264 stdout (Pure GStreamer, no ROS)
- Reads raw RGB frames from /dev/shm
- Pushes via appsrc into NVENC encoder
- Writes H.264 byte-stream with AUD/SPS/PPS to stdout (fd=1)
"""

import gi
import os
import sys
import mmap
import time
import struct

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


class ShmToStdout:
    def __init__(self,
                 shm_path: str = '/dev/shm/isaac_rgb_buffer',
                 width: int = 1280,
                 height: int = 720,
                 fps: int = 20,
                 bitrate_kbps: int = 4000,
                 input_format: str = 'RGB'):
        self.shm_path = shm_path
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate_kbps = bitrate_kbps
        self.input_format = input_format

        # SHM layout
        self.header_size = 32
        self._update_image_size()

        # Open SHM (wait if not ready)
        wait_start = time.time()
        while not os.path.exists(self.shm_path):
            if time.time() - wait_start > 10:
                print(f'[ERR] SHM not found: {self.shm_path}', file=sys.stderr, flush=True)
                sys.exit(1)
            print(f'[INF] Waiting SHM: {self.shm_path}', file=sys.stderr, flush=True)
            time.sleep(0.5)

        self._open_shm_dynamic()

        # Init GStreamer and build pipeline
        Gst.init(None)
        self._build_pipeline()
        self._start_pipeline()

        # Frame tracking for SHM
        self.last_frame_counter = -1
        self.frame_duration_ns = int(Gst.SECOND // self.fps)

        # Use GLib timer for pushing frames without blocking main loop
        GLib.timeout_add(max(1, int(1000 / (self.fps * 2))), self._tick)

    def _update_image_size(self):
        if self.input_format in ['I420', 'NV12', 'YV12']:
            self.image_size = int(self.width * self.height * 1.5)
        elif self.input_format in ['RGB', 'BGR']:
            self.image_size = self.width * self.height * 3
        elif self.input_format in ['RGBA', 'BGRA']:
            self.image_size = self.width * self.height * 4
        else:
            # Default fallback to 3 channels if unknown
            self.image_size = self.width * self.height * 3
            
        self.total_size = self.header_size + self.image_size

    def _open_shm_dynamic(self):
        try:
            self.shm_fd = open(self.shm_path, 'rb')
            file_size = os.path.getsize(self.shm_path)
            if file_size < self.header_size:
                raise RuntimeError('SHM too small')
            self.shm_mmap = mmap.mmap(self.shm_fd.fileno(), file_size, access=mmap.ACCESS_READ)
            self.shm_mmap.seek(0)
            header = self.shm_mmap.read(self.header_size)
            _, w, h, _ = struct.unpack('QIIQ8x', header)
            if w != self.width or h != self.height:
                self.width, self.height = int(w), int(h)
                self._update_image_size()
            print(f'[INF] Opened SHM {self.shm_path} size={file_size} (w={self.width}, h={self.height}, fmt={self.input_format})', file=sys.stderr, flush=True)
        except Exception as e:
            print(f'[ERR] Open SHM failed: {e}', file=sys.stderr, flush=True)
            sys.exit(1)

    def _reopen_shm_for_resolution(self, new_w: int, new_h: int):
        try:
            if hasattr(self, 'shm_mmap') and self.shm_mmap:
                self.shm_mmap.close()
            if hasattr(self, 'shm_fd') and self.shm_fd:
                self.shm_fd.close()
        finally:
            pass
        deadline = time.time() + 3.0
        while True:
            try:
                self.shm_fd = open(self.shm_path, 'rb')
                file_size = os.path.getsize(self.shm_path)
                if file_size >= self.header_size:
                    self.shm_mmap = mmap.mmap(self.shm_fd.fileno(), file_size, access=mmap.ACCESS_READ)
                    self.shm_mmap.seek(0)
                    header = self.shm_mmap.read(self.header_size)
                    _, w, h, _ = struct.unpack('QIIQ8x', header)
                    if int(w) == int(new_w) and int(h) == int(new_h):
                        break
            except Exception:
                pass
            if time.time() > deadline:
                print('[ERR] SHM reopen timeout for new resolution', file=sys.stderr, flush=True)
                raise RuntimeError('SHM reopen timeout')
            time.sleep(0.05)
        self.width, self.height = int(new_w), int(new_h)
        self._update_image_size()
        print(f'[INF] Reopened SHM for {self.width}x{self.height}', file=sys.stderr, flush=True)

    def _pipeline_str(self):
        return (
            f"appsrc name=appsrc is-live=true format=time do-timestamp=false block=true ! "
            f"video/x-raw,format={self.input_format},width={self.width},height={self.height},framerate={self.fps}/1,interlace-mode=progressive,pixel-aspect-ratio=1/1 ! "
            f"videoconvert ! "
            f"video/x-raw,format=I420,width={self.width},height={self.height},framerate={self.fps}/1,interlace-mode=progressive,pixel-aspect-ratio=1/1 ! "
            f"nvh264enc bitrate={self.bitrate_kbps} preset=low-latency-hq rc-mode=cbr-ld-hq gop-size={self.fps} ! "
            f"video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline ! "
            f"h264parse config-interval=-1 ! "
            f"appsink name=sink emit-signals=true max-buffers=5 drop=true sync=false"
        )

    def _build_pipeline(self):
        pipeline_str = self._pipeline_str()
        print(f'[INF] Pipeline: {pipeline_str}', file=sys.stderr, flush=True)
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name('appsrc')
        if not self.appsrc:
            print('[ERR] appsrc not found', file=sys.stderr, flush=True)
            sys.exit(1)
        self.appsrc.set_property('format', Gst.Format.TIME)
        self.appsrc.set_property('is-live', True)
        self.appsrc.set_property('do-timestamp', False)
        self.appsrc.set_property('block', True)
        self.sink = self.pipeline.get_by_name('sink')
        if not self.sink:
            print('[ERR] appsink not found', file=sys.stderr, flush=True)
            sys.exit(1)
        self.sink.connect('new-sample', self._on_new_sample)
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect('message::error', self._on_err)
        self.bus.connect('message::eos', self._on_eos)
        self.bus.connect('message::warning', self._on_warn)

    def _start_pipeline(self):
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print('[ERR] Failed to start pipeline', file=sys.stderr, flush=True)
            sys.exit(1)
        print('[INF] Pipeline started', file=sys.stderr, flush=True)

    def _stop_pipeline(self):
        if hasattr(self, 'pipeline') and self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            time.sleep(0.02)
            self.pipeline = None

    def _read_one(self):
        self.shm_mmap.seek(0)
        header = self.shm_mmap.read(self.header_size)
        if len(header) < self.header_size:
            return None
        timestamp_ns, width, height, frame_counter = struct.unpack('QIIQ8x', header)
        if width != self.width or height != self.height:
            self._handle_resolution_change(width, height)
            return None
        if frame_counter == self.last_frame_counter:
            return None
        image_data = self.shm_mmap.read(self.image_size)
        if len(image_data) != self.image_size:
            return None
        self.last_frame_counter = frame_counter
        return timestamp_ns, image_data

    def _handle_resolution_change(self, new_w: int, new_h: int):
        print(f'[INF] Resolution change detected: {self.width}x{self.height} -> {new_w}x{new_h}', file=sys.stderr, flush=True)
        self._stop_pipeline()
        self._reopen_shm_for_resolution(new_w, new_h)
        self.frame_duration_ns = int(Gst.SECOND // self.fps)
        self._build_pipeline()
        self._start_pipeline()

    def _tick(self):
        try:
            item = self._read_one()
            if not item:
                return True
            timestamp_ns, image_data = item
            buf = Gst.Buffer.new_wrapped(image_data)
            buf.pts = timestamp_ns
            buf.dts = timestamp_ns
            buf.duration = self.frame_duration_ns
            ret = self.appsrc.emit('push-buffer', buf)
            if ret != Gst.FlowReturn.OK:
                # Keep running even if occasional backpressure
                pass
            # 不需要手动排队时间戳，使用 GStreamer Buffer 的 PTS 传递
        except Exception as e:
            print(f'[ERR] tick: {e}', file=sys.stderr, flush=True)
        return True

    def _on_err(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f'[GST][ERR] {err} dbg={dbg}', file=sys.stderr, flush=True)
        self.pipeline.set_state(Gst.State.NULL)
        GLib.idle_add(GLib.MainLoop().quit)

    def _on_eos(self, bus, msg):
        print('[GST][EOS]', file=sys.stderr, flush=True)
        self.pipeline.set_state(Gst.State.NULL)
        GLib.idle_add(GLib.MainLoop().quit)

    def _on_warn(self, bus, msg):
        w, dbg = msg.parse_warning()
        print(f'[GST][WRN] {w} dbg={dbg}', file=sys.stderr, flush=True)

    def _on_new_sample(self, sink):
        # Pull encoded H.264 access-unit and write with custom header [4B len][8B ts]
        sample = sink.emit('pull-sample')
        if not sample:
            return Gst.FlowReturn.ERROR
        buf = sample.get_buffer()
        if not buf:
            return Gst.FlowReturn.ERROR
        ok, map_info = buf.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.ERROR
        try:
            data = map_info.data
            # Use PTS from encoded buffer to keep timestamp aligned even if frames drop
            ts = int(buf.pts) if buf.pts is not None else 0
            # Write header + payload to stdout
            sys.stdout.buffer.write(struct.pack('>IQ', len(data), ts))
            sys.stdout.buffer.write(data)
            sys.stdout.buffer.flush()
        finally:
            buf.unmap(map_info)
        return Gst.FlowReturn.OK


def main():
    # Params from environment for simplicity
    shm_path = os.environ.get('SHM_PATH', '/dev/shm/isaac_rgb_buffer')
    width = int(os.environ.get('WIDTH', '1280'))
    height = int(os.environ.get('HEIGHT', '720'))
    fps = int(os.environ.get('FPS', '20'))
    bitrate_kbps = int(os.environ.get('BITRATE_KBPS', '4000'))
    input_format = os.environ.get('INPUT_FORMAT', 'RGB')

    app = ShmToStdout(shm_path, width, height, fps, bitrate_kbps, input_format)
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        pass
    finally:
        app.pipeline.set_state(Gst.State.NULL)
        if hasattr(app, 'shm_mmap'):
            app.shm_mmap.close()
        if hasattr(app, 'shm_fd'):
            app.shm_fd.close()


if __name__ == '__main__':
    main()
