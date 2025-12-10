#!/usr/bin/env python3
"""
audio_player.py - 接收 RTP 音频数据并通过扬声器播放

功能：
  - 监听 UDP 端口接收 RTP/Opus 音频包
  - 使用 GStreamer 解码 Opus 并输出到扬声器
  
架构：
  Go (WebRTC OnTrack) -> UDP -> audio_player.py -> GStreamer -> 扬声器
"""

import argparse
import signal
import sys
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib


class AudioPlayer:
    def __init__(self, port: int = 5005):
        self.port = port
        self.pipeline = None
        self.loop = None
        
        Gst.init(None)
        self._build_pipeline()
    
    def _build_pipeline(self):
        """
        构建 GStreamer 管道：
        udpsrc -> rtpopusdepay -> opusdec -> audioconvert -> audioresample -> autoaudiosink
        """
        pipeline_str = (
            f"udpsrc port={self.port} caps=\"application/x-rtp,media=audio,encoding-name=OPUS,payload=111\" ! "
            f"rtpopusdepay ! "
            f"opusdec ! "
            f"audioconvert ! "
            f"audioresample ! "
            f"autoaudiosink sync=false"
        )
        
        print(f'[INFO] Pipeline: {pipeline_str}', file=sys.stderr, flush=True)
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        if not self.pipeline:
            print('[ERROR] Failed to build pipeline', file=sys.stderr, flush=True)
            sys.exit(1)
        
        # 监听消息
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message::error', self._on_error)
        bus.connect('message::eos', self._on_eos)
        bus.connect('message::warning', self._on_warning)
    
    def _on_error(self, bus, msg):
        err, dbg = msg.parse_error()
        print(f'[GST][ERROR] {err} dbg={dbg}', file=sys.stderr, flush=True)
        self.stop()
    
    def _on_eos(self, bus, msg):
        print('[GST][EOS] End of stream', file=sys.stderr, flush=True)
        self.stop()
    
    def _on_warning(self, bus, msg):
        w, dbg = msg.parse_warning()
        print(f'[GST][WARNING] {w} dbg={dbg}', file=sys.stderr, flush=True)
    
    def start(self):
        print(f'[INFO] Audio Player initialized', file=sys.stderr, flush=True)
        print(f'[INFO] Listening on UDP port: {self.port}', file=sys.stderr, flush=True)
        print(f'[INFO] Starting audio player...', file=sys.stderr, flush=True)
        
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print('[ERROR] Failed to start pipeline', file=sys.stderr, flush=True)
            sys.exit(1)
        
        print('[INFO] Pipeline is PLAYING', file=sys.stderr, flush=True)
        
        self.loop = GLib.MainLoop()
        try:
            self.loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop()
    
    def stop(self):
        print('[INFO] Stopping audio player...', file=sys.stderr, flush=True)
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        if self.loop and self.loop.is_running():
            self.loop.quit()


def main():
    parser = argparse.ArgumentParser(description='Audio Player - 接收 RTP 音频并播放')
    parser.add_argument('--port', type=int, default=5005, help='UDP 端口 (默认: 5005)')
    args = parser.parse_args()
    
    player = AudioPlayer(port=args.port)
    
    # 处理信号
    def signal_handler(sig, frame):
        print('\n[INFO] Received signal, stopping...', file=sys.stderr, flush=True)
        player.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    player.start()


if __name__ == '__main__':
    main()

