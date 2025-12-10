#!/usr/bin/env python3
"""
USB Microphone Audio Streamer
- Captures audio from USB microphone using GStreamer
- Encodes to Opus
- Sends RTP packets to local UDP port for WebRTC consumption
"""

import gi
import sys
import argparse
import signal

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class AudioStreamer:
    def __init__(self, device_name: str = None, bitrate: int = 32000, udp_port: int = 5004):
        """
        Args:
            device_name: PulseAudio source name (e.g., 'alsa_input.usb-...')
                        If None, uses default source
            bitrate: Opus bitrate in bps (default: 32000 = 32kbps)
            udp_port: UDP port to send RTP packets (default: 5004)
        """
        self.device_name = device_name
        self.bitrate = bitrate
        self.udp_port = udp_port
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Build pipeline
        self.pipeline = self._build_pipeline()
        
        # Setup signal handlers
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # GLib main loop
        self.loop = GLib.MainLoop()
        
        print(f'[INFO] Audio Streamer initialized', file=sys.stderr, flush=True)
        print(f'[INFO] Device: {self.device_name or "default"}', file=sys.stderr, flush=True)
        print(f'[INFO] Bitrate: {self.bitrate} bps', file=sys.stderr, flush=True)
        print(f'[INFO] UDP Port: {self.udp_port}', file=sys.stderr, flush=True)
    
    def _build_pipeline(self):
        """
        Build GStreamer pipeline:
        pulsesrc -> audioconvert -> audioresample -> opusenc -> rtpopuspay -> udpsink
        """
        # Source: PulseAudio
        if self.device_name:
            source_str = f'pulsesrc device="{self.device_name}"'
        else:
            source_str = 'pulsesrc'
        
        # Pipeline string
        pipeline_str = (
            f'{source_str} ! '
            f'audioconvert ! '
            f'audioresample ! '
            f'audio/x-raw,rate=48000,channels=1 ! '
            f'opusenc bitrate={self.bitrate} ! '
            f'rtpopuspay ! '
            f'udpsink host=127.0.0.1 port={self.udp_port}'
        )
        
        print(f'[INFO] Pipeline: {pipeline_str}', file=sys.stderr, flush=True)
        
        # Parse and create pipeline
        pipeline = Gst.parse_launch(pipeline_str)
        
        # Connect bus signals
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect('message', self._on_bus_message)
        
        return pipeline
    
    def _on_bus_message(self, bus, message):
        """Handle GStreamer bus messages"""
        t = message.type
        
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f'[ERROR] {err}: {debug}', file=sys.stderr, flush=True)
            self.stop()
        elif t == Gst.MessageType.EOS:
            print('[INFO] End of stream', file=sys.stderr, flush=True)
            self.stop()
        elif t == Gst.MessageType.WARNING:
            warn, debug = message.parse_warning()
            print(f'[WARN] {warn}: {debug}', file=sys.stderr, flush=True)
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                if new_state == Gst.State.PLAYING:
                    print('[INFO] Pipeline is PLAYING', file=sys.stderr, flush=True)
    
    def _signal_handler(self, sig, frame):
        """Handle SIGINT/SIGTERM"""
        print('\n[INFO] Received signal, stopping...', file=sys.stderr, flush=True)
        self.stop()
    
    def start(self):
        """Start the audio streaming pipeline"""
        print('[INFO] Starting audio streamer...', file=sys.stderr, flush=True)
        
        # Set pipeline to PLAYING
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print('[ERROR] Unable to set pipeline to PLAYING', file=sys.stderr, flush=True)
            sys.exit(1)
        
        # Run main loop
        try:
            self.loop.run()
        except KeyboardInterrupt:
            self.stop()
    
    def stop(self):
        """Stop the audio streaming pipeline"""
        print('[INFO] Stopping audio streamer...', file=sys.stderr, flush=True)
        
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        
        if self.loop.is_running():
            self.loop.quit()


def main():
    parser = argparse.ArgumentParser(description='USB Microphone Audio Streamer for WebRTC')
    parser.add_argument('--device', type=str, default=None,
                        help='PulseAudio device name (e.g., alsa_input.usb-...)')
    parser.add_argument('--bitrate', type=int, default=32000,
                        help='Opus bitrate in bps (default: 32000)')
    parser.add_argument('--port', type=int, default=5004,
                        help='UDP port for RTP output (default: 5004)')
    
    args = parser.parse_args()
    
    # Create and start streamer
    streamer = AudioStreamer(
        device_name=args.device,
        bitrate=args.bitrate,
        udp_port=args.port
    )
    
    streamer.start()


if __name__ == '__main__':
    main()



