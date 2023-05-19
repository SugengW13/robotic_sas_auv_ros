#!/usr/bin/env python3

import gi
import numpy as np

gi.require_version('Gst', '1.0')
from gi.repository import Gst

class Video():
    def __init__(self, port=5600):
        Gst.init(None)

        self.port = port
        self.latest_frame = self.new_frame = None
        self.video_source = 'udpsrc port={}'.format(self.port)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        self.video_decode = '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        self.video_sink_conf = '! appsink emit-signals=true sync=false max-buffers=2 drop=true'
        self.video_pipe = None
        self.video_sink = None
        
        self.run()
    
    def start_gst(self, config=None):
        if not config:
            config = [
                'videotestsrc ! decodebin',
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]
        
        command = ' '.join(config)

        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)

        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8
        )

        return array

    def frame(self):
        if self.frame_available:
            self.latest_frame = self.new_frame
            self.new_frame = None

        return self.latest_frame

    def frame_available(self):
        return self.new_frame is not None

    def run(self):
        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ]
        )

        self.video_sink.connect('new-sample', self.callback)
    
    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self.new_frame = self.gst_to_opencv(sample)

        return Gst.FlowReturn.OK