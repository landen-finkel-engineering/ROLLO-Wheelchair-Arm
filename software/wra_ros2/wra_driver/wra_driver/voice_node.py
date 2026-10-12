"""Voice commands from the ReSpeaker USB mic array (offline Vosk ASR with a fixed grammar).

    ros2 run wra_driver voice_node --ros-args -p model_path:=$HOME/vosk-model-small-en-us-0.15

"stop" / "freeze" always works (no wake word) and cancels any motion: the arm holds position.
Every other command must start with the wake word (default "robot"), e.g. "robot open the door",
"robot go home", "robot open hand", "robot close hand", "robot arm on", "robot arm off".
Voice is a CONVENIENCE input: the hard-wired E-stop on the pendant is the safety function.
"robot open the door" does not ask about the wheel locks: set both before saying it (manual chair, docs/10).
"""
from __future__ import annotations

import json
import queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float32, String
from std_srvs.srv import SetBool, Trigger

PHRASES = ["stop", "freeze", "arm on", "arm off", "go home", "park", "open the door", "open hand", "close hand"]


class VoiceNode(Node):
    def __init__(self):
        super().__init__("wra_voice")
        self.declare_parameter("model_path", "")
        self.declare_parameter("device", "ReSpeaker")
        self.declare_parameter("wake_word", "robot")
        self.declare_parameter("rate", 16000)
        self.wake = self.get_parameter("wake_word").value
        self.pub_stop = self.create_publisher(Empty, "/wra/stop", 10)
        self.pub_claw = self.create_publisher(Float32, "/wra/claw", 10)
        self.pub_heard = self.create_publisher(String, "/wra/voice_heard", 10)
        self.cli_arm = self.create_client(SetBool, "/wra/arm")
        self.cli_home = self.create_client(Trigger, "/wra/home")
        self.cli_door = self.create_client(Trigger, "/wra/open_door")
        self.q = queue.Queue()
        import sounddevice as sd
        import vosk
        mp = self.get_parameter("model_path").value
        if not mp:
            raise RuntimeError("set model_path to an unpacked Vosk model (e.g. vosk-model-small-en-us-0.15)")
        grammar = json.dumps(PHRASES + [f"{self.wake} {p}" for p in PHRASES] + ["[unk]"])
        rate = int(self.get_parameter("rate").value)
        self.rec = vosk.KaldiRecognizer(vosk.Model(mp), rate, grammar)
        dev = None
        for i, d in enumerate(sd.query_devices()):
            if self.get_parameter("device").value.lower() in d["name"].lower() and d["max_input_channels"] > 0:
                dev = i
                break
        self.stream = sd.RawInputStream(samplerate=rate, blocksize=4000, device=dev, dtype="int16", channels=1,
                                        callback=lambda data, *_: self.q.put(bytes(data)))
        self.stream.start()
        self.create_timer(0.05, self.poll)
        self.get_logger().info(f"listening (device {dev}); wake word '{self.wake}'")

    def poll(self):
        while not self.q.empty():
            if self.rec.AcceptWaveform(self.q.get()):
                text = json.loads(self.rec.Result()).get("text", "")
                if text and text != "[unk]":
                    self.handle(text)

    def handle(self, text: str):
        self.pub_heard.publish(String(data=text))
        words = text.split()
        if "stop" in words or "freeze" in words:
            self.pub_stop.publish(Empty())
            self.get_logger().warn("voice STOP")
            return
        if not text.startswith(self.wake + " "):
            return
        cmd = text[len(self.wake) + 1:]
        self.get_logger().info(f"voice command: {cmd}")
        if cmd in ("go home", "park"):
            self.cli_home.call_async(Trigger.Request())
        elif cmd == "open the door":
            self.cli_door.call_async(Trigger.Request())
        elif cmd == "open hand":
            self.pub_claw.publish(Float32(data=65.0))
        elif cmd == "close hand":
            self.pub_claw.publish(Float32(data=0.0))
        elif cmd in ("arm on", "arm off"):
            self.cli_arm.call_async(SetBool.Request(data=cmd == "arm on"))


def main():
    rclpy.init()
    n = VoiceNode()
    try:
        rclpy.spin(n)
    finally:
        n.stream.stop()
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
