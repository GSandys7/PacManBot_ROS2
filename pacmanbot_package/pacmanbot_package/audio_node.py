# node for playing sounds on the Create 3 based on messages for the game state

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from builtin_interfaces.msg import Duration
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from irobot_create_msgs.action import AudioNoteSequence

from pacmanbot_package.audio_library import SONGS


'''
How another node would use this to play a sound


from std_msgs.msg import String

msg = String()
msg.data = "pellet"
sound_pub.publish(msg)

'''

def midi_to_freq(midi_note: int) -> int:
    return int(round(440.0 * (2.0 ** ((midi_note - 69) / 12.0))))


class AudioNode(Node):
    def __init__(self):
        super().__init__('audio_node')

        self.action_client = ActionClient(
            self,
            AudioNoteSequence,
            '/robot_15/audio_note_sequence'
        )

        self.sub = self.create_subscription(
            String,
            '/robot_15/game_sound',
            self.sound_callback,
            10
        )

        self.busy = False
        self.get_logger().info('Audio node ready')

    def sound_callback(self, msg: String):
        sound_name = msg.data.strip()
        self.get_logger().info(f'Received sound request: {sound_name}')

        if sound_name not in SONGS:
            self.get_logger().warning(f'Unknown sound: {sound_name}')
            return

        if self.busy:
            self.get_logger().info(f'Ignoring sound "{sound_name}" because audio is busy')
            return

        melody = SONGS[sound_name]
        self.play_melody(melody)

    def make_note(self, midi_note: int, duration_sec: float) -> AudioNote:
        note = AudioNote()
        note.frequency = midi_to_freq(midi_note)

        dur = Duration()
        dur.sec = int(duration_sec)
        dur.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        note.max_runtime = dur

        return note

    def play_melody(self, melody):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Audio action server not available')
            return

        seq = AudioNoteVector()
        seq.append = False
        seq.notes = [self.make_note(note, dur) for note, dur in melody]

        goal = AudioNoteSequence.Goal()
        goal.iterations = 1
        goal.note_sequence = seq

        self.busy = True
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warning('Audio goal rejected')
            self.busy = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.busy = False
        self.get_logger().info('Finished playing sound')


def main(args=None):
    rclpy.init(args=args)
    node = AudioNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()