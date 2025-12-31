import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import socket
import subprocess
import threading
import json
import time
from typing import Dict, Optional
import signal

# Use relative path for launch directory
DEFAULT_LAUNCH_DIR = os.path.join(os.getcwd(), 'tank_bot', 'launch')


class RosLauncherNode(Node):
    def __init__(self):
        super().__init__('ros_launcher_node')

        # <<< FIX: Use a ROS parameter for the device name, not the hostname
        self.declare_parameter('device_name', 'default_device')
        self.device_name = self.get_parameter('device_name').value
        # ---

        self.declare_parameter('launch_dir', DEFAULT_LAUNCH_DIR)
        self.declare_parameter('poll_interval', 5.0)

        self.launch_dir = self.get_parameter('launch_dir').value
        self.poll_interval = float(self.get_parameter('poll_interval').value)

        # Publishers
        self.avail_pub = self.create_publisher(
            String, '/available_launches', 10)
        self.status_pub = self.create_publisher(String, '/launch_status', 10)
        self.output_pub = self.create_publisher(String, '/launch_output', 10)

        # Subscriber for commands
        self.cmd_sub = self.create_subscription(
            String, '/launch_command', self.command_cb, 10)

        # Active processes
        self.active: Dict[str, Dict] = {}

        # Start a background thread
        self._stop_event = threading.Event()
        self._discovery_thread = threading.Thread(
            target=self._discovery_loop, daemon=True)
        self._discovery_thread.start()

        self.get_logger().info(
            f"ros_launcher_node started as '{self.device_name}', watching {self.launch_dir}")

    def _discovery_loop(self):
        last_list = None
        while not self._stop_event.is_set() and rclpy.ok():
            launches = self._list_launch_files()
            if launches != last_list:
                msg = {
                    # <<< FIX: Tag message with our device_name
                    'device': self.device_name,
                    'type': 'launch_list',
                    'launches': launches
                }
                self.avail_pub.publish(String(data=json.dumps(msg)))
                last_list = launches
            time.sleep(self.poll_interval)

    def _list_launch_files(self):
        # ... (This function is unchanged) ...
        if not os.path.isdir(self.launch_dir):
            return []
        files = []
        try:
            for f in os.listdir(self.launch_dir):
                if f.endswith('.launch.py'):
                    files.append(f)
        except Exception as e:
            self.get_logger().error(f'Error listing launch dir: {e}')
        files.sort()
        return files

    def command_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'Bad JSON in /launch_command: {e}')
            return

        action = data.get('action')
        file = data.get('file')
        # The device this command is FOR
        device_target = data.get('device', 'all')
        use_sim_time = data.get('use_sim_time', False)

        # <<< FIX: Check against our device_name, not hostname
        if device_target != 'all' and device_target != self.device_name:
            # This command is not for us, ignore it.
            return

        # Log that we are acting on the command
        self.get_logger().info(f"Received action '{action}' for file '{file}'")

        if action == 'launch' and file:
            self.start_launch(file, use_sim_time)
        elif action == 'stop' and file:
            self.stop_launch(file)
        elif action == 'list':
            launches = self._list_launch_files()
            msg = {
                # <<< FIX: Tag message with our device_name
                'device': self.device_name,
                'type': 'launch_list',
                'launches': launches
            }
            self.avail_pub.publish(String(data=json.dumps(msg)))
        else:
            self.get_logger().warn(f'Unknown command: {data}')

    def start_launch(self, filename: str, use_sim_time: bool = False):
        # ... (This function is unchanged) ...
        if filename in self.active:
            self._publish_status(filename, 'already_running')
            return
        full = os.path.join(self.launch_dir, filename)
        if not os.path.exists(full):
            self._publish_status(filename, 'not_found')
            return
        try:
            cmd = ['ros2', 'launch', full]
            if use_sim_time:
                cmd.append('use_sim_time:=true')
            else:
                cmd.append('use_sim_time:=false')
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        except Exception as e:
            self._publish_status(filename, f'start_failed: {e}')
            return
        sim_time_str = 'true' if use_sim_time else 'false'
        self._publish_status(
            filename, f'started pid={process.pid} use_sim_time={sim_time_str}')
        t = threading.Thread(target=self._reader_thread, args=(
            filename, process), daemon=True)
        t.start()
        self.active[filename] = {'proc': process, 'thread': t}

    def stop_launch(self, filename: str):
        # ... (This function is unchanged) ...
        info = self.active.get(filename)
        if not info:
            self._publish_status(filename, 'not_running')
            return
        proc: subprocess.Popen = info['proc']
        try:
            self.get_logger().info(
                f"Sending SIGINT to {filename} (pid={proc.pid})")
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.get_logger().warn(
                f"SIGINT failed. Sending SIGTERM to {filename} (pid={proc.pid})")
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    f"SIGTERM failed. Sending SIGKILL to {filename} (pid={proc.pid})")
                proc.kill()
        except Exception as e:
            self.get_logger().warn(f'Exception during stop: {e}')
            pass

    def _reader_thread(self, filename: str, process: subprocess.Popen):
        # ... (This function is unchanged) ...
        try:
            assert process.stdout is not None
            while process.poll() is None and rclpy.ok():
                line = process.stdout.readline()
                if not line:
                    time.sleep(0.01)
                    continue
                self._publish_output(filename, line.rstrip('\n'))
            if process.stdout:
                for line in process.stdout:
                    self._publish_output(filename, line.rstrip('\n'))
        except Exception as e:
            self.get_logger().error(f'reader thread error for {filename}: {e}')
        rc = process.poll()
        if rc is None:
            rc = -1
        self._publish_status(filename, f'exited rc={rc}')
        if filename in self.active:
            del self.active[filename]
            self.get_logger().info(f"Cleaned up exited process: {filename}")

    def _publish_status(self, filename: str, status: str):
        payload = {
            # <<< FIX: Tag message with our device_name
            'device': self.device_name,
            'type': 'status',
            'file': filename,
            'status': status
        }
        self.status_pub.publish(String(data=json.dumps(payload)))

    def _publish_output(self, filename: str, line: str):
        payload = {
            # <<< FIX: Tag message with our device_name
            'device': self.device_name,
            'type': 'output',
            'file': filename,
            'line': line
        }
        self.output_pub.publish(String(data=json.dumps(payload)))

    def destroy_node(self):
        # ... (This function is unchanged) ...
        self._stop_event.set()
        for filename, info in list(self.active.items()):
            try:
                info['proc'].send_signal(signal.SIGINT)
            except Exception:
                pass
            try:
                info['proc'].terminate()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RosLauncherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down launcher node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
