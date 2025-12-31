#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import Trigger
import serial
import re
import subprocess
import time as pytime
from collections import deque


class BatteryLEDBridge(Node):
    def __init__(self):
        super().__init__('battery_led_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/tiny2040')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('voltage_min', 9.0)
        self.declare_parameter('voltage_max', 12.6)
        self.declare_parameter('warning_voltage', 10.5)  # 3.5V/cell - good margin
        self.declare_parameter('critical_voltage', 9.0)   # 3.0V/cell - shutdown
        self.declare_parameter('shutdown_delay', 10.0)
        self.declare_parameter('warning_interval', 60.0)  # Seconds between warnings
        self.declare_parameter('voltage_samples', 20)     # Number of samples to average

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.v_min = self.get_parameter('voltage_min').value
        self.v_max = self.get_parameter('voltage_max').value
        self.v_warn = self.get_parameter('warning_voltage').value
        self.v_crit = self.get_parameter('critical_voltage').value
        self.shutdown_delay = self.get_parameter('shutdown_delay').value
        self.warning_interval = self.get_parameter('warning_interval').value
        voltage_samples = self.get_parameter('voltage_samples').value

        # Voltage averaging buffer
        self.voltage_buffer = deque(maxlen=voltage_samples)

        # Battery publisher
        self.battery_pub = self.create_publisher(BatteryState, 'battery_state', 10)
        
        # LED mode publisher (for feedback to web UI)
        self.led_status_pub = self.create_publisher(String, 'led_status', 10)
        
        # TTS publisher (Pi Zero Piper)
        self.tts_pub = self.create_publisher(String, '/zero/speak_text', 10)

        # LED control subscribers
        self.led_mode_sub = self.create_subscription(
            String, 'led_mode', self.led_mode_callback, 10)
        self.led_color_sub = self.create_subscription(
            ColorRGBA, 'led_color', self.led_color_callback, 10)
        self.led_brightness_sub = self.create_subscription(
            String, 'led_brightness', self.led_brightness_callback, 10)
        self.led_speed_sub = self.create_subscription(
            String, 'led_speed', self.led_speed_callback, 10)

        # Shutdown service (for web UI to trigger shutdown)
        self.shutdown_service = self.create_service(
            Trigger, '/robot/shutdown', self.manual_shutdown_callback)
        
        # Client to shutdown Pi Zero first
        self.zero_shutdown_client = self.create_client(Trigger, '/zero/shutdown')

        self.shutdown_triggered = False
        self.shutdown_timer = None
        self.current_mode = 'idle'
        self.warning_triggered = False
        self.last_warning_time = 0.0
        self.critical_warned = False

        # Serial connection
        try:
            self.ser = serial.Serial(
                port, baud, timeout=1.0, dsrdtr=False, rtscts=False)
            import time
            time.sleep(2.0)
            self.get_logger().info(f'Connected to {port} at {baud} baud')
            self.get_logger().info(f'Voltage averaging over {voltage_samples} samples')
            
            # Set initial mode
            self.send_command('MODE:idle')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.timer = self.create_timer(0.1, self.read_serial)

    def get_averaged_voltage(self, new_reading):
        """Add reading to buffer and return averaged voltage."""
        self.voltage_buffer.append(new_reading)
        return sum(self.voltage_buffer) / len(self.voltage_buffer)

    def send_command(self, cmd):
        """Send command to Tiny 2040 over serial."""
        try:
            self.ser.write(f'{cmd}\n'.encode())
            self.get_logger().debug(f'Sent: {cmd}')
        except Exception as e:
            self.get_logger().error(f'Serial write error: {e}')

    def speak(self, text):
        """Send text to Pi Zero for TTS via Piper."""
        msg = String()
        msg.data = text
        self.tts_pub.publish(msg)
        self.get_logger().info(f'TTS: {text}')

    def led_mode_callback(self, msg):
        """
        Set LED mode. Valid modes from Tiny 2040 firmware:
        - idle: Scanner sweep (default)
        - speaking: Ripples from center
        - thinking: Rotating gradient
        - happy: Heartbeat pulse
        - sad: Tears dripping
        - angry: Fire flames
        - excited: Rainbow sparkles
        - searching: Rain drops
        - alert: Red flash
        - low_battery: Yellow flash
        - critical: Fast red flash
        - off: LEDs off
        """
        mode = msg.data.lower().strip()
        valid_modes = ['idle', 'speaking', 'thinking', 'happy', 'sad', 'angry', 
                       'excited', 'searching', 'alert', 'low_battery', 'critical', 'off']
        
        if mode in valid_modes:
            self.current_mode = mode
            self.send_command(f'MODE:{mode}')
            self.get_logger().info(f'LED mode set to: {mode}')
            
            # Publish status feedback
            status_msg = String()
            status_msg.data = mode
            self.led_status_pub.publish(status_msg)
        else:
            self.get_logger().warn(f'Invalid LED mode: {mode}. Valid: {valid_modes}')

    def led_color_callback(self, msg):
        """Set LED color for animations. RGBA values 0.0-1.0"""
        r = int(msg.r * 255)
        g = int(msg.g * 255)
        b = int(msg.b * 255)
        self.send_command(f'COLOR:{r},{g},{b}')
        self.get_logger().info(f'LED color set to RGB({r},{g},{b})')

    def led_brightness_callback(self, msg):
        """Set global LED brightness using DIM:0-7 (0=max, 7=dimmest)."""
        try:
            level = int(msg.data)
            level = max(0, min(7, level))
            self.send_command(f'DIM:{level}')
            self.get_logger().info(f'LED brightness level set to: {level}')
        except ValueError:
            self.get_logger().warn(f'Invalid brightness value: {msg.data}')

    def led_speed_callback(self, msg):
        """Set animation speed (1=slow, 10=fast)."""
        try:
            speed = int(msg.data)
            speed = max(1, min(10, speed))
            self.send_command(f'SPEED:{speed}')
            self.get_logger().info(f'LED animation speed set to: {speed}')
        except ValueError:
            self.get_logger().warn(f'Invalid speed value: {msg.data}')

    def manual_shutdown_callback(self, request, response):
        """Handle shutdown request from web UI."""
        self.get_logger().warn('Received manual shutdown request from Web UI.')
        
        # Use the same graceful shutdown sequence
        self.trigger_shutdown()
        
        response.success = True
        response.message = "Initiating shutdown sequence: Zero first, then Pi4..."
        return response

    def trigger_shutdown(self):
        """Initiate graceful shutdown sequence: Zero first, then Pi4."""
        self.get_logger().error('INITIATING SHUTDOWN SEQUENCE...')
        
        # Use alert mode (red flash)
        self.send_command('MODE:alert')
        
        # Announce shutdown
        self.speak('Initiating shutdown sequence. Goodbye, sir.')
        pytime.sleep(2.0)  # Let TTS finish
        
        # Shutdown Pi Zero first
        self.shutdown_zero()
        
        # Wait for Zero to shutdown (give it 10 seconds)
        self.get_logger().info('Waiting for Pi Zero to shutdown...')
        pytime.sleep(10.0)
        
        # Now shutdown Pi4
        self.shutdown_pi4()

    def shutdown_zero(self):
        """Send shutdown command to Pi Zero."""
        self.get_logger().info('Sending shutdown to Pi Zero...')
        
        if not self.zero_shutdown_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Pi Zero shutdown service not available')
            return False
        
        try:
            request = Trigger.Request()
            future = self.zero_shutdown_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                self.get_logger().info(f'Zero shutdown: {future.result().message}')
                return True
            else:
                self.get_logger().warn('Zero shutdown call failed')
                return False
        except Exception as e:
            self.get_logger().error(f'Error shutting down Zero: {e}')
            return False

    def shutdown_pi4(self):
        """Shutdown the Pi4."""
        self.get_logger().error('Shutting down Pi4...')
        try:
            subprocess.run(['sudo', 'shutdown', '-h', 'now'], check=True)
        except Exception as e:
            self.get_logger().error(f'Failed to shutdown Pi4: {e}')

    def read_serial(self):
        """Read battery voltage from Tiny 2040."""
        try:
            if self.ser.in_waiting:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()

                # Parse "VOLTAGE:12.345" format
                match = re.match(r'VOLTAGE:([\d.]+)', line)
                if match:
                    raw_voltage = float(match.group(1))
                    
                    # Get averaged voltage
                    voltage = self.get_averaged_voltage(raw_voltage)

                    msg = BatteryState()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.voltage = voltage
                    msg.current = float('nan')
                    msg.charge = float('nan')
                    msg.capacity = float('nan')
                    msg.design_capacity = float('nan')

                    # Calculate percentage (0-100%)
                    percentage = (voltage - self.v_min) / (self.v_max - self.v_min) * 100.0
                    msg.percentage = max(0.0, min(100.0, percentage))

                    current_time = self.get_clock().now().nanoseconds / 1e9

                    # Warning voltage - gentle reminder via TTS
                    if voltage <= self.v_warn and voltage > self.v_crit:
                        if not self.warning_triggered:
                            # First warning
                            self.warning_triggered = True
                            self.last_warning_time = current_time
                            self.speak(f'Sir, battery is getting low at {percentage:.0f} percent. '
                                      f'I recommend returning to the charging station soon.')
                            # Use firmware's low_battery mode (yellow flash)
                            self.send_command('MODE:low_battery')
                            self.get_logger().warn(f'LOW BATTERY WARNING: {voltage:.2f}V ({percentage:.0f}%)')
                        
                        elif current_time - self.last_warning_time >= self.warning_interval:
                            # Periodic reminder
                            self.last_warning_time = current_time
                            self.speak(f'Battery reminder: {percentage:.0f} percent remaining.')
                            self.get_logger().warn(f'LOW BATTERY REMINDER: {voltage:.2f}V ({percentage:.0f}%)')
                    
                    elif voltage > self.v_warn and self.warning_triggered:
                        # Voltage recovered above warning level
                        self.warning_triggered = False
                        self.send_command(f'MODE:{self.current_mode}')
                        self.get_logger().info('Battery voltage recovered above warning level')

                    # Critical voltage shutdown logic
                    if voltage <= self.v_crit and not self.shutdown_triggered:
                        self.get_logger().error(
                            f'CRITICAL BATTERY: {voltage:.2f}V! '
                            f'Shutting down in {self.shutdown_delay} seconds...')
                        self.shutdown_triggered = True
                        
                        # Urgent TTS warning
                        if not self.critical_warned:
                            self.critical_warned = True
                            self.speak('Warning! Battery critical. Initiating emergency shutdown to protect the cells.')
                        
                        # Use firmware's critical mode (fast red flash)
                        self.send_command('MODE:critical')
                        
                        self.shutdown_timer = self.create_timer(
                            self.shutdown_delay, self.trigger_shutdown)

                    elif voltage > self.v_crit and self.shutdown_triggered:
                        self.get_logger().info('Battery voltage recovered, canceling shutdown')
                        self.shutdown_triggered = False
                        self.critical_warned = False
                        if self.shutdown_timer:
                            self.shutdown_timer.cancel()
                            self.shutdown_timer = None
                        # Restore normal mode
                        self.send_command(f'MODE:{self.current_mode}')
                        self.speak('Battery voltage recovered. Shutdown cancelled.')

                    # Set power supply status
                    if voltage <= self.v_crit:
                        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_DEAD
                    else:
                        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
                        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD

                    msg.present = True
                    self.battery_pub.publish(msg)
                    
                    self.get_logger().info(
                        f'Battery: {voltage:.2f}V ({msg.percentage:.1f}%)',
                        throttle_duration_sec=5.0)

        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = BatteryLEDBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
