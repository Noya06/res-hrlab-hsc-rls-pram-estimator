#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from evdev import InputDevice, ecodes, ff
import math
import errno

class JoystickFFBNode(Node):
    def __init__(self):
        super().__init__('joystick_ffb_node')

        self.declare_parameter('device_path', '/dev/input/event17')
        device_path = self.get_parameter('device_path').value

        self.device = None
        self.effect_id = None

        self.device = InputDevice(device_path)
        self.get_logger().info(f'Connected to FFB device: {self.device.name} at {device_path}')
        self.setup_ffb_effect()

        # Pゲイン
        self.declare_parameter('p_gain', 5000.0) 
        self.p_gain = self.get_parameter('p_gain').value

        # Dゲイン
        self.declare_parameter('d_gain', 0.0)
        self.d_gain = self.get_parameter('d_gain').value

        # 微分計算用の前回値と時間
        self.prev_target_roll = 0.0
        self.prev_time = self.get_clock().now()

        self.subscription = self.create_subscription(
            Float64,
            '/joystick/target_roll',
            self.target_roll_callback,
            10
        )
        self.get_logger().info('Joystick FFB node is ready.')

    def setup_ffb_effect(self):
        if self.device is None:
            return

        constant = ff.Constant(level=0)
        direction = 0x4000 
        
        effect = ff.Effect(
            ecodes.FF_CONSTANT, -1, direction,
            ff.Trigger(0, 0),
            ff.Replay(0, 0),
            ff.EffectType(ff_constant_effect=constant)
        )
        
        try:
            self.effect_id = self.device.upload_effect(effect)
            self.device.write(ecodes.EV_FF, self.effect_id, 1)
        except Exception as e:
            self.get_logger().error(f'Failed to upload initial FFB effect: {e}')
            self.device = None 

    def target_roll_callback(self, msg):
        if self.device is None or self.effect_id is None:
            return

        current_time = self.get_clock().now()
        target_roll = msg.data

        # 経過時間(dt)と目標値の変化量(derivative)を計算
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        derivative = 0.0
        if dt > 0:
            derivative = (target_roll - self.prev_target_roll) / dt

        self.prev_target_roll = target_roll
        self.prev_time = current_time

        # PゲインとDゲインを合成してフィードバック量を計算
        raw_level = int((target_roll * self.p_gain) + (derivative * self.d_gain))
        
        # 32767 から -32767 の範囲に制限
        level = max(min(raw_level, 32767), -32767)

        try:
            if level == 0:
                constant = ff.Constant(level=0)
                effect = ff.Effect(
                    ecodes.FF_CONSTANT, self.effect_id, 0,
                    ff.Trigger(0, 0),
                    ff.Replay(0, 0),
                    ff.EffectType(ff_constant_effect=constant)
                )
                self.device.upload_effect(effect)
                return

            abs_level = abs(level)
            direction = 0xC000 if level > 0 else 0x4000

            constant = ff.Constant(level=abs_level)
            effect = ff.Effect(
                ecodes.FF_CONSTANT, self.effect_id, direction,
                ff.Trigger(0, 0),
                ff.Replay(0, 0),
                ff.EffectType(ff_constant_effect=constant)
            )
            self.device.upload_effect(effect)
            
        except OSError as e:
            if e.errno == errno.ENODEV:
                self.get_logger().warn('MOZA joystick disconnected during operation. Disabling FFB.')
                self.device = None
            else:
                self.get_logger().warn(f'Failed to update FFB effect: {e}')
        except Exception as e:
            self.get_logger().warn(f'Failed to update FFB effect: {e}')

    def destroy_node(self):
        if self.device is not None and self.effect_id is not None:
            try:
                self.device.write(ecodes.EV_FF, self.effect_id, 0)
                self.device.erase_effect(self.effect_id)
                self.get_logger().info('Erased FFB effect.')
            except Exception as e:
                 self.get_logger().error(f'Error while stopping FFB: {e}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickFFBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()