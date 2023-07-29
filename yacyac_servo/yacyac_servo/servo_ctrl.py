import rclpy as rp
from rclpy.node import Node

from yacyac_interface.msg import Servo
from dynamixel_sdk_custom_interfaces.msg import SetPosition
from dynamixel_sdk_custom_interfaces.srv import GetPosition

# 목적 포지션을 받으면 그에 맞게 서보를 움직임
# 목적 포지션에 도달하면 도착 메시지를 보냄


class ServoCtrl(Node):
    def __init__(self):
        super().__init__("servo_ctrl")
        self.sub = self.create_subscription(Servo, "/yacyac/servo", self.callback_yac, 10)
        self.pub = self.create_publisher(SetPosition, "/set_position", 20)
        self.cli = self.create_client(GetPosition, "/get_position")

        self.cmd_pose_id = Servo()
        
        # 목적 포지션 list
        self.position_set = [0, 455, 910, 1365, 1820, 2275, 2730, 3185, 3640, 4095]
        # 방향 flag
        self.position_flag = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        # 현재 포지션 index
        self.position_cnt = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        for i in range(8):
            self.init_position(i+1)


    def init_position(self, id):
        req = GetPosition.Request()
        req.id = id
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

        future = self.cli.call_async(req)
        rp.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()

            closest_idx = 0
            min_diff = abs(response.position - self.position_set[id-1])
            for i, val in enumerate(self.position_set):
                diff = abs(response.position - val)
                if diff < min_diff:
                    min_diff = diff
                    closest_idx = i
            
            cmd_pose = SetPosition()
            cmd_pose.id = id
            cmd_pose.position = self.position_set[closest_idx]
            
            # closest_idx이 중앙값을 넘는경우 
            if closest_idx > 5:
                # 반시계 방향으로 돌아야함
                self.position_flag[closest_idx] = 1
                self.position_cnt[id-1] = closest_idx 
            else:
                # 시계 방향으로 돌아야함
                self.position_flag[closest_idx] = 0
                self.position_cnt[id-1] = closest_idx

            print("cmd_pose.position : ", cmd_pose.position)
            self.pub.publish(cmd_pose)
        else:
            self.get_logger().info(f"Service call for ID {id} failed")
   


    def control_position(self, id, servo):
        # 1번 서보 제어
        flag = int(servo)
        if flag != 0:
            # 포지션이 끝에 도달하면 방향을 바꿈
            # 역방향 진행
            if self.position_cnt[id] > 0 and self.position_flag[id] == 1:
                cmd_pose = SetPosition()
                self.position_flag[id] = 1
                self.position_cnt[id] -= 1
                cmd_pose.position = self.position_set[self.position_cnt[id]]
                cmd_pose.id = id + 1
                self.pub.publish(cmd_pose)
                if self.position_cnt[id] == 0:
                    self.position_flag[id] = 0
                    self.position_cnt[id] = 1
            # 정방향 진행
            elif self.position_cnt[id] < 9 and self.position_flag[id] == 0:
                cmd_pose = SetPosition()
                self.position_flag[id] = 0
                self.position_cnt[id] += 1
                cmd_pose.position = self.position_set[self.position_cnt[id]]
                cmd_pose.id = id + 1
                self.pub.publish(cmd_pose)
                if self.position_cnt[id] == 9:
                    self.position_flag[id] = 1
                    self.position_cnt[id] = 8

    def callback_yac(self, msg):
        servo_list = list(msg.servo_list)
        for idx in range(len(servo_list)):
            self.control_position(idx, servo_list[idx])


def main(args=None):
    rp.init(args=args)

    servo_node = ServoCtrl()
    rp.spin(servo_node)

    servo_node.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    print("start servo_node")
    main()