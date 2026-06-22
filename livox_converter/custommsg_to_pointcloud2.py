#!/usr/bin/env python3
"""
custommsg_to_pointcloud2.py

Converte livox_ros_driver2/CustomMsg  ->  sensor_msgs/PointCloud2
no formato PointXYZRTLT (x, y, z, intensity, tag, line, timestamp),
exatamente o mesmo que o driver publica quando xfer_format = 0.

É o caminho inverso do pointCloud_converter2.py.

Vantagem: o timestamp de cada ponto é reconstruido a partir de
timebase + offset_time. Assim NAO se perde a informacao de tempo
individual de cada ponto (que no Mid-360 o PointCloud2 do driver
costuma "achatar" para um unico valor por frame). Nenhum dado do
CustomMsg e descartado.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField
from livox_ros_driver2.msg import CustomMsg


# Layout identico ao PointXYZRTLT do livox_ros_driver2.
# A PointCloud2 e auto-descritiva: qualquer leitor (pcl::fromROSMsg,
# os iterators de sensor_msgs, read_points...) usa estes offsets/nomes,
# entao o empacotamento "apertado" de 26 bytes e totalmente valido.
_FIELDS = [
    PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name='tag',       offset=16, datatype=PointField.UINT8,   count=1),
    PointField(name='line',      offset=17, datatype=PointField.UINT8,   count=1),
    PointField(name='timestamp', offset=18, datatype=PointField.FLOAT64, count=1),
]
_POINT_STEP = 26  # 4+4+4+4+1+1+8, sem padding

# dtype numpy com os mesmos offsets -> os bytes batem com _FIELDS
_DTYPE = np.dtype({
    'names':    ['x', 'y', 'z', 'intensity', 'tag', 'line', 'timestamp'],
    'formats':  ['<f4', '<f4', '<f4', '<f4', 'u1', 'u1', '<f8'],
    'offsets':  [0, 4, 8, 12, 16, 17, 18],
    'itemsize': _POINT_STEP,
})


class CustomMsgToPointCloud2(Node):
    def __init__(self):
        super().__init__('custommsg_to_pointcloud2')

        # Topicos e modo de timestamp configuraveis por parametro
        self.declare_parameter('input_topic', '/livox/lidar')
        self.declare_parameter('output_topic', '/livox/lidar_pc2')
        # 'absolute_ns' -> timebase + offset_time, em ns  (igual ao driver)
        # 'absolute_s'  -> o mesmo, porem em segundos
        # 'relative_s'  -> apenas offset_time, em segundos (varios SLAMs usam isso)
        self.declare_parameter('timestamp_mode', 'absolute_ns')

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value
        self.ts_mode = self.get_parameter('timestamp_mode').value

        # depth 10 padrao (reliable) - mesmo QoS do seu conversor que ja funciona.
        # Se nao chegar nenhum dado, o driver pode estar publicando em sensor-data
        # QoS (best-effort); nesse caso troque para um QoSProfile best_effort.
        self.sub = self.create_subscription(CustomMsg, in_topic, self.callback, 10)
        self.pub = self.create_publisher(PointCloud2, out_topic, 10)

        self.get_logger().info(
            f'Convertendo {in_topic} (CustomMsg) -> {out_topic} (PointCloud2), '
            f'timestamp_mode={self.ts_mode}')

    def callback(self, msg: CustomMsg):
        pts = msg.points
        n = len(pts)
        if n == 0:
            return

        arr = np.empty(n, dtype=_DTYPE)
        arr['x']         = np.fromiter((p.x for p in pts),            np.float32, n)
        arr['y']         = np.fromiter((p.y for p in pts),            np.float32, n)
        arr['z']         = np.fromiter((p.z for p in pts),            np.float32, n)
        arr['intensity'] = np.fromiter((p.reflectivity for p in pts), np.float32, n)
        arr['tag']       = np.fromiter((p.tag for p in pts),          np.uint8,   n)
        arr['line']      = np.fromiter((p.line for p in pts),         np.uint8,   n)

        # tempo por ponto: timebase (ns) + offset_time (ns relativo ao timebase)
        offset = np.fromiter((p.offset_time for p in pts), np.uint64, n)
        abs_ns = (np.uint64(msg.timebase) + offset).astype(np.float64)

        if self.ts_mode == 'absolute_s':
            arr['timestamp'] = abs_ns * 1e-9
        elif self.ts_mode == 'relative_s':
            arr['timestamp'] = offset.astype(np.float64) * 1e-9
        else:  # 'absolute_ns'
            arr['timestamp'] = abs_ns

        cloud = PointCloud2()
        cloud.header = msg.header          # mantem stamp e frame_id originais
        cloud.height = 1
        cloud.width = n
        cloud.fields = _FIELDS
        cloud.is_bigendian = False
        cloud.point_step = _POINT_STEP
        cloud.row_step = _POINT_STEP * n
        cloud.is_dense = True
        cloud.data = arr.tobytes()

        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = CustomMsgToPointCloud2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
