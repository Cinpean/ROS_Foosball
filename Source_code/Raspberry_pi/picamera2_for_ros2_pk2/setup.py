from setuptools import setup

package_name = 'picamera2_for_ros2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bobo',
    maintainer_email='bobo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cameratotopic_node = picamera2_for_ros2_pkg.cameratotopic_node:main',
            'camerasubscriber = picamera2_for_ros2_pkg.camerasubscriber:main',
            'cameratopic_node_v2 = picamera2_for_ros2_pkg.cameratopic_node_v2:main',
            'Serial_send_publisher = picamera2_for_ros2_pkg.Serial_send_publisher:main',
            'Serial_send_subscribe = picamera2_for_ros2_pkg.Serial_send_subscribe:main',
            'cameratopic_node_v3 = picamera2_for_ros2_pkg.cameratopic_node_v3:main'
        ],
    },
)
