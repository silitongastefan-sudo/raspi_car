from setuptools import find_packages, setup

package_name = 'raspi_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='stefan',
    maintainer_email='stefan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'motor_node = raspi_car.motor_node:main',
        'led_service = raspi_car.led_service:main',
        'led_control = raspi_car.led_control:main',
        'motor_pid = raspi_car.motor_pid:main',
        'dead_reckoning = raspi_car.dead_reckoning:main',
        ],
    },
)
