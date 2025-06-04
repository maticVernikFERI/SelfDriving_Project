from setuptools import find_packages, setup

package_name = 'car_controls'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'pigpio',
                      'pynput',],
    zip_safe=True,
    maintainer='matic',
    maintainer_email='matic.vernik1@student.um.si',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_node = car_controls.car_node:main',
            'keyboard_control = car_controls.keyboard_control_node:main',
            'simple_driver = car_controls.simple_driver:main',
        ],
    },
)
