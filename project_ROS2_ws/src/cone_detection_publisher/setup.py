from setuptools import find_packages, setup

package_name = 'cone_detection_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'depthai', 'opencv-python', 'numpy',],
    zip_safe=True,
    maintainer='matic',
    maintainer_email='matic.vernik1@student.um.si',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cone_publisher = cone_detection_publisher.cone_publisher:main',
        ],
    },
)
