from setuptools import find_packages, setup

package_name = 'my_camera_pkg'

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
    maintainer='arda',
    maintainer_email='arda@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'cam_node = my_camera_pkg.camera_publisher:main',
        	'scanner_node = my_camera_pkg.datamatrix_scanner:main',
        	'rfid_node = my_camera_pkg.rfid_node:main',
        ],
    },
)
