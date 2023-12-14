from setuptools import find_packages, setup

package_name = 'neato_tag'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/bringup.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bmorris',
    maintainer_email='benmor2020@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'allconnect = neato_tag.allconnect:main',
            'lidar_detector = neato_tag.lidar_detector:main',
            'camera_detector = neato_tag.camera_detector:main',
            'tagging = neato_tag.tagging:main',
            'navigator=neato_tag.navigator:main'
        ],
    },
)
