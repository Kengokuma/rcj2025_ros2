from setuptools import find_packages, setup

package_name = 'rcj2025_camera'

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
    maintainer='ryota',
    maintainer_email='tyusuke903@gmail.com',
    description='img_detection_publisher',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_detector = rcj2025_camera.img_detector:main'
        ],
    },
)
