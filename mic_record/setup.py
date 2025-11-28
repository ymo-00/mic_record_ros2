from setuptools import setup

package_name = 'mic_record'

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
    maintainer='ymo',
    maintainer_email='ymo@o.cnu.ac.kr',
    description='Record audio when PX4 is armed.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run mic_record mic_record
            'mic_record = mic_record.mic_record:main',
        ],
    },
)
