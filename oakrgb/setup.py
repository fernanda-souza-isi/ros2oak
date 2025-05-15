from setuptools import setup

package_name = 'oakrgb'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/oakrgb.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgb_publisher = oakrgb.rgb_publisher:main',
        ],
    },
)
