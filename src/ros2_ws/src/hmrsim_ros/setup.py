from setuptools import setup

package_name = 'hmrsim_ros'

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
    maintainer='Kálley Wilkerson',
    maintainer_email='kalleywra@gmail.com',
    description='Ros package that runs the HMRSim simulation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulation = hmrsim_ros.hmrsim_simulation:main'
        ],
    },
)
