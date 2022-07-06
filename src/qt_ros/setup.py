from setuptools import setup

package_name = 'qt_ros'
submodules = 'qt_ros/submodules'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lpc',
    maintainer_email='lpc@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_gui = qt_ros.main_gui:main',
            'display_gui = qt_ros.display_gui:main',
        ],
    },
)
