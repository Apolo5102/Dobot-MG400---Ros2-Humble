from setuptools import find_packages, setup

package_name = 'dobot_mg400'

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
    maintainer='apolo',
    maintainer_email='apolo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dobot = dobot_mg400.move_dobot:main',
            'minimal_publisher = dobot_mg400.prueba_pub:main',
            'img_publisher = dobot_mg400.image_publisher:main',
            'detector_colores = dobot_mg400.Nodo_yolo:main'
        ],
    },
)
