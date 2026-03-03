from setuptools import find_packages, setup

package_name = 'slvrov_nodes_python'

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
    maintainer='LegionaryOfLogic',
    maintainer_email='calebhof11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pca9685_node=slvrov_nodes_python.pca9685_node:main',
            'pca9685_pin_configs_server=slvrov_nodes_python.pca9685_pin_configs_server:main',
            'pca9685_pin_configs_client=slvrov_nodes_python.pca9685_pin_configs_client:main'
        ],
    },
)
