from setuptools import find_packages, setup

package_name = 'hitl'

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
    maintainer='widha893',
    maintainer_email='widha.rizqika.prasetya@mail.ugm.ac.id',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'communication = hitl.communication:main',
            'receiver = hitl.receiver:main',
            'hitl_node = hitl.hitl_node:main',
            'data_analyzer = hitl.data_analyzer:main',
            'controller = hitl.quadrotor_controller:main'
        ],
    },
)
