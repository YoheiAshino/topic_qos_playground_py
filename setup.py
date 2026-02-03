from setuptools import find_packages, setup

package_name = 'topic_qos_playground_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    	('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    	('share/' + package_name, ['package.xml']),
    	('share/' + package_name + '/launch', ['launch/demo.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ashino',
    maintainer_email='yohei.ashino@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    	'console_scripts': [
    		'qos_talker_py = topic_qos_playground_py.talker:main',
    		'qos_listener_py = topic_qos_playground_py.listener:main',
    	],
    },
)
