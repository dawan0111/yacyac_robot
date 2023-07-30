from setuptools import setup

package_name = 'yacyac_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치파일 추가 
        ('share/' + package_name + '/launch', ['launch/yacyac_servo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ggh',
    maintainer_email='0380089@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_ctrl = yacyac_servo.servo_ctrl:main',
        ],
    },
)