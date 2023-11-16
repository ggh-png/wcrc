from setuptools import setup

package_name = 'wcrc_ctrl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch
        ('share/' + package_name + '/launch',
         ['launch/position_ctrl.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ggh',
    maintainer_email='ggh@example.com',  # 유효한 이메일 주소로 변경하세요.
    # 실제 설명으로 변경하세요.
    description='The wcrc_ctrl package is for controlling WCRC robots.',
    license='Apache 2.0',  # 실제 사용하는 라이선스로 변경하세요.
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'MovingControl = wcrc_ctrl.MovingControl:main',
            'main = wcrc_ctrl.main:main',
        ],
    },
)
