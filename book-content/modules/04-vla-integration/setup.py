from setuptools import setup

package_name = 'vla_integration'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    py_modules=['voice_to_text_converter', 'intent_recognizer', 'voice_to_action_pipeline', 'action_execution_system', 'test_voice_command_execution', 'test_vla_integration', 'vla_integration_main'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/vla_integration_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='developer@example.com',
    description='Vision-Language-Action Integration Package for Voice-Controlled Robotics',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_to_text_converter = voice_to_text_converter:main',
            'intent_recognizer = intent_recognizer:main',
            'voice_to_action_pipeline = voice_to_action_pipeline:main',
            'action_execution_system = action_execution_system:main',
            'test_voice_command_execution = test_voice_command_execution:main',
            'test_vla_integration = test_vla_integration:main',
            'vla_integration_main = vla_integration_main:main',
        ],
    },
)