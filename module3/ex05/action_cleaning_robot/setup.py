from setuptools import find_packages, setup

package_name = 'action_cleaning_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Важно! Эта строка сообщает, где найти ваш action-файл
        ('share/' + package_name + '/action', ['action/CleaningTask.action']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@todo.todo',
    description='A cleaning robot action server and client',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Это самая важная часть! Она делает ваши .py файлы исполняемыми узлами.
    entry_points={
        'console_scripts': [
            'cleaning_action_server = action_cleaning_robot.cleaning_action_server:main',
            'cleaning_action_client = action_cleaning_robot.cleaning_action_client:main',
        ],
    },
)