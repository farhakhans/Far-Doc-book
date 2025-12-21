// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: [
        'hardware-requirements/index',
        'hardware-requirements/economy-kit',
        'hardware-requirements/edge-kit',
        'hardware-requirements/robot-lab-options',
        'hardware-requirements/workstations',
      ],
    },
    {
      type: 'category',
      label: 'Modules',
      items: [
        'modules/index',
        {
          type: 'category',
          label: 'Module 1: ROS2 Fundamentals',
          items: [
            'modules/module-1-ros2/index',
            'modules/module-1-ros2/nodes-topics-services',
            'modules/module-1-ros2/rclpy',
            'modules/module-1-ros2/urdf',
            'modules/module-1-ros2/practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: Simulation',
          items: [
            'modules/module-2-simulation/index',
            'modules/module-2-simulation/gazebo',
            'modules/module-2-simulation/unity',
            'modules/module-2-simulation/physics-simulation',
            'modules/module-2-simulation/sensor-modeling',
            'modules/module-2-simulation/practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: NVIDIA Isaac',
          items: [
            'modules/module-3-nvidia-isaac/index',
            'modules/module-3-nvidia-isaac/isaac-sim',
            'modules/module-3-nvidia-isaac/isaac-ros',
            'modules/module-3-nvidia-isaac/nav2',
            'modules/module-3-nvidia-isaac/practical-exercises',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action',
          items: [
            'modules/module-4-vla/index',
            'modules/module-4-vla/vision-language-action',
            'modules/module-4-vla/llms',
            'modules/module-4-vla/whisper',
            'modules/module-4-vla/practical-exercises',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Weekly Breakdown',
      items: [
        'weekly-breakdown/week-1',
        'weekly-breakdown/week-2',
        'weekly-breakdown/week-3',
        'weekly-breakdown/week-4',
        'weekly-breakdown/week-5',
        'weekly-breakdown/week-6',
        'weekly-breakdown/week-7',
        'weekly-breakdown/week-8',
        'weekly-breakdown/week-9',
        'weekly-breakdown/week-10',
        'weekly-breakdown/week-11',
        'weekly-breakdown/week-12',
        'weekly-breakdown/week-13',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      items: [
        'assessments/index',
        'assessments/capstone-project',
        'assessments/isaac-project',
        'assessments/ros2-project',
        'assessments/simulation-project',
      ],
    },
    {
      type: 'category',
      label: 'Models',
      items: [
        'models/index',
        'models/physical-ai',
        'models/humanoid-robotics',
      ],
    },
  ],
};

export default sidebars;