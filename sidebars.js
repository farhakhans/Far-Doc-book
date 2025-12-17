// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro', 'intro/overview', 'intro/learning-outcomes'],
    },
    {
      type: 'category',
      label: 'Modules',
      items: [
        'modules/module-1-ros2/index',
        'modules/module-1-ros2/nodes-topics-services',
        'modules/module-1-ros2/rclpy',
        'modules/module-1-ros2/urdf',
        'modules/module-1-ros2/practical-exercises',
        'modules/module-2-simulation/index',
        'modules/module-2-simulation/gazebo',
        'modules/module-2-simulation/unity',
        'modules/module-2-simulation/physics-simulation',
        'modules/module-2-simulation/sensor-modeling',
        'modules/module-2-simulation/practical-exercises',
        'modules/module-3-nvidia-isaac/index',
        'modules/module-3-nvidia-isaac/isaac-sim',
        'modules/module-3-nvidia-isaac/isaac-ros',
        'modules/module-3-nvidia-isaac/nav2',
        'modules/module-3-nvidia-isaac/practical-exercises',
        'modules/module-4-vla/index',
        'modules/module-4-vla/vision-language-action',
        'modules/module-4-vla/whisper',
        'modules/module-4-vla/llms',
        'modules/module-4-vla/practical-exercises',
      ],
    },
    {
      type: 'category',
      label: 'AI Models',
      items: [
        'models/index',
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
        'assessments/ros2-project',
        'assessments/simulation-project',
        'assessments/isaac-project',
        'assessments/capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Hardware Requirements',
      items: [
        'hardware-requirements/index',
        'hardware-requirements/workstations',
        'hardware-requirements/edge-kit',
        'hardware-requirements/robot-lab-options',
        'hardware-requirements/economy-kit',
      ],
    },
    {
      type: 'doc',
      id: 'ai-assistant',
    },
  ],
};

module.exports = sidebars;