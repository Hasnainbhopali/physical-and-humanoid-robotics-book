// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Physical AI & Humanoid Robotics',
      items: [
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          items: [
            'modules/physical-ai-robotics/module-1-ros2-nervous-system/module-overview',
            'modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-1-ros2-fundamentals',
            'modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-2-ai-agents-ros2',
            'modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-3-robot-anatomy-urdf',
            'modules/physical-ai-robotics/module-1-ros2-nervous-system/glossary'
          ],
        },
      ],
    },
  ],
};

export default sidebars;