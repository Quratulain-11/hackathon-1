// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro/index',
    'intro/overview',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module-1-ros-nervous-system/index',
        'module-1-ros-nervous-system/package-structure'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin',
      items: [
        'module-2-digital-twin/index',
        'module-2-digital-twin/urdf-modeling'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Robot Brain',
      items: [
        'module-3-ai-robot-brain/index'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module-4-vision-language-action/index'
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/index'
      ],
    },
  ],
};

module.exports = sidebars;